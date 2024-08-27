/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_redirect.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <math.h>

#include "lcd16x2.h"
#include "config.h"
#include "thermistor.h"
#include "leds.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_VDDA  (3.3)             // V
#define ADC_RANGE (4096)
#define THRM_AVG  (40)

#define LED_RANGE ((2 << 16) - 1)
#define LED_RATE  (100)

#define INT_TEMP_STEP   (4.3)       // mV
#define INT_TEMP_V25    (1430.0)    // mV

#define ADC_THRM_CH     (0)
#define ADC_INTTEMP_CH  (1)
#define ADC_VREF_CH     (2)

// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t adc_results[3] ;
uint16_t thrm_res_buffer[THRM_AVG] ;

static uint16_t thrm_buf_idx = 0 ;
static uint16_t ssr_on_counter = 0 ;
static float plate_set_temp = 25.0 ; 
static float user_set_temp = DEFAULT_TEMP ; 

uint8_t ssr_off_char[8] = {0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f, 0x00} ;
uint8_t ssr_on_char[8]  = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x00} ;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  float adc_v = 0, plate_temp ;
  float therm_rt, int_temp ;
  char temp_update[17] ;
  int32_t last_count ;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1) ;

  HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(SSR_GPIO_Port, SSR_Pin, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_RESET) ;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) ;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) ;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) ;

  init_leds(100) ;
  leds_state_t * orange_led = new_led(16, &(TIM2->CCR1)) ;
  leds_state_t * blue_led = new_led(16, &(TIM2->CCR2)) ;

  set_led(blue_led, 20, 500) ;
  printf("Start!\n\r") ;

  TIM2->CCR3 = 27000 ; // LCD contrast

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) ;
  
  LCD_Init(DispD0_GPIO_Port, IF_8BIT, 
           DispRW_Pin, DispRS_Pin, DispE_Pin,
           DispD0_Pin, DispD1_Pin, DispD2_Pin, DispD3_Pin, 
           DispD4_Pin, DispD5_Pin, DispD6_Pin, DispD7_Pin) ;

  LCD_SetCustomChar(1, ssr_off_char) ;
  LCD_SetCustomChar(2, ssr_on_char) ;

  LCD_SetPosition(LINE_1, 0) ;
  LCD_Print("  Temp.   Set  \x01") ;
//           1234567890123456
  
//  setvbuf(stdout, NULL, _IONBF, 0);
  plate_temp = 0.0 ;
  therm_rt = 0.0 ;
  last_count = TIM1->CNT ;

  pid_config_t * plate_pid = init_pid(20, 0, 0, 0, 200) ;

  uint8_t update_display = 1 ; 

  HAL_TIM_Base_Start_IT(&htim3) ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  //  printf("-----\n\r") ;
    
    if(thrm_buf_idx >= THRM_AVG) {
      float thrm_sum = 0 ;

      // Average ADC readings
      for(int i = 0 ; i < THRM_AVG ; i++) thrm_sum += (float)thrm_res_buffer[i] ;
      thrm_sum /= THRM_AVG ;
      thrm_buf_idx = 0 ;

      adc_v = ADC_VDDA * (thrm_sum / (float)ADC_RANGE) ;

      // Calculate plate temperature
      therm_rt = rt_by_ratio(thrm_sum / (float)ADC_RANGE, THERM_R1) ;
      plate_temp = calculate_temperature(therm_rt, THERM_BETA, THERM_R_T25) ;

      // Update PID and set plate SSR on time
      double pid_res = update_pid(plate_pid, plate_set_temp - plate_temp, plate_temp) ;
      if(pid_res <= 0) ssr_on_counter = 0 ;
      else ssr_on_counter = (uint16_t)pid_res ; 

      update_display = 1 ;
    }
    
  //  adc_v = ADC_VDDA * ((float) adc_results[ADC_INTTEMP_CH] / (float) ADC_RANGE) ;
  //  int_temp = (INT_TEMP_V25 - adc_v * 1000) / INT_TEMP_STEP + 25 ;
  //  printf("IntTemp = %3.1f | VIntTemp = %1.3fV (%d)\n\r", int_temp, adc_v, adc_results[ADC_INTTEMP_CH]) ;

  //  adc_v = ADC_VDDA * ((float) adc_results[2] / (float) ADC_RANGE) ;
  //  printf("Vref = %1.3fV (%d)\n\r", adc_v, adc_results[2]) ;

    // Update set temperature value
    if((TIM1->CNT >> 2) - last_count) { 
      int32_t change = ((int32_t)(TIM1->CNT >> 2) - last_count) ;

      if(abs(change) < 10) user_set_temp += change * 1.0 ;
      else user_set_temp += change * 5.0 ;

      last_count = TIM1->CNT >> 2 ;

      if(user_set_temp > MAX_TEMP) user_set_temp = MAX_TEMP ;
      if(user_set_temp < 0) user_set_temp = 0 ;

      update_display = 1 ; 
    }

    if(ssr_on_counter > 0) {
      LCD_SetPosition(LINE_1, 15) ;
      LCD_Print("\x02") ;
      HAL_GPIO_WritePin(SSR_GPIO_Port, SSR_Pin, GPIO_PIN_SET) ;
    }
    else {
      LCD_SetPosition(LINE_1, 15) ;
      LCD_Print("\x01") ;
      HAL_GPIO_WritePin(SSR_GPIO_Port, SSR_Pin, GPIO_PIN_RESET) ;
    }

    // Update LCD 
    if(update_display) {
      LCD_SetPosition(LINE_2, 0) ;
      snprintf(temp_update, 17, "% 3.1f\xb2\x43  % 3.0f\xb2\x43 ", plate_temp, user_set_temp) ;
      LCD_Print(temp_update) ;

      printf("-----\n\r") ;
      printf("VTherm = %1.3fV\n\r", adc_v) ;
      printf("Rt = %3.1f  | Tp = %3.1f\n\r", therm_rt, plate_temp) ;
      printf("SSR on counter: %d\n\r", ssr_on_counter) ;

      update_display = 0 ;
    }
  //  CDC_Transmit_FS((uint8_t *) test, 6) ;
//    HAL_Delay(1000) ;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SSR_Pin|Fan_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DispD6_Pin|DispD7_Pin|DispE_Pin|DispRW_Pin
                          |DispRS_Pin|DispD0_Pin|DispD1_Pin|DispD2_Pin
                          |DispD3_Pin|DispD4_Pin|DispD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BP_LED_Pin */
  GPIO_InitStruct.Pin = BP_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BP_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SSR_Pin Fan_Pin */
  GPIO_InitStruct.Pin = SSR_Pin|Fan_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DispD6_Pin DispD7_Pin DispE_Pin DispRW_Pin
                           DispRS_Pin DispD0_Pin DispD1_Pin DispD2_Pin
                           DispD3_Pin DispD4_Pin DispD5_Pin */
  GPIO_InitStruct.Pin = DispD6_Pin|DispD7_Pin|DispE_Pin|DispRW_Pin
                          |DispRS_Pin|DispD0_Pin|DispD1_Pin|DispD2_Pin
                          |DispD3_Pin|DispD4_Pin|DispD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EncButton_Pin */
  GPIO_InitStruct.Pin = EncButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EncButton_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == EncButton_Pin) {
    plate_set_temp = user_set_temp ;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint16_t led_counter = 0 ;

  if(htim == &htim3) {
    // Toggle onboard LED
    if(++led_counter == 100) {
      HAL_GPIO_TogglePin(BP_LED_GPIO_Port, BP_LED_Pin) ;
      led_counter = 0 ;
    }

    // Do ADC capture
    if((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_BUSY) == 0)
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_results, 3) ;

    // Updates plate SSR on counter
    if(ssr_on_counter > 0) ssr_on_counter-- ;

    // Update LEDs state
    update_leds() ;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  if(hadc == &hadc1) {
    if(thrm_buf_idx < THRM_AVG)
      thrm_res_buffer[thrm_buf_idx++] = adc_results[ADC_THRM_CH] ;
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
