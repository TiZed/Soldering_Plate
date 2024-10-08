
/**
  ******************************************************************************
  * @file           : lcd16x2.c
  * @author         : TiZed
  * @brief          : Contains functions for driving a 16x2 text LCD.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
*/

#include <stdarg.h>

#include "stm32f1xx_hal.h"
#include "lcd16x2.h"

IF_Type mode_8b_4b ;

GPIO_TypeDef *LCDPort ;
static uint16_t DispRW, DispRS, DispE ;
static uint16_t DispD0, DispD1, DispD2, DispD3 ;
static uint16_t DispD4, DispD5, DispD6, DispD7 ;

/**
  * @brief Write to command or data to LCD
  * @param op: Select command (OP_CMD), or data (OP_DATA) operation.
  * @param data: Value to write.
  * @retval void
  */
void LCD_write(OP_Type op, uint8_t data) {
  uint32_t odr = DispRS * op ;

  HAL_GPIO_WritePin(LCDPort, DispRW, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(LCDPort, DispRS, op) ;

  odr |= DispD7 * CHECKBIT(data, 8) ;
  odr |= DispD6 * CHECKBIT(data, 7) ;
  odr |= DispD5 * CHECKBIT(data, 6) ;
  odr |= DispD4 * CHECKBIT(data, 5) ;

  if(mode_8b_4b == IF_8BIT) {
    odr |= DispD3 * CHECKBIT(data, 4) ;
    odr |= DispD2 * CHECKBIT(data, 3) ;
    odr |= DispD1 * CHECKBIT(data, 2) ;
    odr |= DispD0 * CHECKBIT(data, 1) ;
  }

  LCDPort->ODR = odr ;

  HAL_Delay(E_DELAY_MS) ;
  HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_SET) ;
  HAL_Delay(E_DELAY_MS) ;
  HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_RESET) ;
  HAL_Delay(E_DELAY_MS) ;

  if(mode_8b_4b == IF_4BIT) {

    odr = DispRS * op ;

    odr |= DispD7 * CHECKBIT(data, 4) ;
    odr |= DispD6 * CHECKBIT(data, 3) ;
    odr |= DispD5 * CHECKBIT(data, 2) ;
    odr |= DispD4 * CHECKBIT(data, 1) ;

    LCDPort->ODR = odr ;
    
  //  HAL_Delay(E_DELAY_MS) ;
    HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_SET) ;
    HAL_Delay(E_DELAY_MS) ;
    HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_RESET) ;
    HAL_Delay(E_DELAY_MS) ;
  }
}

/**
  * @brief Initialize LCD
  * @param lcdport: GPIO port connected to LCD.
  * @param if_type: Interface type, 4bit (IF_4BIT) or 8bit (IF_8BIT).
  * @param disp_rw: R~W pin.
  * @param disp_rs: R~S pin.
  * @param disp_e: Enable pin.
  * @param ...: Data pins, 4 or 8 depending on interface.
  * @retval void
  */
void LCD_Init(GPIO_TypeDef *lcdport, IF_Type if_type, 
              uint16_t disp_rw, uint16_t disp_rs, uint16_t disp_e, ...) 
{
  LCDPort = lcdport ;
  DispRW = disp_rw ;
  DispRS = disp_rs ;
  DispE = disp_e ;

  mode_8b_4b = if_type ;
  
  va_list port_args ;
  va_start(port_args, disp_e) ;

  if(if_type == IF_8BIT) {
    DispD0 = va_arg(port_args, int) ;
    DispD1 = va_arg(port_args, int) ;
    DispD2 = va_arg(port_args, int) ;
    DispD3 = va_arg(port_args, int) ;
  }

  DispD4 = va_arg(port_args, int) ;
  DispD5 = va_arg(port_args, int) ;
  DispD6 = va_arg(port_args, int) ;
  DispD7 = va_arg(port_args, int) ;

  va_end(port_args) ;

  HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(LCDPort, DispRW, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(LCDPort, DispRS, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(LCDPort, DispD7, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(LCDPort, DispD6, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(LCDPort, DispD5, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(LCDPort, DispD4, GPIO_PIN_RESET) ;

  if(if_type == IF_8BIT) {
    HAL_GPIO_WritePin(LCDPort, DispD3, GPIO_PIN_RESET) ;
    HAL_GPIO_WritePin(LCDPort, DispD2, GPIO_PIN_RESET) ;
    HAL_GPIO_WritePin(LCDPort, DispD1, GPIO_PIN_RESET) ;
    HAL_GPIO_WritePin(LCDPort, DispD0, GPIO_PIN_RESET) ;
  }

  HAL_Delay(150) ;
  if(if_type == IF_4BIT) {
    uint32_t odr = 0x0 ;

    odr |= DispD7 * CHECKBIT(0x02, 4) ;
    odr |= DispD6 * CHECKBIT(0x02, 3) ;
    odr |= DispD5 * CHECKBIT(0x02, 2) ;
    odr |= DispD4 * CHECKBIT(0x02, 1) ;

    LCDPort->ODR = odr ;
    
    HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_SET) ;
    HAL_Delay(E_DELAY_MS) ;
    HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_RESET) ;
    HAL_Delay(E_DELAY_MS) ;

    LCD_write(OP_CMD, FUNCTION_SET | DATA_LEN_4BIT | DISP_2LINES) ;
    HAL_Delay(5) ;
    LCD_write(OP_CMD, FUNCTION_SET | DATA_LEN_4BIT | DISP_2LINES) ;
  }
  else {
    LCD_write(OP_CMD, FUNCTION_SET | DATA_LEN_8BIT | DISP_2LINES) ;
    HAL_Delay(5) ;
    LCD_write(OP_CMD, FUNCTION_SET | DATA_LEN_8BIT | DISP_2LINES) ;
  }

  HAL_Delay(5) ;
  LCD_write(OP_CMD, DISPLAY_CONTROL | DISPLAY_ON) ;
  LCD_write(OP_CMD, CLEAR_DISPLAY) ;
  HAL_Delay(2) ;
  LCD_write(OP_CMD, ENTRY_MODE_SET | MOVE_CUR_RIGHT | DONT_SHIFT_D) ;
}

/**
  * @brief Print on LCD
  * @param str: String to display.
  * @retval void
  */
void LCD_Print(char * str) {
  uint32_t i = 0 ;

  while(str[i] != '\0') {
    LCD_write(OP_DATA, str[i++]) ;
  }
}

/**
  * @brief Set cursor position on LCD
  * @param line: First line (LINE_1), or second line (LINE_2).
  * @param col: Column position (0~15).
  * @retval void
  */
void LCD_SetPosition(Line_Num_t line, uint8_t col) {
  if(col > 0x0f) return ;
  LCD_write(OP_CMD, DDRAM_ADDRESS | line | col) ;
  HAL_Delay(2) ;
}

/**
  * @brief Set custom character
  * @param pattern: Pattern number to store (0-7)
  * @param char_data: Character bitmap pattern 8 x uint8
  * @retval void
  */
void LCD_SetCustomChar(uint8_t pattern, uint8_t * char_data) {
  if(pattern > 7) return ;
  LCD_write(OP_CMD, CGRAM_ADDRESS + (pattern << 3)) ;
  LCD_Wait() ;
  for(uint8_t i = 0 ; i < 8 ; i++) {
    LCD_write(OP_DATA, char_data[i]) ;
    LCD_Wait() ;
  } 
}

/**
  * @brief Clear LCD
  * @retval void
  */
void LCD_Clear() {
  LCD_write(OP_CMD, CLEAR_DISPLAY) ;
  HAL_Delay(2) ;
}


/**
  * @brief Wait on LCD busy flag
  * @retval void
  */
void LCD_Wait() {
  uint32_t data ;
  uint8_t busy_flag = 1 ;

  HAL_GPIO_WritePin(LCDPort, DispRW, GPIO_PIN_SET) ;
  HAL_GPIO_WritePin(LCDPort, DispRS, GPIO_PIN_RESET) ;

  while(busy_flag) {
    HAL_Delay(E_DELAY_MS) ;
    HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_SET) ;
    HAL_Delay(E_DELAY_MS) ;
    HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_RESET) ;
    HAL_Delay(E_DELAY_MS) ;

    data = LCDPort->IDR ;
    busy_flag = (data & DispD7) ? 1:0 ;

    if(mode_8b_4b == IF_4BIT) {
      HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_SET) ;
      HAL_Delay(E_DELAY_MS) ;
      HAL_GPIO_WritePin(LCDPort, DispE, GPIO_PIN_RESET) ;
      HAL_Delay(E_DELAY_MS) ;
    }
  }
}