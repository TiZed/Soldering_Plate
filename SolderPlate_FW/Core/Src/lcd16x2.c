
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

void LCD_Print(char * str) {
  uint32_t i = 0 ;

  while(str[i] != '\0') {
    LCD_write(OP_DATA, str[i++]) ;
  }
}

void LCD_SetPosition(Line_Num_t line, uint8_t col) {
  if(col > 0x0f) return ;
  LCD_write(OP_CMD, DDRAM_ADDRESS | line | col) ;
  HAL_Delay(2) ;
}

void LCD_Clear() {
  LCD_write(OP_CMD, CLEAR_DISPLAY) ;
  HAL_Delay(2) ;
}