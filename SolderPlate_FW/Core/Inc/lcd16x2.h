/**
  ******************************************************************************
  * @file           : lcd16x2.h
  * @brief          : Header for lcd16x2.c file.
  *                   Contains the common defines for driving a 16x2 text LCD.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef __LCD16X2_H
#define __LCD16X2_H

#ifdef __cplusplus
extern "C" {
#endif

#define E_DELAY_MS   (2)

// KS0066U Commands
#define CLEAR_DISPLAY   0x01
#define RETURN_HOME     0x02
#define ENTRY_MODE_SET  0x04
#define DISPLAY_CONTROL 0x08
#define CUR_DIS_SHIFT   0x10
#define FUNCTION_SET    0x20
#define CGRAM_ADDRESS   0x40
#define DDRAM_ADDRESS   0x80

// Entry modes
#define MOVE_CUR_RIGHT  0x02
#define MOVE_CUR_LEFT   0x00

#define SHIFT_DISPLAY   0x01
#define DONT_SHIFT_D    0x00

// Display control modes
#define DISPLAY_ON      0x04
#define DISPLAY_OFF     0x00

#define CURSOR_ON       0x02
#define CURSOR_OFF      0x00

#define CURSOR_BLINK    0x01
#define CURSOR_SOLID    0x00

// Cursor or Display shifts
#define CUR_SHIFT_LEFT  0x00
#define CUR_SHIFT_RIGHT 0x04
#define DIS_SHIFT_LEFT  0x08
#define DIS_SHIFT_RIGHT 0x0C

// Function Set modes
#define DATA_LEN_8BIT   0x10
#define DATA_LEN_4BIT   0x00

#define DISP_2LINES     0x08
#define DISP_1LINE      0x00

#define FONT_5X11       0x04
#define FONT_5X8        0x00

// Busy flag
#define BF_FLAG         0x80

#define CHECKBIT(val, bit) ((val >> (bit - 1)) & 0x01)

typedef enum {IF_4BIT = 0, IF_8BIT} IF_Type ;
typedef enum {OP_CMD = 0, OP_DATA = 1} OP_Type ;
typedef enum {LINE_1 = 0x00, LINE_2 = 0x40} Line_Num_t ;

// Functions
void LCD_write(OP_Type op, uint8_t data) ;
void LCD_Init(GPIO_TypeDef *lcdport, IF_Type if_type, 
              uint16_t disp_rw, uint16_t disp_rs, uint16_t disp_e, ...) ;
void LCD_Print(char * str) ;
void LCD_SetPosition(Line_Num_t line, uint8_t col) ;
void LCD_Clear() ;

#ifdef __cplusplus
}
#endif

#endif /* __LCD16X8_H */