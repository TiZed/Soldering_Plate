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

#define E_DELAY_MS   (1)

// KS0066U Commands
#define CLEAR_DISPLAY   0x001
#define RETURN_HOME     0x002
#define ENTRY_MODE_SET  0x004
#define DISPLAY_CONTROL 0x008
#define CUR_DIS_SHIFT   0x010
#define FUNCTION_SET    0x020
#define CGRAM_ADDRESS   0x040
#define DDRAM_ADDRESS   0x080
#define BF_AND_ADDRESS  0x100
#define WRITE_TO_RAM    0x200
#define READ_FROM_RAM   0x300

// Entry modes
#define MOVE_CUR_RIGHT  0b10
#define MOVE_CUR_LEFT   0b00

#define SHIFT_DISPLAY   0b01
#define DONT_SHIFT_D    0b00

// Display control modes
#define DISPLAY_ON      0b100
#define DISPLAY_OFF     0b000

#define CURSOR_ON       0b010
#define CURSOR_OFF      0b000

#define CURSOR_BLINK    0b001
#define CURSOR_SOLID    0b000

// Cursor or Display shifts
#define CUR_SHIFT_LEFT  0b0000
#define CUR_SHIFT_RIGHT 0b0100
#define DIS_SHIFT_LEFT  0b1000
#define DIS_SHIFT_RIGHT 0b1100

// Function Set modes
#define DATA_LEN_8BIT   0b10000
#define DATA_LEN_4BIT   0b00000

#define DISP_2LINES     0b01000
#define DISP_1LINE      0b00000

#define FONT_5X11       0b00100
#define FONT_5X8        0b00000

// Busy flag
#define BF_FLAG         0b10000000

#define CHECKBIT(val, bit) ((val >> (bit - 1)) & 0xfe)

typedef enum {IF_4BIT, IF_8BIT} IF_Type ;
typedef enum {OP_CMD = 0, OP_DATA} OP_Type ;
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