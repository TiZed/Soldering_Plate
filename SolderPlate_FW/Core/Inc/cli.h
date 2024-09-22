/**
  ******************************************************************************
  * @file           : cli.h
  * @author         : TiZed
  * @brief          : Header for cli.c file.
  *                   Contains the common defines for a command line I/F.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef __CLI_H
#define __CLI_H

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_SIZE      64
#define CMDS_MAX_ARGS 10
#define HISTORY_LEN   20

#define KEY_CHAR    '\e'
#define UP_KEY      'A'
#define DOWN_KEY    'B'
#define RIGHT_KEY   'C'
#define LEFT_KEY    'D'

#define BACKSPACE_KEY   '\b'
#define ENTER_KEY       '\r'

#define CLI_CLEAR_FORMAT    "\x1b[0m"

enum cli_state {REG_CHAR = 0, ESC_SEQ} ;

void cli_init() ;
void cli_input(const char *) ;
void cli_exec() ;

#ifdef __cplusplus
}
#endif

#endif /* __CLI_H */