/**
  ******************************************************************************
  * @file           : cli.h
  * @author         : TiZed
  * @brief          : Header for cli.c file.
  *                   Contains the common defines for a command line I/F.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
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

#define UP_KEY      "\x1b[A"
#define DOWN_KEY    "\x1b[B"
#define RIGHT_KEY   "\x1b[C"
#define LEFT_KEY    "\x1b[D"

#define BACKSPACE_KEY   '\b'
#define ENTER_KEY       '\r'

#define CLI_CLEAR_FORMAT    "\x1b[0m"

void cli_init() ;
void cli_exec() ;

#ifdef __cplusplus
}
#endif

#endif /* __CLI_H */