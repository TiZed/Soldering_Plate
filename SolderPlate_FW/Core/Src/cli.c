/**
  ******************************************************************************
  * @file           : cli.c
  * @author         : TiZed
  * @brief          : Code to add a command line interface to a project
  *                 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
*/

#include <string.h>
#include <stdio.h>

#include "stm32f1xx_hal.h"

#include "cli.h" 
#include "cli_commands.h"
#include "state.h"

char cmd_buffer[CMD_SIZE] ;
// char history[HISTORY_LEN][CMD_SIZE] ;

static unsigned int cmd_i = 0 ; 
// static int hist_pos = -1 ;

static char prompt[] = "\x1B[0m\x1B[K\x1B[1m#>\x1B[22m " ; 

void cli_init() {
  printf("%s", prompt) ;
  fflush(stdout) ;
}

void cli_input(const char * u_in) {
  static int in_state = REG_CHAR ;
  unsigned int i = 0 ;
//  for(uint32_t j = 0 ; (j < CMD_SIZE) && u_in[j] != '\0' ; j++) printf("%x,", u_in[j]) ;
  while(u_in[i]) {
    if (in_state == ESC_SEQ) {
      if (u_in[i] == '[') ;
      else if (u_in[i] == UP_KEY) {
//        printf("up\r\n") ;
        in_state = REG_CHAR ;
      }
      else if (u_in[i] == DOWN_KEY) {
//        printf("down\r\n") ; 
        in_state = REG_CHAR ;       
      }
      else if (u_in[i] == LEFT_KEY) {
//        printf("left\r\n") ;
        in_state = REG_CHAR ;
        
      }
      else if (u_in[i] == RIGHT_KEY) {
//        printf("right\r\n") ;
        in_state = REG_CHAR ;
      }
      else in_state = REG_CHAR ;
    }

    else if (u_in[i] == ENTER_KEY) SystemState = SYS_CMD_EXEC ;

    else if (u_in[i] == BACKSPACE_KEY) {
      if (cmd_i > 0) {
        printf("\x1B[1D\x1B[K") ;
        fflush(stdout) ;
        cmd_i-- ;
      }
    }

    else if (u_in[i] == KEY_CHAR) in_state = ESC_SEQ ;

    else {
      cmd_buffer[cmd_i++] = u_in[i] ;
      putchar(u_in[i]) ;
      fflush(stdout) ;
    }

    i++ ;
  }
}

void cli_exec()
{
  SystemState = SYS_RUNNING ;
  printf("\r\n") ;

  if (cmd_i == 0) {
    printf("%s", prompt) ;
    fflush(stdout) ;
    return ;
  }

  cmd_buffer[cmd_i] = '\0' ;
  cmd_i = 0 ;

  char *argv[CMDS_MAX_ARGS] ;
	int argc = 0 ;
  char * loc ;
  int pos = 0 ;

  while((loc = strchr(&cmd_buffer[pos], ' ')) != NULL) {
    argv[argc++] = &cmd_buffer[pos] ; 
    *loc = '\0' ; 
    pos += (loc - &cmd_buffer[pos]) ;
  }

  argv[argc++] = &cmd_buffer[pos] ;   
  
  int i = 0 ;
  while(commands_list[i].cmd_str[0]) {
    if(strncmp(argv[0], commands_list[i].cmd_str, CMD_SIZE) == 0) {
      commands_list[i].cmd_ptr(argc, argv) ;
      printf("%s", prompt) ;
      fflush(stdout) ;
      return ;
    }
    i++ ;
  }
  printf("Unknown command!\r\n") ;
  printf("%s", prompt) ;
  fflush(stdout) ;
}

