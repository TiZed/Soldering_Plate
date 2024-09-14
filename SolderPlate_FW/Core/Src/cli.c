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

#include "cli.h" 
#include "cli_commands.h"

static char cmd_buffer[CMD_SIZE] ;
static char history[HISTORY_LEN][CMD_SIZE] ;

static unsigned int cmd_i = 0 ; 
static int hist_pos = -1 ;

static char prompt[] = "\n\e[0m\r\e[K\e[1m# >\e[22m" ; 

void cli_init() {
  printf(prompt) ;
}

void cli_input(const char * u_in) {
  unsigned int i = 0 ;
  while(u_in[i]) {
    switch (u_in[i++])
    {
      case ENTER_KEY:
        cmd_buffer[cmd_i] = '\0' ;
        cmd_i = 0 ;
        printf("\n") ;
//        cli_exec(cmd_buffer) ;
        break ;
      case BACKSPACE_KEY:
        if (cmd_i > 0) {
          printf("\e[1D\e[K") ;
          cmd_i-- ;
        }
        break ;
      case KEY_CHAR:
        if (u_in[i++] != '[') break ;
        switch (u_in[i++])
        {
          case UP_KEY:
            break ;
          case DOWN_KEY:
            break ;
          case LEFT_KEY:
            break ;
          case RIGHT_KEY:
            break ;
        }
        break ;
     
      default:
        cmd_buffer[cmd_i++] = u_in[i] ;
        printf("%c", u_in[i]) ;
    }
  }
}

void cli_exec(char * cmd)
{
  int i = 0 ;

  char *argv[CMDS_MAX_ARGS] ;
	int argc = 0 ;
  char * loc ;
  int pos = 0 ;

  while(loc = strchr(&cmd[pos], ' ')) {
    argv[argc++] = &cmd[pos] ; 
    *loc = '\0' ; 
    pos += (loc - &cmd[pos]) ;
  }

  while(commands_list[i].cmd_str) {
    if(strncmp(argv[0], commands_list[i].cmd_str, CMD_SIZE) == 0) {
      commands_list[i].cmd_ptr(argc, argv) ;
    }
    else {
      printf("Unknown command!") ;
    }
  }

  printf(prompt) ;
}

