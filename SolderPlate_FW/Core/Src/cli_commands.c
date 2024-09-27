/**
  ******************************************************************************
  * @file           : cli_commands.c
  * @author         : TiZed
  * @brief          : CLI commands
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

#include "cli_commands.h"
#include "state.h"
#include "persistent_data.h"

s_cmd commands_list[] = {
  {"reset",   cmd_reset,  0, 1},
  {"get_pid_param", get_pid_param, 0, 0},
  {"\0", NULL, 0, 0}
} ;

unsigned int cmd_reset(int argc, char **argv) {
  if (argc > 0) {
    if (strcmp(argv[1], "bootloader")) {
      SystemState = SYS_BOOTLOADER_RESET ;
    }
  }
  else SystemState = SYS_RESET ;
  return 1 ;
}

unsigned int get_pid_param(int argc, char **argv) {
  printf("\r\n") ;
  printf("P Gain = %f\r\n", persistent_data->p_gain) ;
  printf("D Gain = %f\r\n", persistent_data->d_gain) ;
  printf("I Gain = %f\r\n", persistent_data->i_gain) ;
  printf("I Min. = %f\r\n", persistent_data->i_min) ;
  printf("I Max. = %f\r\n", persistent_data->i_max) ;

  return 1 ;
}