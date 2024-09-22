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

s_cmd commands_list[] = {
    {"reset",   cmd_reset,  0, 0},
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
