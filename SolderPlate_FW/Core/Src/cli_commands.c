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
#include "reset.h"

s_cmd commands_list[] = {
    {"reset",   cmd_reset,  0, 0},
    {0, 0, 0, 0}
} ;

unsigned int cmd_reset(int argc, char **argv) {
    if (argc > 0)
      if (strcmp(argv[1], "bootloader")) {
        reset_to_bootloader() ;
      }
    else reset() ;
    return 1 ;
}
