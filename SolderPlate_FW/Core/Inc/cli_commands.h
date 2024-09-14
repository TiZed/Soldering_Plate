/**
  ******************************************************************************
  * @file           : cli_command.h
  * @author         : TiZed
  * @brief          : Header for cli_commands.c file.
  *                   Contains the common defines CLI commands
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef __CLI_COMMANDS_H
#define __CLI_COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    const char * cmd_str ;
    unsigned int (* cmd_ptr)(int argc, char *argv[]) ;

    const int max_args ;
    const int min_args ;    
} s_cmd ;

extern s_cmd commands_list[] ;

unsigned int cmd_reset(int argc, char **argv) ;

#ifdef __cplusplus
}
#endif

#endif /* __CLI_COMMANDS_H */