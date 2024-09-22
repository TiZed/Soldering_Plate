/**
  ******************************************************************************
  * @file           : state.h
  * @author         : TiZed
  * @brief          : System state variables.
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

#ifndef __STATE_H
#define __STATE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SYS_POWER_UP = 0,
  SYS_RUNNING,
  SYS_CMD_EXEC,
  SYS_RESET,
  SYS_BOOTLOADER_RESET
} system_state_t ;

extern system_state_t SystemState ;

#ifdef __cplusplus
}
#endif

#endif /* __STATE_H */