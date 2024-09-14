/**
  ******************************************************************************
  * @file           : reset.h
  * @author         : TiZed
  * @brief          : Header for reset.c file.
  *                   System reset calls.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef __RESET_H
#define __RESET_H

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t _estack;

static inline void reset_to_bootloader() {
    uint64_t * ptr = (uint64_t*)&_estack;
	*ptr = 0xDEADBEEFCC00FFEEULL;

    HAL_NVIC_SystemReset() ;
}

static inline void reset() {
    HAL_NVIC_SystemReset() ;
}

#ifdef __cplusplus
}
#endif

#endif /* __RESET_H */