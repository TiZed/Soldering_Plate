/**
  ******************************************************************************
  * @file           : menu.h
  * @author         : TiZed
  * @brief          : Header for menu.c file.
  *                   Contains the common defines for user menu
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef __MENU_H
#define __MENU_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char * title ;

    const double min_value ;
    const double max_value ;

    double * value ;
} menu_item_t ;

extern menu_item_t user_menu[] ;
extern uint32_t menu_size ;

#ifdef __cplusplus
}
#endif

#endif /* __MENU_H */