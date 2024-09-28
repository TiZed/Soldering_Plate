/**
  ******************************************************************************
  * @file           : menu.c
  * @author         : TiZed
  * @brief          : User menu to control plate parameters
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
#include <stdint.h>

#include "menu.h"
#include "persistent_data.h"

menu_item_t user_menu[] = {
    {"Menu", 0, 0, NULL},
    {"P Gain", 0.0, 1000.0, &(persistent_store.values.p_gain)},
    {"D Gain", 0.0, 1000.0, &(persistent_store.values.d_gain)},
    {"I Gain", 0.0, 1000.0, &(persistent_store.values.i_gain)},
    {"I Min.", 0.0, 1000.0, &(persistent_store.values.i_min)},
    {"I Max.", 0.0, 1000.0, &(persistent_store.values.i_max)},
} ;

uint32_t menu_size = sizeof(user_menu) / sizeof(menu_item_t) ;