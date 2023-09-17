/**
  ******************************************************************************
  * @file           : leds.h
  * @author         : TiZed
  * @brief          : Header for leds.c file.
  *                   Contains the common defines leds PWM control.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef __LEDS_H
#define __LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_LEDS  (10)

typedef struct {
    uint32_t pwm_range ;
    uint32_t pwm_percent ;
    uint32_t pwm_value ;

    int32_t transition_step ;

    volatile uint32_t * pwm_register ;
} leds_state_t ;

void init_leds(uint32_t rate) ;
leds_state_t * new_led(uint8_t pwm_bits, volatile uint32_t * pwm_register) ;
void set_led(leds_state_t * led, uint32_t intensity, uint32_t transition_time) ;
void remove_led(leds_state_t * led) ;
void update_leds() ;

#ifdef __cplusplus
}
#endif

#endif /* __LEDS_H */