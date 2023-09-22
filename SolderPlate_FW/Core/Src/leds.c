/**
  ******************************************************************************
  * @file           : leds.c
  * @author         : TiZed
  * @brief          : Code for controlling PWM LEDs 
  *                 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
*/

#include <stdint.h>
#include <stdlib.h>

#include "leds.h"

uint32_t system_rate = 0 ;      // Hz
uint32_t system_T = 0 ;         // msec.

leds_state_t * all_leds[MAX_LEDS] ;
uint32_t num_of_leds = 0 ;

/**
  * @brief Initialize the PWM LEDs library.
  * @param rate: System real-time update cycles per sec.
  * @retval void
  */
void init_leds(uint32_t rate) {
    system_rate = rate ;
    system_T = (uint32_t)(1000.0 / (float)system_rate) ;

    for(uint8_t i = 0 ; i < MAX_LEDS ; i++) all_leds[i] = NULL ;
}

/**
  * @brief Initialize a new PWM LED
  * @param pwm_bits: Number of PWM control bits.
  * @param pwm_register: Pointer to PWM control register.
  * @retval leds_state_t, PWM LED state structure.
  */
leds_state_t * new_led(uint8_t pwm_bits, volatile uint32_t * pwm_register) {
    if(num_of_leds == MAX_LEDS) return NULL ;

    leds_state_t * led = malloc(sizeof(leds_state_t)) ;
    if(led == NULL) return NULL ;

    led->pwm_range = (2 << (pwm_bits-1)) - 1 ;
    led->pwm_percent = led->pwm_range / 100 ;
    led->pwm_register = pwm_register ;
    led->pwm_value = *(led->pwm_register) = 0 ;

    all_leds[num_of_leds++] = led ;

    return led ;
}

/**
  * @brief Set PWM LED state
  * @param led: LED state structure, to select LED.
  * @param intensity: New LED intensity to set.
  * @param transition_time: Time, in msec., to transition to new state.
  * @retval void
  */
void set_led(leds_state_t * led, uint32_t intensity, uint32_t transition_time) {
    if(intensity > 100) return ;
    led->pwm_value = led->pwm_percent * intensity ;
    if(led->pwm_value > led->pwm_range) led->pwm_value = led->pwm_range ;

    int32_t transistion_steps = transition_time / system_T ;
    if(transistion_steps == 0) transistion_steps = 1 ;

    led->transition_step = (led->pwm_value - *(led->pwm_register)) ; 
    led->transition_step /= transistion_steps  ; 
}

/**
  * @brief Remove a PWM LED 
  * @param led: LED state structure, to select LED.
  * @retval void
  */
void remove_led(leds_state_t * led) {
    uint32_t i = 0, j = 0 ;
    num_of_leds-- ;

    while(i < num_of_leds) {
        if(led == all_leds[i]) i++ ;
        if(i >= MAX_LEDS) break ;
        all_leds[j++] = all_leds[i++] ;
    }

    free(led) ;
}

/**
  * @brief Update all PWM LEDs state, should be run in a periodic loop.
  * @retval void
  */
void update_leds() {
    for(uint8_t i = 0 ; i < num_of_leds ; i++) {
        leds_state_t * led = all_leds[i] ;

        if((*(led->pwm_register) - led->pwm_value) < led->transition_step) 
            *(led->pwm_register) = led->pwm_value ;
        else *(led->pwm_register) += led->transition_step ;
    }
}

