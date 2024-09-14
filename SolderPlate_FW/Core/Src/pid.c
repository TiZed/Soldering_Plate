/**
  ******************************************************************************
  * @file           : pid.c
  * @author         : TiZed
  * @brief          : PID library code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
*/

#include <stdlib.h>

#include "pid.h"

/**
  * @brief Initialize PID controller
  * @param p_gain: Proportional gain value.
  * @param i_gain: Integral gain value.
  * @param d_gain: Differential gain value.
  * @param i_min: Integral min. value limit.
  * @param i_max: Integral max. value limit
  * @retval pid_config_t, PID control configuration and state
  */
pid_config_t * init_pid(double p_gain, double i_gain, double d_gain, double i_min, double i_max) {
    pid_config_t * new_pid = malloc(sizeof(pid_config_t)) ;
    if(new_pid == NULL) return NULL ;

    new_pid->p_gain = p_gain ;
    new_pid->i_gain = i_gain ;
    new_pid->d_gain = d_gain ;

    new_pid->i_min = i_min ;
    new_pid->i_max = i_max ;

    new_pid->i_state = 0 ;
    new_pid->prev_state = 0 ;

    return new_pid ;
}

/**
  * @brief Update PID controller state and control output
  * @param pid_config_t: PID configuration.
  * @param error: Process error value.
  * @param state: Current state value.
  * @retval double, control output value.
  */
double update_pid(pid_config_t * pid_config, double error, double state) {
    double p_value = 0, i_value = 0, d_value = 0 ;

    p_value = pid_config->p_gain * error ;

    if(pid_config->i_gain != 0) {
        pid_config->i_state += error ;
        if(pid_config->i_state > pid_config->i_max) pid_config->i_state = pid_config->i_max ;
        else if(pid_config->i_state < pid_config->i_min) pid_config->i_state = pid_config->i_min ;

        i_value = pid_config->i_gain * pid_config->i_state ;
    }

    if(pid_config->d_gain != 0) {
        d_value = pid_config->d_gain * (state - pid_config->prev_state) ;
        pid_config->prev_state = state ;
    }

    return p_value + i_value - d_value ;
}

/**
  * @brief Clear a PID controller
  * @param pid_config_t: PID configuration.
  * @retval void
  */
void clear_pid(pid_config_t * pid_config) {
    free(pid_config) ;
}