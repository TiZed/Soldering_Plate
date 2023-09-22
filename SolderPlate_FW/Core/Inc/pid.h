/**
  ******************************************************************************
  * @file           : pid.h
  * @author         : TiZed
  * @brief          : PID library header
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
*/

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double p_gain ;
    double i_gain ;
    double d_gain ;

    double i_min ;
    double i_max ;

    double i_state ;
    double prev_state ;
} pid_config_t ;

pid_config_t * init_pid(double p_gain, double i_gain, double d_gain, double i_min, double i_max) ;
double update_pid(pid_config_t * pid_config, double error, double state) ;
void clear_pid(pid_config_t * pid_config) ;


#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
