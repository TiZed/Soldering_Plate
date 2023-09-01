/**
  ******************************************************************************
  * @file           : thermistor.h
  * @author         : TiZed
  * @brief          : Header for thermistor.c file.
  *                   Contains the common defines thermistor calculations.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef THERMISTOR_H
#define THERMISTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define KMIN  (298.15)
#define T25K  (KMIN - 25)

float rt_by_voltage(unsigned vr_therm, unsigned int r1, unsigned int vcc_mv) ;
float rt_by_ratio(float ratio, unsigned int r1) ;
float calculate_temperature(float rt, unsigned int beta, unsigned int r25) ;

#ifdef __cplusplus
}
#endif

#endif /* THERMISTOR_H */