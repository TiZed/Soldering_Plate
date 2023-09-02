/**
  ******************************************************************************
  * @file           : thermistor.c
  * @author         : TiZed
  * @brief          : Code for calculating thermistor resistance and 
  *                   temperature
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 TiZed.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#include <math.h>

#include "thermistor.h"

/**
  * @brief Calculate thermistor resistance, by Vrt, R1 and Vcc. Use
  *        this function when Vcc on divider is different supply from ADC Va.
  * @param vr_therm: Voltage over thermistor as measured by ADC, in mV.
  * @param r1: Value of resistor in series with thermistor, in ohms.
  * @param vcc_mv: Vcc voltage, feeding the resistors, in mV.
  * @retval float, resistance in ohms
  */
float rt_by_voltage(unsigned vr_therm, unsigned int r1, unsigned int vcc_mv) {
    return ((float)vr_therm * r1) / (float)(vcc_mv - vr_therm) ;
}

/**
  * @brief Calculate thermistor resistance, by Ratio. The ratio is
  *        ADC reading divided by Max. ADC value. Use this function when
  *        Divider Vcc and ADC Va are fed from the same supply. 
  * @param ratio: ADC reading divided by Max. ADC value.
  * @param r1: Value of resistor in series with thermistor, in ohms.
  * @retval float, resistance in ohms
  */
float rt_by_ratio(float ratio, unsigned int r1) {
    return ((float)r1 * ratio)/(1 - ratio) ;
}

/**
 * @brief Calculate temperature from thermistor resistance.
 * @param rt: Measured resistance of thermistor, in ohms.
 * @param beta: \beta value of thermistor from spec.
 * @param r25: Thermistor resistance @25C from spec, in ohms.
 * @retval float, temperature in C.
 * 
 */
float calculate_temperature(float rt, unsigned int beta, unsigned int r25) {
    float t = KMIN * (float)beta ;
    t /= KMIN * log(rt / (float)r25) + (float)beta ;
    t -= T25K ;

    return t ;
}