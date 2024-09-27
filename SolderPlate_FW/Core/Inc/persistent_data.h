

#ifndef __PERSISTENT_DATA_H
#define __PERSISTENT_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DEFAULT_P_GAIN   15.0 
#define DEFAULT_D_GAIN    0.0 
#define DEFAULT_I_GAIN    0.0 
#define DEFAULT_I_MIN     0.0 
#define DEFAULT_I_MAX   200.0

#define HEADER_VAL  0xCAFEC0DE08691234UL

typedef struct {
    uint64_t header_dword ;
    double p_gain ;
    double i_gain ;
    double d_gain ;
    double i_min ;
    double i_max ;
} persistent_vars_t ;

typedef union {
    persistent_vars_t values ;
    uint32_t pers_bits[sizeof(persistent_vars_t)/sizeof(uint32_t)] ;
} persistent_store_t ;

extern persistent_vars_t * persistent_data ;

uint32_t persistent_save() ;
uint32_t persistent_load() ;

#ifdef __cplusplus
}
#endif

#endif /* __PERSISTENT_DATA_H */