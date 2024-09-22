

typedef struct {
    double p_gain ;
    double i_gain ;
    double d_gain ;
    double i_min ;
    double i_max ;
} persistent_vars_t ;

extern persistent_vars_t persistent_data ;

uint32_t persistent_save() ;

