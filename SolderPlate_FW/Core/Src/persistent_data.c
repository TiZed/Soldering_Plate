

#include <string.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "persistent_data.h"

__attribute__((__section__(".pers_vars"))) const persistent_store_t pers_in_flash ;

persistent_store_t persistent_store ;
persistent_vars_t * persistent_data = &persistent_store.values ; 

uint32_t persistent_save() {
    HAL_StatusTypeDef stat ;

    HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = (uint32_t) &pers_in_flash ;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError;

    // Have to erase before write
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) return HAL_ERROR ;
    FLASH_WaitForLastOperation(200) ;
//	HAL_Delay(50);

    uint32_t words = sizeof(persistent_store_t)/sizeof(uint32_t) ;

    for(uint32_t i = 0 ; i < words ; i++) {
        stat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
                                 (uint32_t) &pers_in_flash.pers_bits[i], 
                                 (uint64_t) persistent_store.pers_bits[i] );
//       FLASH_WaitForLastOperation(200) ;
//        HAL_Delay(50);
        if (stat != HAL_OK) return HAL_ERROR ;
    } 
	HAL_FLASH_Lock();

    return HAL_OK ;
}

uint32_t persistent_load() {
    memcpy(persistent_store.pers_bits, pers_in_flash.pers_bits, sizeof(persistent_vars_t)) ;

    // Check that values are legit, if not set default values
    if (persistent_data->header_dword != HEADER_VAL) {
        persistent_data->header_dword = HEADER_VAL ;
        persistent_data->p_gain = DEFAULT_P_GAIN ;
        persistent_data->d_gain = DEFAULT_D_GAIN;
        persistent_data->i_gain = DEFAULT_I_GAIN ;
        persistent_data->i_min = DEFAULT_I_MIN ;
        persistent_data->i_max = DEFAULT_I_MAX ;

        // save so there are proper values the next time
        persistent_save() ;
    }

    return HAL_OK ;
}