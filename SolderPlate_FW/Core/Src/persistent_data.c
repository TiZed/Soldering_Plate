

#include "stm32f1xx_hal.h"

#include "persistent_data.h"

__attribute__((__section__(".pers_vars"))) const persistent_vars_t pers_in_flash ;
persistent_vars_t persistent_data ;

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
	HAL_Delay(50);

    uint32_t words = sizeof(persistent_data) ;

    for(uint32_t i = 0 ; i < words / 8 ; i++) {
        stat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                 (uint32_t) &pers_in_flash, 
                                 (uint64_t) *((uint64_t *) (&persistent_data + i * 8)) );
        HAL_Delay(50);
        if (stat != HAL_OK) return HAL_ERROR ;
    }
	HAL_FLASH_Lock();

    return HAL_OK ;
}