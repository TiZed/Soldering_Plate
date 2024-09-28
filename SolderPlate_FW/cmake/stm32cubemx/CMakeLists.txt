cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F103xB
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../USB_DEVICE/App
    ../../USB_DEVICE/Target
    ../../Core/Inc
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Inc
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32f1xx_it.c
    ../../Core/Src/stm32f1xx_hal_msp.c
    ../../USB_DEVICE/App/usb_device.c
    ../../USB_DEVICE/App/usbd_desc.c
    ../../USB_DEVICE/App/usbd_cdc_if.c
    ../../USB_DEVICE/Target/usbd_conf.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_iwdg.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    ../../Core/Src/system_stm32f1xx.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    /tools/STM32/STM32Cube_FW_F1_V1.8.6/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32f103xb.s
    ../../Core/Src/leds.c
    ../../Core/Src/lcd16x2.c
    ../../Core/Src/cli.c
    ../../Core/Src/cli_commands.c
    ../../Core/Src/pid.c
    ../../Core/Src/state.c
    ../../Core/Src/persistent_data.c
    ../../Core/Src/thermistor.c
    ../../Core/Src/usb_redirect.c
    ../../Core/Src/menu.c
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


