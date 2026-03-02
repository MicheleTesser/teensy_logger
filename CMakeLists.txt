cmake_minimum_required(VERSION 3.25)

project(teensy_logger C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(PROJECT_TARGET "${PROJECT_NAME}.elf")
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/MIMXRT1062_flash.ld")
set(ARM_CPU_FLAGS
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv5-d16
  -mfloat-abi=hard
)

set(PROJECT_SOURCES
  source/led_blinky.c
  source/semihost_hardfault.c
  startup/startup_mimxrt1062.c

  board/board.c
  board/clock_config.c
  board/dcd.c
  board/pin_mux.c

  drivers/fsl_clock.c
  drivers/fsl_common.c
  drivers/fsl_common_arm.c
  drivers/fsl_gpio.c
  drivers/fsl_lpuart.c

  device/system_MIMXRT1062.c

  xip/evkbmimxrt1060_flexspi_nor_config.c
  xip/fsl_flexspi_nor_boot.c
)

add_executable(${PROJECT_TARGET} ${PROJECT_SOURCES})

target_include_directories(${PROJECT_TARGET}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS
    ${CMAKE_CURRENT_SOURCE_DIR}/board
    ${CMAKE_CURRENT_SOURCE_DIR}/component/lists
    ${CMAKE_CURRENT_SOURCE_DIR}/component/serial_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/component/uart
    ${CMAKE_CURRENT_SOURCE_DIR}/device
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/xip
)

target_compile_definitions(${PROJECT_TARGET}
  PRIVATE
    CPU_MIMXRT1062DVL6A
    __USE_CMSIS=1
    __MCUXPRESSO=1
    XIP_EXTERNAL_FLASH=1
    XIP_BOOT_HEADER_ENABLE=1
    XIP_BOOT_HEADER_DCD_ENABLE=0
    SKIP_SYSCLK_INIT=1
    SDK_DEBUGCONSOLE=0
    SERIAL_PORT_TYPE_UART=1
    FSL_SDK_ENABLE_DRIVER_CACHE_CONTROL=1
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
)

target_compile_options(${PROJECT_TARGET}
  PRIVATE
    ${ARM_CPU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-cast-function-type
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2>
)

target_link_options(${PROJECT_TARGET}
  PRIVATE
    ${ARM_CPU_FLAGS}
    "-T${LINKER_SCRIPT}"
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-$<LOWER_CASE:$<CONFIG>>.map"
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -specs=nano.specs
    -specs=nosys.specs
)

set_target_properties(${PROJECT_TARGET}
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}
    SUFFIX ".elf"
)

find_program(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy REQUIRED)
set(HEX_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/output")

add_custom_command(
  TARGET ${PROJECT_TARGET}
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${HEX_OUTPUT_DIR}"
  COMMAND ${ARM_NONE_EABI_OBJCOPY} -O ihex
          $<TARGET_FILE:${PROJECT_TARGET}>
          "${HEX_OUTPUT_DIR}/$<LOWER_CASE:$<CONFIG>>.hex"
  COMMENT "Generating HEX $<LOWER_CASE:$<CONFIG>>.hex"
)
