# ARBATOS Zephyr 4.4 application entry.
cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr 4.4 REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arbatos)

target_sources(app PRIVATE
    src/main.c
    src/ArbatosTarget.c
)

target_sources_ifdef(CONFIG_ARBATOS_LEGACY_SOURCES app PRIVATE
    src/ArbatosRuntime.c
    compat/freertos/ArbFreeRtosCompat.c
    port/common/BspCoreZephyr.c
    port/algorithm/AhrsZephyr.c
    port/algorithm/ArmMathZephyr.c
    port/can/BspCanZephyr.c
    port/uart/BspZephyrUart.c
    port/sensors/BspImuPwm.c
    port/sensors/ImuFusionTask.c
    port/storage/BspZephyrDiskio.c
    port/storage/BspZephyrSdSpiPort.c
    port/platform/ArbatosPlatform.c
    port/platform/BspAdc.c
    port/platform/BspBuzzer.c
    port/platform/BspFlash.c
    port/platform/BspKey.c
    port/platform/BspLed.c
    port/platform/BspResetEvidence.c
    port/platform/BspServoPwm.c
    port/platform/BspShootTrig.c
    port/platform/BspUsb.c
    port/usb/BspZephyrUsbCdc.c
)

if(CONFIG_ARBATOS_LEGACY_SOURCES)
    zephyr_linker_sources(SECTIONS linker/arbatos-legacy-ram.ld)

    if((CONFIG_ARBATOS_TARGET_HERO_C OR CONFIG_ARBATOS_TARGET_MINIWHEELEG_C) AND
       NOT CONFIG_BOARD_DJI_C_F407)
        message(FATAL_ERROR "HERO-C and MINIWHEELEG-C require board dji_c_f407")
    elseif((CONFIG_ARBATOS_TARGET_INFANTRY_A OR CONFIG_ARBATOS_TARGET_CARRIER_A) AND
           NOT CONFIG_BOARD_DJI_A_F427)
        message(FATAL_ERROR "INFANTRY-A and CARRIER-A require board dji_a_f427")
    elseif((CONFIG_ARBATOS_TARGET_HERO_M OR
            CONFIG_ARBATOS_TARGET_SENTINEL_M OR
            CONFIG_ARBATOS_TARGET_MINIWHEELEG_M) AND
           NOT CONFIG_BOARD_DM_MC02_H7)
        message(FATAL_ERROR "HERO-M, SENTINEL-M and MINIWHEELEG-M require board dm_mc02_h7")
    endif()

    if(CONFIG_ARBATOS_TARGET_HERO_C OR CONFIG_ARBATOS_TARGET_MINIWHEELEG_C)
        target_sources(app PRIVATE
            port/sensors/BspBmi088Port.c
            port/sensors/BspIst8310Port.c
        )
    elseif(CONFIG_ARBATOS_TARGET_INFANTRY_A OR CONFIG_ARBATOS_TARGET_CARRIER_A)
        target_sources(app PRIVATE port/sensors/Mpu6500.c)
    elseif(CONFIG_ARBATOS_TARGET_HERO_M OR
           CONFIG_ARBATOS_TARGET_SENTINEL_M OR
           CONFIG_ARBATOS_TARGET_MINIWHEELEG_M)
        target_sources(app PRIVATE port/sensors/BspBmi088Port.c)
    endif()

    if(CONFIG_ARBATOS_TARGET_SENTINEL_M)
        target_sources(app PRIVATE port/subboard/SubBoardBringupZephyr.c)
    endif()
endif()

target_include_directories(app BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/compat/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/compat/freertos
    ${CMAKE_CURRENT_SOURCE_DIR}/compat/stm32
    ${CMAKE_CURRENT_SOURCE_DIR}/port/algorithm/include
    ${CMAKE_CURRENT_SOURCE_DIR}/port/can
    ${CMAKE_CURRENT_SOURCE_DIR}/port/uart
    ${CMAKE_CURRENT_SOURCE_DIR}/port/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/port/storage
    ${CMAKE_CURRENT_SOURCE_DIR}/port/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/port/subboard
    ${CMAKE_CURRENT_SOURCE_DIR}/port/usb
)

# The existing Robotconfig and shared trees were written for STM32 HAL and
# FreeRTOS.  Keep their exact source locations in the build graph, but make
# reuse opt-in until the Zephyr port layer supplies every required interface.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ArbatosLegacy.cmake)
arbatos_add_legacy_sources(app)
