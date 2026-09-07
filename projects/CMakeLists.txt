# ARBATOS Zephyr 4.4 application entry.
cmake_minimum_required(VERSION 3.20.0)

get_filename_component(ARBATOS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
list(APPEND BOARD_ROOT ${ARBATOS_ROOT})
list(APPEND DTS_ROOT ${ARBATOS_ROOT}/boards)
find_package(Zephyr 4.4 REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arbatos)


include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ArbatosBuildInfo.cmake)
arbatos_add_build_info(app)

target_sources(app PRIVATE
    src/main.c
    src/ArbatosTarget.c
)
target_sources_ifdef(CONFIG_ARBATOS_LOG_RAM app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/platform/ArbLogRam.c)

target_sources_ifdef(CONFIG_ARBATOS_LEGACY_SOURCES app PRIVATE
    src/ArbatosRuntime.c
    ${ARBATOS_ROOT}/shared/zephyr/compat/freertos/ArbFreeRtosCompat.c
    ${ARBATOS_ROOT}/shared/zephyr/port/common/BspCoreZephyr.c
    ${ARBATOS_ROOT}/shared/zephyr/port/algorithm/AhrsZephyr.c
    ${ARBATOS_ROOT}/shared/zephyr/port/algorithm/ArmMathZephyr.c
    ${ARBATOS_ROOT}/shared/zephyr/port/can/BspCanZephyr.c
    ${ARBATOS_ROOT}/shared/zephyr/port/can/BspCanFaultPort.c
    ${ARBATOS_ROOT}/shared/zephyr/port/uart/BspZephyrUart.c
    ${ARBATOS_ROOT}/shared/zephyr/port/uart/BspRs485FaultPort.c
    ${ARBATOS_ROOT}/shared/zephyr/port/sensors/BspImuPwm.c
    ${ARBATOS_ROOT}/shared/zephyr/port/sensors/ImuFusionTask.c
    ${ARBATOS_ROOT}/shared/zephyr/port/storage/BspZephyrDiskio.c
    ${ARBATOS_ROOT}/shared/zephyr/port/storage/BspZephyrSdSpiPort.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/ArbatosPlatform.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspFlash.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspKey.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspLed.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspResetEvidence.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/RobotFaultZephyr.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspServoPwm.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspShootTrig.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspUsb.c
    ${ARBATOS_ROOT}/shared/zephyr/port/usb/BspZephyrUsbCdc.c
)

if(CONFIG_ARBATOS_LEGACY_SOURCES)
    # 两个异常单元仍有不受 FAULT_DUMP 控制的打印；仅它们禁用日志，
    # 避免可恢复判断完成前或未处理 IRQ 路径重入日志锁，不改 SDK 源码。
    set_property(SOURCE "${ZEPHYR_BASE}/arch/arm/core/cortex_m/fault.c"
        TARGET_DIRECTORY arch__arm__core__cortex_m APPEND PROPERTY COMPILE_OPTIONS
        "-include" "${ARBATOS_ROOT}/shared/zephyr/include/ArbFaultLogConfig.h")
    set_property(SOURCE "${ZEPHYR_BASE}/arch/arm/core/fatal.c"
        TARGET_DIRECTORY arch__arm__core APPEND PROPERTY COMPILE_OPTIONS
        "-include" "${ARBATOS_ROOT}/shared/zephyr/include/ArbFaultLogConfig.h")
    # 保存 Zephyr 清除故障标志前的快照；可恢复异常仍由内核原流程处理。
    zephyr_link_libraries(-Wl,--wrap=z_arm_fault -Wl,--wrap=z_fatal_error)
    zephyr_linker_sources(SECTIONS ${ARBATOS_ROOT}/shared/zephyr/linker/arbatos-legacy-ram.ld)

    if(CONFIG_BOARD_DM_MC02_H7)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspBuzzerM.c ${ARBATOS_ROOT}/shared/zephyr/port/platform/MBoardIo.c
            ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspAdcM.c ${ARBATOS_ROOT}/shared/zephyr/port/sensors/ImuCalStore.c)
    else()
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspBuzzer.c ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspAdc.c)
    endif()
    target_sources_ifdef(CONFIG_ARBATOS_SUBBOARD_MUSIC app PRIVATE
        ${ARBATOS_ROOT}/shared/zephyr/port/subboard/SubBoardMusic.c)
    target_sources_ifdef(CONFIG_ARBATOS_PREFLIGHT_ONLY app PRIVATE
        ${ARBATOS_ROOT}/shared/zephyr/port/subboard/MPreflight.c)
    target_sources_ifdef(CONFIG_ARBATOS_RECEIVE_ONLY app PRIVATE
        ${ARBATOS_ROOT}/shared/zephyr/port/subboard/MReceiveCheck.c)

    if((CONFIG_ARBATOS_TARGET_HERO_M OR CONFIG_ARBATOS_TARGET_SENTINEL_M OR
        CONFIG_ARBATOS_TARGET_MINIWHEELEG_M) AND NOT CONFIG_BOARD_DM_MC02_H7)
        message(FATAL_ERROR "HERO-M, SENTINEL-M and MINIWHEELEG-M require board dm_mc02_h7")
    endif()

    if(CONFIG_BOARD_DJI_A_F427)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors/Mpu6500.c)
    elseif(CONFIG_BOARD_DJI_C_F407)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors/BspBmi088Port.c ${ARBATOS_ROOT}/shared/zephyr/port/sensors/BspIst8310Port.c)
    elseif(CONFIG_BOARD_DM_MC02_H7)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors/BspBmi088Port.c)
    else()
        message(FATAL_ERROR "Board sensor port is not configured")
    endif()

    if(CONFIG_ARBATOS_TARGET_SENTINEL_M)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/subboard/SubBoardBringupZephyr.c)
    endif()
endif()

target_include_directories(app BEFORE PRIVATE
    ${ARBATOS_ROOT}/shared/zephyr/include
    ${ARBATOS_ROOT}/shared/zephyr/compat/cmsis
    ${ARBATOS_ROOT}/shared/zephyr/compat/freertos
    ${ARBATOS_ROOT}/shared/zephyr/compat/stm32
    ${ARBATOS_ROOT}/shared/zephyr/port/algorithm/include
    ${ARBATOS_ROOT}/shared/zephyr/port/can
    ${ARBATOS_ROOT}/shared/zephyr/port/uart
    ${ARBATOS_ROOT}/shared/zephyr/port/sensors
    ${ARBATOS_ROOT}/shared/zephyr/port/storage
    ${ARBATOS_ROOT}/shared/zephyr/port/platform
    ${ARBATOS_ROOT}/shared/zephyr/port/subboard
    ${ARBATOS_ROOT}/shared/zephyr/port/usb
)

# The existing Robotconfig and shared trees were written for STM32 HAL and
# FreeRTOS.  Keep their exact source locations in the build graph, but make
# reuse opt-in until the Zephyr port layer supplies every required interface.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ArbatosLegacy.cmake)
arbatos_add_legacy_sources(app)
