# ARBATOS Zephyr 4.4 application entry.
cmake_minimum_required(VERSION 3.20.0)

get_filename_component(ARBATOS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(ARBATOS_ROBOT "HERO-M" CACHE STRING "Robotconfig target name")
file(GLOB_RECURSE ARBATOS_ROBOT_CONFIG_INPUTS CONFIGURE_DEPENDS
    "${ARBATOS_ROOT}/Robotconfig/*/RobotConfig.toml"
    "${ARBATOS_ROOT}/boards/*/PortCatalog.toml"
    "${ARBATOS_ROOT}/shared/controllers/*/Controller.toml")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${ARBATOS_ROOT}/tools/config/RobotConfigGen.py"
    "${ARBATOS_ROOT}/tools/config/SourcePolicy.py"
    "${ARBATOS_ROOT}/shared/application/robot/RobotTaskCatalog.def"
    ${ARBATOS_ROBOT_CONFIG_INPUTS})

# 车型资料只在 Robotconfig/<车型>/RobotConfig.toml 中维护。这里在
# find_package(Zephyr) 前生成配置，确保 BOARD、Kconfig 片段和 overlay 都来自同一份声明。
if(NOT DEFINED Python3_EXECUTABLE AND DEFINED WEST_PYTHON)
    set(Python3_EXECUTABLE "${WEST_PYTHON}" CACHE FILEPATH "Python used by west")
endif()
find_package(Python3 3.12 REQUIRED COMPONENTS Interpreter)
set(ARBATOS_TARGET_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated/robotconfig")
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -X utf8 "${ARBATOS_ROOT}/tools/config/RobotConfigGen.py"
        generate --target "${ARBATOS_ROBOT}" --out "${ARBATOS_TARGET_GENERATED_DIR}"
    WORKING_DIRECTORY "${ARBATOS_ROOT}"
    RESULT_VARIABLE ARBATOS_TARGET_GENERATE_RESULT
    OUTPUT_VARIABLE ARBATOS_TARGET_GENERATE_OUTPUT
    ERROR_VARIABLE ARBATOS_TARGET_GENERATE_ERROR
)
if(NOT ARBATOS_TARGET_GENERATE_RESULT EQUAL 0)
    message(FATAL_ERROR "Cannot generate robot configuration for ${ARBATOS_ROBOT}: ${ARBATOS_TARGET_GENERATE_ERROR}${ARBATOS_TARGET_GENERATE_OUTPUT}")
endif()
include("${ARBATOS_TARGET_GENERATED_DIR}/RobotTarget.cmake")
if(NOT DEFINED ARBATOS_BOARD OR "${ARBATOS_BOARD}" STREQUAL "")
    message(FATAL_ERROR "RobotTarget.cmake did not provide ARBATOS_BOARD")
endif()
if(DEFINED BOARD AND NOT "${BOARD}" STREQUAL "${ARBATOS_BOARD}")
    message(FATAL_ERROR "BOARD=${BOARD} conflicts with ${ARBATOS_ROBOT} board ${ARBATOS_BOARD}")
endif()
set(BOARD "${ARBATOS_BOARD}" CACHE STRING "Zephyr board selected by ARBATOS_ROBOT" FORCE)
set(ARBATOS_TARGET_CONF "${ARBATOS_TARGET_GENERATED_DIR}/robot.conf")
set(ARBATOS_OLD_TARGET_CONF "${CMAKE_CURRENT_SOURCE_DIR}/${ARBATOS_ROBOT}/prj.conf")
# 生成的 robot.conf 永远排在前面。专用音乐、准备和只接收配置只能追加；
# 它们不能重新选择车型或替换正式车型的基础能力。
set(ARBATOS_EXTRA_CONF_FILES "")
if(DEFINED EXTRA_CONF_FILE)
    foreach(ARBATOS_CONF IN LISTS EXTRA_CONF_FILE)
        if("${ARBATOS_CONF}" STREQUAL "" OR "${ARBATOS_CONF}" STREQUAL "${ARBATOS_TARGET_CONF}")
            continue()
        endif()
        if("${ARBATOS_CONF}" STREQUAL "${ARBATOS_OLD_TARGET_CONF}")
            # 旧缓存只保留同车型路径的一次迁移兼容，不能再次叠加。
            continue()
        endif()
        get_filename_component(ARBATOS_CONF_ABSOLUTE "${ARBATOS_CONF}" ABSOLUTE
            BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        if(NOT EXISTS "${ARBATOS_CONF_ABSOLUTE}")
            message(FATAL_ERROR "EXTRA_CONF_FILE references missing file: ${ARBATOS_CONF}")
        endif()
        file(STRINGS "${ARBATOS_CONF_ABSOLUTE}" ARBATOS_CONF_IDENTITY
            REGEX "^[ \t]*CONFIG_ARBATOS_(ROBOT_NAME|TARGET_[A-Z0-9_]+|LEGACY_SOURCES)=")
        if(ARBATOS_CONF_IDENTITY)
            message(FATAL_ERROR "Extra configuration ${ARBATOS_CONF} must not select or replace a robot identity")
        endif()
        list(APPEND ARBATOS_EXTRA_CONF_FILES "${ARBATOS_CONF_ABSOLUTE}")
    endforeach()
endif()
list(REMOVE_DUPLICATES ARBATOS_EXTRA_CONF_FILES)
set(EXTRA_CONF_FILE "${ARBATOS_TARGET_CONF};${ARBATOS_EXTRA_CONF_FILES}" CACHE STRING
    "Generated robot configuration followed by optional test fragments" FORCE)
if(ARBATOS_TARGET_OVERLAY)
    set(ARBATOS_OLD_TARGET_OVERLAY "${CMAKE_CURRENT_SOURCE_DIR}/${ARBATOS_ROBOT}/app.overlay")
    if(DEFINED DTC_OVERLAY_FILE AND NOT "${DTC_OVERLAY_FILE}" STREQUAL "${ARBATOS_TARGET_OVERLAY}" AND
       NOT "${DTC_OVERLAY_FILE}" STREQUAL "${ARBATOS_OLD_TARGET_OVERLAY}")
        message(FATAL_ERROR "DTC_OVERLAY_FILE=${DTC_OVERLAY_FILE} conflicts with ${ARBATOS_ROBOT} overlay ${ARBATOS_TARGET_OVERLAY}")
    endif()
    set(DTC_OVERLAY_FILE "${ARBATOS_TARGET_OVERLAY}" CACHE FILEPATH "Robot device-tree overlay" FORCE)
elseif(DEFINED DTC_OVERLAY_FILE AND NOT "${DTC_OVERLAY_FILE}" STREQUAL "")
    message(FATAL_ERROR "DTC_OVERLAY_FILE is set but ${ARBATOS_ROBOT} has no declared overlay")
endif()

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

    if(CONFIG_BOARD_DJI_A_F427)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors/Mpu6500.c)
    elseif(CONFIG_BOARD_DJI_C_F407)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors/BspBmi088Port.c ${ARBATOS_ROOT}/shared/zephyr/port/sensors/BspIst8310Port.c)
    elseif(CONFIG_BOARD_DM_MC02_H7)
        target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors/BspBmi088Port.c)
    else()
        message(FATAL_ERROR "Board sensor port is not configured")
    endif()

endif()

target_include_directories(app BEFORE PRIVATE
    ${ARBATOS_TARGET_GENERATED_DIR}
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
