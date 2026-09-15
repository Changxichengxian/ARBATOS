# ARBATOS Zephyr source manifest for shared robot control.
#
# This maintained list is the source of truth for the three Zephyr targets.
# It was initially transcribed from the old projects, but configure/build
# never reads a Keil project or requires an installed Keil toolchain.
#
# Do not add generated CubeMX files, startup assembly, FreeRTOS, STM32 HAL,
# legacy BSP implementations, or ARMCC .lib files here.  Zephyr owns those
# responsibilities.  The omitted board files are recorded below so that a
# future port does not accidentally resurrect the old startup path.

set(ARBATOS_LEGACY_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(arbatos_add_legacy_sources target)
    if(NOT CONFIG_ARBATOS_LEGACY_SOURCES)
        return()
    endif()

    # CMAKE_CURRENT_LIST_DIR changes to the caller while a CMake function is
    # running, so preserve this file's location at include time above.
    set(ARBATOS_ROOT "${ARBATOS_LEGACY_CMAKE_DIR}/../..")

    # Shared robot sources are selected here. Old platform calls are
    # supplied by the Zephyr compatibility layer, never by old startup,
    # FreeRTOS, STM32 HAL, or ARMCC binary-library source entries.
    set(ARBATOS_COMMON_CORE
        shared/application/input/ControlInput.c
        shared/application/input/ManualInput.c
        shared/application/input/RcSbusTask.c
        shared/application/robot/LowCmd.c
        shared/application/robot/RobotLifecycle.c
        shared/application/robot/StateStore.c
        shared/application/robot/FaultMgr.c
        shared/application/robot/ExternalMotionIntent.c
        shared/application/robot/ControlMgr.c
        shared/application/robot/ControlRuntime.c
        shared/application/robot/ControlRuntimeTask.c
        shared/application/robot/RobotConfigBlocks.c
        shared/application/motors/MotorModelDb.c
        shared/application/motors/MotorInst.c
        shared/application/motors/MotorHealth.c
        shared/application/comm/can/CanReceive.c
        shared/application/comm/can/CanRxTask.c
        shared/application/comm/can/PowerMeter.c
        shared/application/comm/can/CanTxTask.c
        shared/application/motors/CanMitMotorDriver.c
        shared/application/wheelleg/WheelLegMitTask.c
        shared/application/services/diagnostics/RtProf.c
        shared/application/chassis/ChassisBehaviour.c
        shared/application/chassis/ChassisPowerControl.c
        shared/application/chassis/ChassisControlTask.c
        shared/application/chassis/ChassisCtrl.c
        shared/application/gimbal/GimbalBehaviour.c
        shared/application/gimbal/GimbalControlTask.c
        shared/application/services/buzzer/BuzzerFilePlayer.c
        shared/application/comm/referee/Referee.c
        shared/application/shoot/Shoot.c
        shared/application/shoot/ShootCtrl.c
        shared/application/services/diagnostics/Watch.c
        shared/application/services/diagnostics/RobotFaultGuard.c
        shared/components/algorithm/AhrsMiddleware.c
        shared/components/algorithm/KalmanFilter.c
        shared/components/algorithm/UserLib.c
        shared/components/controller/AxisCurrentConditioner.c
        shared/components/controller/ChassisPowerLimiter.c
        shared/components/controller/GimbalPid.c
        shared/components/controller/Pid.c
        shared/components/support/Crc8Crc16.c
    )

    # 经典底盘、单云台和 RM 射击的实现仍会调用 SD 日志、俯仰校准和
    # 多协议电机的兼容符号，即使车型没有启用它们对应的任务也必须链接。
    # 这些实现不等于开放 A 板服务，车型选择由 RobotConfig.toml 生成器限制。
    set(ARBATOS_PORTABLE_RUNTIME
        ${ARBATOS_COMMON_CORE}
        shared/application/arm/ArmTask.c
        shared/application/arm/ArmMotion.c
        shared/application/services/battery/BatteryMonitorTask.c
        shared/application/motors/UnitreeMotorDriver.c
        shared/application/motors/N6014bMotorDriver.c
        shared/application/services/calibration/PitchCali.c
        shared/application/services/storage/SdCard.c
        shared/application/services/storage/SdLog.c
        shared/application/services/storage/SdLogTask.c
        shared/application/comm/referee/RefereeRxTask.c
        shared/application/services/diagnostics/CpuUsage.c
        shared/components/devices/SdSpi.c
        shared/components/support/Fifo.c
        shared/components/support/MemMang4.c
        shared/components/support/fatfs/ff.c
        shared/components/support/fatfs/ffsystem.c
        shared/components/support/fatfs/ffunicode.c
    )
    # H723 的 IMU 路径不使用 AhrsMiddleware；A 板改用 MPU6500，不能
    # 编入 BMI088 驱动及其专属设备树端口。
    if(CONFIG_BOARD_DM_MC02_H7)
        list(REMOVE_ITEM ARBATOS_PORTABLE_RUNTIME
            shared/components/algorithm/AhrsMiddleware.c)
        list(APPEND ARBATOS_PORTABLE_RUNTIME
            shared/components/devices/Bmi088Driver.c)
    endif()

    # 车型差异由 RobotConfig.toml 生成的 RobotTarget.cmake 提供。
    if(NOT DEFINED ARBATOS_TARGET_DIR OR NOT DEFINED ARBATOS_BOARD_DIR OR
       NOT DEFINED ARBATOS_TARGET_SOURCES)
        message(FATAL_ERROR "RobotTarget.cmake is incomplete; regenerate the selected Robotconfig")
    endif()
    set(ARBATOS_TARGET_SOURCES
        ${ARBATOS_PORTABLE_RUNTIME}
        ${ARBATOS_TARGET_SOURCES}
        ${ARBATOS_TARGET_PORT_SOURCES}
    )

    list(TRANSFORM ARBATOS_TARGET_SOURCES PREPEND "${ARBATOS_ROOT}/")
    target_sources(${target} PRIVATE ${ARBATOS_TARGET_SOURCES})

    # The old projects use headers with no uniform include root.  Header
    # discovery is restricted to the selected target and its actual board,
    # which keeps one vehicle from silently relying on another vehicle's API.
    # 只扫描项目使用的目录，避免暂存的第三方源码污染头文件搜索路径。
    file(GLOB_RECURSE ARBATOS_LEGACY_HEADERS CONFIGURE_DEPENDS
        "${ARBATOS_ROOT}/shared/application/*.h"
        "${ARBATOS_ROOT}/shared/components/*.h"
        "${ARBATOS_ROOT}/shared/controllers/*.h"
        "${ARBATOS_ROOT}/shared/generated/*.h"
        "${ARBATOS_ROOT}/shared/hal/*.h"
        "${ARBATOS_ROOT}/shared/zephyr/*.h"
        "${ARBATOS_ROOT}/Robotconfig/${ARBATOS_TARGET_DIR}/*.h"
        "${ARBATOS_ROOT}/${ARBATOS_BOARD_DIR}/*.h")
    set(ARBATOS_LEGACY_INCLUDE_DIRS "")
    foreach(header ${ARBATOS_LEGACY_HEADERS})
        get_filename_component(header_dir "${header}" DIRECTORY)
        list(APPEND ARBATOS_LEGACY_INCLUDE_DIRS "${header_dir}")
    endforeach()
    list(REMOVE_DUPLICATES ARBATOS_LEGACY_INCLUDE_DIRS)
    target_include_directories(${target} PRIVATE ${ARBATOS_LEGACY_INCLUDE_DIRS})

    # Deliberately excluded Keil entries.  These files are tracked here as
    # board coverage, but must be reimplemented through Zephyr drivers rather
    # than compiled beside them:
    # - shared/hal/*.c (STM32 HAL, CMSIS-RTOS, CubeMX handles)
    # - boards/DmMc02H7/app/{BoardMain,BoardFreertos,InsTask}.c and bsp/**/*.c
    # - shared/components/algorithm/{AHRS,arm_cortexM4lf_math}.lib
    # - projects/*/{Core,Drivers,Middlewares,USB_DEVICE}/** and startup *.s
endfunction()
