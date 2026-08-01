# ARBATOS legacy source manifest.
#
# This is intentionally an explicit transcription of the seven Keil projects,
# not a recursive source glob.  It makes a target's ownership auditable while
# Zephyr progressively replaces the STM32/FreeRTOS boundary.
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

    # All source paths in this section appear in at least one project under
    # projects/*/MDK-ARM/*.uvprojx.  Any old platform calls they make are
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
        shared/application/motors/MotorModelDb.c
        shared/application/motors/MotorInst.c
        shared/application/motors/MotorHealth.c
        shared/application/comm/can/CanReceive.c
        shared/application/comm/can/CanRxTask.c
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

    set(ARBATOS_FULL_FEATURE_CORE
        ${ARBATOS_COMMON_CORE}
        shared/application/services/calibration/CalibrateTask.c
        shared/application/services/calibration/PitchCali.c
        shared/application/services/diagnostics/CpuUsage.c
        shared/application/comm/referee/RefereeRxTask.c
        shared/application/services/startup/StartupServiceTask.c
        shared/application/input/ElrsTask.c
        shared/application/comm/host/HostLinkTask.c
        shared/application/comm/host/AuxPort.c
        shared/application/comm/host/AuxTune.c
        shared/application/comm/host/AuxParam.c
        shared/application/comm/host/AuxAutotune.c
        shared/application/comm/host/AuxTelem.c
        shared/application/comm/host/HostTuneBridge.c
        shared/application/comm/vision/VisionLink.c
        shared/application/input/ImageRemoteLink.c
        shared/application/services/battery/BatteryMonitorTask.c
        shared/application/services/startup/StatusLedTask.c
        shared/application/services/servo/ServoControlTask.c
        shared/application/services/storage/SdCard.c
        shared/application/services/storage/SdLog.c
        shared/application/services/storage/SdLogTask.c
        shared/components/devices/Bmi088Driver.c
        shared/components/devices/Ist8310Driver.c
        shared/components/devices/SdSpi.c
        shared/components/support/Fifo.c
        shared/components/support/MemMang4.c
        shared/components/support/fatfs/ff.c
        shared/components/support/fatfs/ffsystem.c
        shared/components/support/fatfs/ffunicode.c
        shared/components/controller/PidAdvanced.c
    )

    set(ARBATOS_H7_CORE
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
        shared/components/devices/Bmi088Driver.c
        shared/components/devices/SdSpi.c
        shared/components/support/Fifo.c
        shared/components/support/MemMang4.c
        shared/components/support/fatfs/ff.c
        shared/components/support/fatfs/ffsystem.c
        shared/components/support/fatfs/ffunicode.c
    )
    # The H723 Keil targets link the same control stack but do not include
    # AhrsMiddleware.c; their IMU path is assembled differently.
    list(REMOVE_ITEM ARBATOS_H7_CORE
        shared/components/algorithm/AhrsMiddleware.c)

    # Source entries below are target-specific deltas from the Keil manifest.
    # Do not merge targets merely because their vehicle names sound similar.
    if(CONFIG_ARBATOS_TARGET_HERO_C)
        set(ARBATOS_TARGET_DIR HERO-C)
        set(ARBATOS_BOARD_DIR boards/DjiCF407)
        set(ARBATOS_TARGET_SOURCES
            ${ARBATOS_FULL_FEATURE_CORE}
            Robotconfig/HERO-C/RobotConfig.c
            Robotconfig/HERO-C/DetectTask.c
            Robotconfig/HERO-C/PitchCaliBuiltin.c
        )
    elseif(CONFIG_ARBATOS_TARGET_MINIWHEELEG_C)
        set(ARBATOS_TARGET_DIR MINIWHEELEG-C)
        set(ARBATOS_BOARD_DIR boards/DjiCF407)
        set(ARBATOS_TARGET_SOURCES
            ${ARBATOS_FULL_FEATURE_CORE}
            Robotconfig/MINIWHEELEG-C/RobotConfig.c
            Robotconfig/MINIWHEELEG-C/DetectTask.c
        )
    elseif(CONFIG_ARBATOS_TARGET_CARRIER_A)
        set(ARBATOS_TARGET_DIR CARRIER-A)
        set(ARBATOS_BOARD_DIR boards/DjiAF427)
        set(ARBATOS_TARGET_SOURCES
            ${ARBATOS_COMMON_CORE}
            shared/application/services/calibration/PitchCali.c
            shared/application/services/storage/SdCard.c
            shared/application/services/storage/SdLog.c
            shared/application/services/storage/SdLogTask.c
            shared/application/comm/host/HostLinkTaskStub.c
            shared/components/support/MemMang4.c
            shared/components/support/fatfs/ff.c
            shared/components/support/fatfs/ffsystem.c
            shared/components/support/fatfs/ffunicode.c
            Robotconfig/CARRIER-A/RobotConfig.c
            Robotconfig/CARRIER-A/DetectTask.c
        )
    elseif(CONFIG_ARBATOS_TARGET_INFANTRY_A)
        set(ARBATOS_TARGET_DIR INFANTRY-A)
        set(ARBATOS_BOARD_DIR boards/DjiAF427)
        set(ARBATOS_TARGET_SOURCES
            ${ARBATOS_COMMON_CORE}
            shared/application/services/calibration/PitchCali.c
            shared/application/services/storage/SdCard.c
            shared/application/services/storage/SdLog.c
            shared/application/services/storage/SdLogTask.c
            shared/application/input/ElrsTask.c
            shared/application/comm/host/HostLinkTask.c
            shared/application/comm/host/AuxPort.c
            shared/application/comm/host/AuxTune.c
            shared/application/comm/host/AuxParam.c
            shared/application/comm/host/AuxAutotune.c
            shared/application/comm/host/AuxTelem.c
            shared/application/comm/host/HostTuneBridge.c
            shared/application/comm/vision/VisionLink.c
            shared/application/input/ImageRemoteLink.c
            shared/components/support/MemMang4.c
            shared/components/support/fatfs/ff.c
            shared/components/support/fatfs/ffsystem.c
            shared/components/support/fatfs/ffunicode.c
            Robotconfig/INFANTRY-A/RobotConfig.c
            Robotconfig/INFANTRY-A/DetectTask.c
        )
    elseif(CONFIG_ARBATOS_TARGET_HERO_M)
        set(ARBATOS_TARGET_DIR HERO-M)
        set(ARBATOS_BOARD_DIR boards/DmMc02H7)
        set(ARBATOS_TARGET_SOURCES
            ${ARBATOS_H7_CORE}
            shared/application/comm/host/HostLinkTaskStub.c
            Robotconfig/HERO-M/RobotConfig.c
            Robotconfig/HERO-M/Mc02Compat.c
            Robotconfig/HERO-M/DetectTask.c
            Robotconfig/HERO-M/ArmMotorTable.c
            Robotconfig/HERO-M/PitchCaliBuiltin.c
        )
    elseif(CONFIG_ARBATOS_TARGET_MINIWHEELEG_M)
        set(ARBATOS_TARGET_DIR MINIWHEELEG-M)
        set(ARBATOS_BOARD_DIR boards/DmMc02H7)
        set(ARBATOS_TARGET_SOURCES
            ${ARBATOS_H7_CORE}
            shared/application/comm/host/HostLinkTaskStub.c
            Robotconfig/MINIWHEELEG-M/RobotConfig.c
            Robotconfig/MINIWHEELEG-M/Mc02Compat.c
            Robotconfig/MINIWHEELEG-M/DetectTask.c
            Robotconfig/MINIWHEELEG-M/ArmMotorTable.c
        )
    elseif(CONFIG_ARBATOS_TARGET_SENTINEL_M)
        set(ARBATOS_TARGET_DIR SENTINEL-M)
        set(ARBATOS_BOARD_DIR boards/DmMc02H7)
        set(ARBATOS_TARGET_SOURCES
            ${ARBATOS_H7_CORE}
            shared/application/comm/vision/VisionLink.c
            Robotconfig/SENTINEL-M/RobotConfig.c
            Robotconfig/SENTINEL-M/Mc02Compat.c
            Robotconfig/SENTINEL-M/DetectTask.c
            Robotconfig/SENTINEL-M/ArmMotorTable.c
            Robotconfig/SENTINEL-M/UsbHostLinkTask.c
        )
    else()
        message(FATAL_ERROR "ARBATOS legacy sources need one ARBATOS_TARGET selection")
    endif()

    list(TRANSFORM ARBATOS_TARGET_SOURCES PREPEND "${ARBATOS_ROOT}/")
    target_sources(${target} PRIVATE ${ARBATOS_TARGET_SOURCES})

    # The old projects use headers with no uniform include root.  Header
    # discovery is restricted to the selected target and its actual board,
    # which keeps one vehicle from silently relying on another vehicle's API.
    file(GLOB_RECURSE ARBATOS_LEGACY_HEADERS CONFIGURE_DEPENDS
        "${ARBATOS_ROOT}/shared/*.h"
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
    # - boards/DjiCF407/{bsp,devices}/**/*.c and boards/DjiAF427/{bsp,devices}/**/*.c
    # - boards/DmMc02H7/app/{BoardMain,BoardFreertos,InsTask}.c and bsp/**/*.c
    # - shared/components/algorithm/{AHRS,arm_cortexM4lf_math}.lib
    # - projects/*/{Core,Drivers,Middlewares,USB_DEVICE}/** and startup *.s
endfunction()
