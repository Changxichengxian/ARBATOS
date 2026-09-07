cmake_minimum_required(VERSION 3.20.0)
get_filename_component(ARBATOS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
list(APPEND BOARD_ROOT ${ARBATOS_ROOT})
list(APPEND DTS_ROOT ${ARBATOS_ROOT}/boards)
find_package(Zephyr 4.4 REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arbatos_board_check)

target_sources(app PRIVATE main.c
    ${ARBATOS_ROOT}/shared/zephyr/port/platform/BspKey.c)
if(CONFIG_BOARD_DJI_A_F427)
    target_sources(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors/Mpu6500.c)
    target_include_directories(app PRIVATE ${ARBATOS_ROOT}/shared/zephyr/port/sensors)
endif()
