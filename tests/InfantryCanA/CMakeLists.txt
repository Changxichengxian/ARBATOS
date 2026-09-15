cmake_minimum_required(VERSION 3.20.0)
get_filename_component(ARBATOS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
list(APPEND BOARD_ROOT ${ARBATOS_ROOT})
list(APPEND DTS_ROOT ${ARBATOS_ROOT}/boards)
find_package(Zephyr 4.4 REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(infantry_can_a_rx)

target_sources(app PRIVATE main.c)
