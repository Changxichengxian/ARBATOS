cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
find_package(Zephyr 4.4 REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arbatos_can_smoke)

target_sources(app PRIVATE
    main.c
    ../BspCanZephyr.c
)

target_include_directories(app PRIVATE
    ..
    ../../../../shared/hal
    ../../../../shared/components/support
    ../../../compat/freertos
)
