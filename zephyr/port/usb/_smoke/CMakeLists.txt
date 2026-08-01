cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
find_package(Zephyr 4.4 REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arbatos_usb_cdc_smoke)

target_sources(app PRIVATE
    main.c
    ../BspZephyrUsbCdc.c
)

target_include_directories(app PRIVATE ..)
