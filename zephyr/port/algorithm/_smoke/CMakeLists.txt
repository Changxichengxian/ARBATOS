cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
find_package(Zephyr 4.4 REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arbatos_algorithm_smoke)

target_sources(app PRIVATE
    main.c
    ../AhrsZephyr.c
    ../ArmMathZephyr.c
    ../../../../shared/components/algorithm/AhrsMiddleware.c
    ../../../../Robotconfig/HERO-M/Mc02Compat.c
)

target_include_directories(app PRIVATE
    ../include
    ../../../../shared/components/algorithm
    ../../../../shared/components/support
    ../../../../shared/hal
)
