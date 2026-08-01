# Allows D:/ARBATOS/zephyr to be supplied through ZEPHYR_EXTRA_MODULES.
# Applications still use ../CMakeLists.txt as their primary build entry.
zephyr_interface_library_named(arbatos_port)
target_include_directories(arbatos_port INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/../compat/cmsis
    ${CMAKE_CURRENT_LIST_DIR}/../compat/freertos
)
