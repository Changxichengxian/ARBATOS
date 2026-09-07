# 每个构建目录各自生成版本头；脚本仅在内容变化时改写该文件。
set(ARBATOS_BUILD_INFO_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(arbatos_add_build_info target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "arbatos_add_build_info: target '${target}' 不存在")
    endif()

    if(NOT Python3_EXECUTABLE)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
    endif()

    get_filename_component(ARBATOS_BUILD_INFO_ROOT "${ARBATOS_BUILD_INFO_CMAKE_DIR}/../.." ABSOLUTE)
    set(ARBATOS_BUILD_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
    set(ARBATOS_BUILD_INFO_HEADER "${ARBATOS_BUILD_INFO_DIR}/build_info_autogen.h")
    set(ARBATOS_BUILD_INFO_TARGET "${target}_build_info")

    add_custom_target(${ARBATOS_BUILD_INFO_TARGET} ALL
        COMMAND "${Python3_EXECUTABLE}" "${ARBATOS_BUILD_INFO_ROOT}/tools/build/GenBuildInfo.py"
            --repo "${ARBATOS_BUILD_INFO_ROOT}"
            --output "${ARBATOS_BUILD_INFO_HEADER}"
        BYPRODUCTS "${ARBATOS_BUILD_INFO_HEADER}"
        COMMENT "更新 ${target} 构建版本信息"
        VERBATIM)
    add_dependencies(${target} ${ARBATOS_BUILD_INFO_TARGET})
    target_include_directories(${target} PRIVATE "${ARBATOS_BUILD_INFO_DIR}")
    target_compile_definitions(${target} PRIVATE ARBATOS_BUILD_INFO_AUTOGEN_AVAILABLE=1)
endfunction()
