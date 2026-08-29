cmake_minimum_required(VERSION 3.22)

# 设置项目名称
set(CMAKE_PROJECT_NAME infantry)

# 设置 C 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

#####################添加C++编译与版本配置#################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


set(SIZE arm-none-eabi-size)

# 定义构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 启用编译命令导出（便于 clangd 等工具索引）
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Add color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

# 包含工具链文件（必须在 project() 前设置编译器）
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

# 核心项目设置
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 启用 C 和 ASM 语言支持
enable_language(C CXX ASM)

# 禁用编译时警告输出
# add_compile_options(-w)

# 创建可执行文件
add_executable(${CMAKE_PROJECT_NAME})

# 添加 STM32CubeMX 生成的源码
add_subdirectory(cmake/stm32cubemx)

##############以下为修改过的cmakelist#################
function(USER_ADD_DIR dir)
    file(GLOB USER_SOURCE
        ${dir}/*.h
        ${dir}/*.c
        ${dir}/*.hpp
        ${dir}/*.cpp
        ${dir}/*.cc
    )
    target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${USER_SOURCE})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${dir})
endfunction()
###############手动添加文件路径#########
# 只需要添加根目录，函数会递归扫描所有子目录




#######################################


###############自动扫描project目录###############
function(AUTO_SCAN_PROJECT_DIR base_dir)
    # 递归查找所有 .c, .cpp, .cc, .h, .hpp 文件
    file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
        ${base_dir}/*.c
        ${base_dir}/*.cpp
        ${base_dir}/*.cc
        ${base_dir}/*.h
        ${base_dir}/*.hpp
    )

    if(PROJECT_SOURCES)
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${PROJECT_SOURCES})

        # 收集所有包含头文件的目录
        set(INCLUDE_DIRS "")
        foreach(source_file ${PROJECT_SOURCES})
            get_filename_component(source_dir "${source_file}" DIRECTORY)
            list(APPEND INCLUDE_DIRS "${source_dir}")
        endforeach()

        # 去重并添加包含目录
        list(REMOVE_DUPLICATES INCLUDE_DIRS)
        target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${INCLUDE_DIRS})

        message(STATUS "Auto-scanned ${base_dir}:")
        message(STATUS "  Found ${PROJECT_SOURCES} source files")
        message(STATUS "  Added ${INCLUDE_DIRS} include directories")
    else()
        message(STATUS "No source files found in ${base_dir}")
    endif()
endfunction()

if(EXISTS ${CMAKE_SOURCE_DIR}/project)
    AUTO_SCAN_PROJECT_DIR(${CMAKE_SOURCE_DIR}/project)
endif()

##################编译选项#################
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:
        -fno-rtti
        -fno-exceptions
        -fno-threadsafe-statics
    >
)

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
)
