cmake_minimum_required(VERSION 3.22)

# 项目名称
set(CMAKE_PROJECT_NAME 27_infantry)   # TODO: 改成你的工程名

# C / C++ 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 导出编译命令（clangd 索引用）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

# 工具链（必须在 project() 之前）
# 注意：CubeMX 生成代码时可能删掉下面这行，所以工具链优先由
# CMakePresets.json 的 CMAKE_TOOLCHAIN_FILE 提供，这里仅作无 preset 时的兜底
if(NOT CMAKE_TOOLCHAIN_FILE)
    include("cmake/gcc-arm-none-eabi.cmake")
endif()

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
enable_language(C CXX ASM)

# 保险：如果工具链 include 丢失，CMake 会回退到宿主编译器，这里直接报错
if(NOT CMAKE_C_COMPILER MATCHES "arm-none-eabi")
    message(FATAL_ERROR "未检测到 ARM GCC 工具链！请确认 CMakeLists.txt 中 include(\"cmake/gcc-arm-none-eabi.cmake\") 这一行存在")
endif()

# 芯片宏定义（换芯片改这里；CubeMX CMake 导出后由 cmake/stm32cubemx 提供，自动跳过）
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx)
    add_compile_definitions(USE_HAL_DRIVER STM32F407xx)   # TODO: STM32F103xB / STM32H723xx 等
endif()

add_executable(${CMAKE_PROJECT_NAME})

# ---------- CubeMX 生成部分 ----------
if(EXISTS ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx)
    # CubeMX CMake 导出：Core / Drivers / Middlewares 由它负责
    add_subdirectory(cmake/stm32cubemx)
else()
    # 尚未导出时自动扫描 CubeMX 目录
    file(GLOB_RECURSE CUBEMX_SRC CONFIGURE_DEPENDS
        ${CMAKE_SOURCE_DIR}/Core/*.c
        ${CMAKE_SOURCE_DIR}/Core/*.s
        ${CMAKE_SOURCE_DIR}/Drivers/*.c
        ${CMAKE_SOURCE_DIR}/Middlewares/*.c)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CUBEMX_SRC})
    file(GLOB_RECURSE CUBEMX_INC CONFIGURE_DEPENDS
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/**/Inc
        ${CMAKE_SOURCE_DIR}/Middlewares/**/Inc)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CUBEMX_INC})
endif()

# 链接脚本（CubeMX 生成的 *.ld，放在工程根目录或 Core/）
file(GLOB LD_SCRIPT CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/*.ld ${CMAKE_SOURCE_DIR}/Core/*.ld)
if(LD_SCRIPT)
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -T ${LD_SCRIPT})
endif()

# ---------- 用户代码：递归扫描 project/，新增文件无需改 CMake ----------
file(GLOB_RECURSE USER_SRC CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/project/*.c
    ${CMAKE_SOURCE_DIR}/project/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SRC})

file(GLOB_RECURSE USER_HDR CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/project/*.h
    ${CMAKE_SOURCE_DIR}/project/*.hpp)
foreach(hdr ${USER_HDR})
    get_filename_component(hdr_dir ${hdr} DIRECTORY)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${hdr_dir})
endforeach()

# ---------- DSP 库（可选，按芯片启用） ----------
# target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib)
# target_link_libraries(${CMAKE_PROJECT_NAME} -lc -lm -lnosys libarm_cortexM4lf_math.a)

# 生成 hex / bin
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Building hex & bin..."
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)
