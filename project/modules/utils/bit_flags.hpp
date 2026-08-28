/**
 * @file    bit_flags.hpp
 * @brief   位标志枚举运算符重载宏（为 enum class 提供 | & ~ 位运算）
 * @note    用法：定义 enum class Feature 后，在类外调用 ENABLE_BITWISE_OPS(Feature)
 *          C++ 的 enum class 不能隐式位运算，本宏生成 constexpr 运算符重载，
 *          编译期零开销，语法与 C 的位标志枚举一致。
 */
#pragma once

#include <cstdint>
#include <type_traits>

// 为位标志枚举生成 | & ~ 运算符重载（constexpr，编译期展开）
#define ENABLE_BITWISE_OPS(Enum)                                          \
    constexpr Enum operator|(Enum a, Enum b) {                           \
        using U = std::underlying_type_t<Enum>;                          \
        return static_cast<Enum>(static_cast<U>(a) | static_cast<U>(b)); \
    }                                                                     \
    constexpr Enum operator&(Enum a, Enum b) {                           \
        using U = std::underlying_type_t<Enum>;                          \
        return static_cast<Enum>(static_cast<U>(a) & static_cast<U>(b)); \
    }                                                                     \
    constexpr Enum operator~(Enum a) {                                   \
        using U = std::underlying_type_t<Enum>;                          \
        return static_cast<Enum>(~static_cast<U>(a));                    \
    }
