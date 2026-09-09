/**
 * @file    bsp_spi.h
 * @brief   SPI 软件片选主机（C → C++）
 * @note    从 C 版 bsp_spi 迁移：struct SPI_Instance → class SPI，Register → init、
 *          Transfer → transfer，逻辑不变，禁堆（原 malloc 实例改为栈/全局对象）。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include "spi.h"
#include <stdint.h>

class SPI {
public:
    /// 初始化配置
    struct Config {
        SPI_HandleTypeDef* hspi;    ///< SPI 句柄（CubeMX 配置为主机、无 DMA/中断）
        GPIO_TypeDef* cs_port;      ///< 软件片选端口
        uint16_t cs_pin;            ///< 软件片选引脚
    };

    // —— 实例变量 ——
private:
    SPI_HandleTypeDef* hspi_;   ///< SPI 句柄
    GPIO_TypeDef* cs_port_;     ///< 软件片选端口
    uint16_t cs_pin_;           ///< 软件片选引脚

    // —— 实例函数 ——
public:
    void init(const Config& config);                          ///< 替代 SPI_Register（禁堆）
    void transfer(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len);  ///< 轮询 SPI 传输
};
