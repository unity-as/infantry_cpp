#ifndef CRC_H
#define CRC_H

#include <stdint.h>

/*
 * CRC 模块 - RoboMaster 专用
 * 仅用于实现 RoboMaster 场景下的协议校验
 * 不具备泛用性和可移植性
 *
 * CRC-8/MAXIM(反射):  多项式 0x31(反射 0x8C),   初始值 0xFF   (裁判系统帧头校验)
 * CRC-16/CCITT(反射): 多项式 0x1021(反射 0x8408), 初始值 0xFFFF (裁判系统 + 图传整包校验)
 */

#define CRC_USE_TABLE  1

// CRC-8/MAXIM (裁判系统帧头, 反射 LSB-first)
#define CRC8_POLY      0x31   // 正常形式 (文档用)
#define CRC8_POLY_REV  0x8C   // 反射形式 (代码实际使用)
#define CRC8_INIT      0xFF

// CRC-16/CCITT (裁判系统 + 图传整包, 反射 LSB-first)
#define CRC16_POLY      0x1021  // 正常形式 (文档用)
#define CRC16_POLY_REV  0x8408  // 反射形式 (代码实际使用)
#define CRC16_INIT      0xFFFF

// CRC-8
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t len);
uint8_t CRC8_Verify(const uint8_t *data, uint16_t len);

// CRC-16
uint16_t CRC16_Calculate(const uint8_t *data, uint16_t len);
uint8_t  CRC16_Verify(const uint8_t *data, uint16_t len);

#endif
