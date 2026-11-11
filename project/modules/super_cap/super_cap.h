/** @file super_cap.h @brief 超电协议接入接口；帧编解码待本车协议确定。 */
#pragma once

#include "daemon.h"

class SuperCap {
public:
    struct Config {
        Daemon::Config daemon;
    };
    struct Feedback {
        float output_power_w;  ///< 超电向底盘输出的电功率，W；不是裁判输入功率。
    };
    struct Command {
        float input_power_limit_w;  ///< 输入功率目标，W；协议适配层负责实际发送。
    };

private:
    Feedback feedback_ = {};
    Daemon daemon_;

public:
    void init(const Config& config);
    /// 由未来协议解析器在收到有效新样本后调用，非有限功率不喂狗。
    void receive(const Feedback& feedback);
    bool isOnline() const;
    const Feedback& feedback() const;
};
