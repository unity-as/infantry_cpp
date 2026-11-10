#include "chassis_power.h"
#include "chassis_velocity.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>

static unsigned power_checks;
#define VERIFY(x) do { ++power_checks; if (!(x)) { std::fprintf(stderr, "power line %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)

// double 仅用于主机独立基准；二分寻找可行最大比例，不复用生产解析根。
static double referencePower(const ChassisPower::Config& c, const float* current, const float* speed, double scale) {
    double result = c.idle_power_w;
    for (int i = 0; i < 4; ++i) {
        const double torque = std::clamp(double(current[i]), -double(c.current_limit_a), double(c.current_limit_a)) * scale * c.torque_constant;
        const double omega = double(speed[i]) * (3.14159265358979323846 / 180.0);
        result += std::max(0.0, torque * omega) + c.speed_loss * std::abs(omega) + c.torque_loss * torque * torque;
    }
    return result;
}

unsigned testChassisPower() {
    ChassisPower manager;
    ChassisPower::Config c{0.3f, 0.22f, 1.2f, 2.78f, 20.0f};
    uint32_t random = 91;
    auto next = [&]() { random = random * 1664525u + 1013904223u; return float(random >> 8) / 16777216.0f; };
    for (int trial = 0; trial < 5000; ++trial) {
        // 二次、一次、纯铜损、空载及预算低于不可控损耗。
        c.speed_loss = trial % 3 ? 0.22f : 0.0f;
        c.torque_loss = trial % 4 ? 1.2f : 0.0f;
        manager.init(c);
        float current[4], speed[4], out[4];
        for (int i = 0; i < 4; ++i) {
            current[i] = (next() - 0.5f) * 80.0f;
            speed[i] = trial % 5 ? (next() - 0.5f) * 4000.0f : 0.0f;
        }
        const float budget = trial % 11 ? next() * 400.0f : 0.0f;
        manager.setPowerLimit(budget);
        manager.limit(current, speed, out);
        double expected = 0.0;
        if (budget > 0.0f && referencePower(c, current, speed, 0.0) < budget) {
            double lower = 0.0, upper = 1.0;
            for (int step = 0; step < 60; ++step) {
                const double middle = (lower + upper) / 2;
                if (referencePower(c, current, speed, middle) <= budget) lower = middle;
                else upper = middle;
            }
            expected = lower;
        }
        for (int i = 0; i < 4; ++i) {
            const double clipped = std::clamp(double(current[i]), -20.0, 20.0);
            VERIFY(std::isfinite(out[i]));
            VERIFY(std::abs(out[i]) <= 20.0f);
            VERIFY(std::abs(out[i] - expected * clipped) < 5e-5);
        }
        // 低于静态/转速损耗时确实无可行预算，不能把零电流报成零功率。
        VERIFY(std::abs(manager.predict(out, speed) - referencePower(c, out, speed, 1.0)) < 0.001);
    }

    c = {0.3f, 0.22f, 1.2f, 2.78f, 20.0f};
    manager.init(c);
    float current[] = {1, 2, 3, 4}, speed[] = {100, 200, 300, 400}, out[4];
    const float before = manager.predict(current, speed);
    manager.setPowerLimit(0.0f); manager.limit(current, speed, out);
    for (float value : out) VERIFY(value == 0.0f);
    manager.setPowerLimit(-20.0f); manager.limit(current, speed, out);
    for (float value : out) VERIFY(value == 0.0f);
    for (int i = 0; i < 1000; ++i) manager.limit(current, speed, out);
    VERIFY(manager.predict(current, speed) == before); // 没有反馈不更新、不遗忘。

    auto truth = c; truth.speed_loss = 0.4f; truth.torque_loss = 2.0f;
    for (int sample = 0; sample < 3000; ++sample) {
        for (int i = 0; i < 4; ++i) { current[i] = next() * 8.0f; speed[i] = next() * 1500.0f; }
        manager.updateModel(float(referencePower(truth, current, speed, 1)), current, speed);
    }
    VERIFY(std::abs(manager.predict(current, speed) - referencePower(truth, current, speed, 1)) < 0.02);
    const float learned = manager.predict(current, speed);
    manager.updateModel(NAN, current, speed);
    VERIFY(manager.predict(current, speed) == learned);
    current[0] = -current[0];
    const float braking = manager.predict(current, speed);
    manager.updateModel(100.0f, current, speed); // 制动样本暂不训练。
    VERIFY(manager.predict(current, speed) == braking);

    static SuperCap cap;
    cap.init({.daemon = {.tim_config = {.htim = &htim5}, .cycle = 4, .daemon_callback = nullptr, .device = nullptr}});
    VERIFY(!cap.isOnline());
    c.super_cap = &cap; manager.init(c);
    const float estimated = manager.getPower(current, speed);
    cap.receive({123.0f});
    VERIFY(cap.isOnline() && manager.getPower(current, speed) == 123.0f);
    cap.receive({NAN});
    VERIFY(manager.getPower(current, speed) == 123.0f);
    for (int i = 0; i < 3; ++i) TIM::periodElapsedCallback(&htim5); // 测试时基 2 ms，N+1 次到期。
    VERIFY(!cap.isOnline());
    VERIFY(manager.getPower(current, speed) == estimated);
    cap.receive({45.0f});
    VERIFY(cap.isOnline() && manager.getPower(current, speed) == 45.0f);
    // 实际速度环 → 功率管理 → 驱动发送链；单轮限幅与功率缩放后才回灌。
    static ChassisVelocity velocity;
    ChassisVelocity::Config vc{};
    vc.can_handle = &hcan1;
    vc.motor_id_rf = 1; vc.motor_id_lf = 2; vc.motor_id_lr = 3; vc.motor_id_rr = 4;
    vc.reduction_ratio = 19.0f;
    vc.pid_velocity.kp = 1.0f; vc.pid_velocity.ki = 0.3f;
    vc.pid_velocity.period = 0.001f;
    vc.pid_velocity.features = PID::FeatureOutputLimit | PID::FeatureIntegralLimit;
    vc.pid_velocity.integral_limit = 40.0f;
    vc.pid_velocity.output_min = -20.0f; vc.pid_velocity.output_max = 20.0f;
    vc.power = c; vc.power.super_cap = nullptr;
    velocity.init(vc);
    DJIMotor::timbaseSelect(&htim5);
    const std::array<uint8_t, 8> frame{};
    for (int i = 0; i < 4; ++i) CAN::inject(&hcan1, 0x201 + i, frame);
    float target[] = {100, 100, 100, 100};
    velocity.setTarget(target); velocity.setEnable(1); velocity.setPowerLimit(10.0f);
    velocity.tick(); TIM::periodElapsedCallback(&htim5);
    const int expected_count = int(std::sqrt((10.0 - c.idle_power_w) / (4 * c.torque_loss * 0.3 * 0.3)) * 819.2);
    for (int i = 0; i < 4; ++i) {
        const auto& tx = CAN::sent.back();
        const int count = int16_t((tx.data[2*i] << 8) | tx.data[2*i+1]);
        VERIFY(tx.id == 0x200 && std::abs(count - expected_count) <= 1);
    }
    velocity.setPowerLimit(0.0f); velocity.tick(); TIM::periodElapsedCallback(&htim5);
    for (uint8_t byte : CAN::sent.back().data) VERIFY(byte == 0);
    velocity.setEnable(0); velocity.tick(); TIM::periodElapsedCallback(&htim5);
    for (uint8_t byte : CAN::sent.back().data) VERIFY(byte == 0);
    std::printf("power: analytic allocation, one retained RLS model, feedback/Daemon passed\n");
    return power_checks;
}
