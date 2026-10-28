# PID 模块

通用位置式／增量式 PID。类、配置字段及功能枚举以 [pid.h](pid.h) 为准，
成员实现见 [pid.cpp](pid.cpp)。仅依赖 C 数学与整数类型头，无外设、HAL、RTOS 或堆分配依赖。
对象生命周期遵循 [API 约定](../../../../.claude/conventions/api.md)。

## 使用方式

```cpp
#include "pid.h"

static PID controller;

void initializeController() {
    PID::Config config{};
    config.kp = 4.0f;
    config.kd = 0.00001f;
    config.mode = PID::Mode::Position;
    config.features = PID::FeatureOutputLimit | PID::FeatureDerivativeOnMeasurement;
    config.output_min = -360.0f;
    config.output_max = 360.0f;
    config.period = 0.01f;
    controller.init(config);
    controller.reset();
}

// 每 10 ms 调用一次；返回值交给应用，由应用负责执行器单位与限幅。
float stepController(float target, float feedback) {
    controller.setSetpoint(target);
    controller.update(feedback);
    return controller.output_;
}
```

`init()` 写入配置，不清除已有运行历史；重新初始化时需要清理历史的，由调用者显式 `reset()`。
`reset()` 清除积分、误差／反馈历史和输出，但保留目标、配置及前馈当前值／历史值。
增量式启用前馈时，复位后的首拍仍会使用此前的前馈历史；它不是完整的冷启动复位。

## 参数与时间口径

- `period` 为 `float` 秒，支持亚毫秒周期和大于 255 ms 的周期；应与实际调用周期一致。
  非正数或 NaN／Inf 按 `0.001f`（1 ms）处理。积分乘以秒数，微分除以秒数，`ki/kd` 按秒整定。
  从旧毫秒配置等价迁移时：`period`、`kd`、`kd_extra`、`integral_limit` 除以 1000，
  `ki`、`derivative_limit` 乘以 1000，比例系数及输出限幅不变；已有积分状态也须除以 1000，或显式 `reset()`。
  `setIntegral()`／`integral_` 使用误差单位·s，微分限幅使用误差单位/s。
- `feedforward_gain == 0` 按 1.0 处理；需要停用前馈时清除 `FeatureFeedforward`。
- `filter_alpha <= 0` 按 0.3 处理，`> 1` 钳为 1；滤波仅在 `FeatureFilter` 开启时作用于反馈。
- 除周期的默认处理外，`init()` 不检查其他参数是否合法，也不拒绝其 NaN／Inf；没有初始化成功或输入有效状态码。
  配置、输入的有限性与限幅上下界由调用者保证。

## 模式与功能边界

位置式输出为本拍的绝对控制量；`integral_` 为跨拍累积误差。
增量式将本拍增量累加到 `output_`，`integral_` 每拍覆盖为 `error * period`，不是积分历史。
两种模式的参数与历史含义不同，切换配置不自动实现无扰交接。

可选功能包括积分限幅、微分限幅、输出限幅、死区、反馈滤波、变增益、微分先行、前馈和梯形积分。
积分限幅与梯形积分只作用于位置式；增量式启用输出限幅时，对累积输出限幅。
微分先行在位置式使用反馈一阶差分，在增量式使用反馈二阶差分。
位置式前馈叠加当前值，增量式叠加前后两拍前馈的差值。

## 抗饱和与外部限幅

位置式启用 `FeatureOutputLimit` 时，按上一拍 `output_` 判断是否冻结继续推动饱和的积分。
若应用在 PID 之后再次限幅，可在取得实际限幅结果后调用 `backCalcAntiWindup(limited_output)`。

该接口仅在新输出绝对值小于 PID 当前输出绝对值时回灌：位置式在 `ki != 0` 时调整积分，
两种模式都会更新 `output_`。回灌不会再次执行积分限幅；调用者须保证参数单位一致、数值有效，
且传入值对应本次执行的限幅结果。它不自动获取功率控制或执行器的实际输出。

## 本项目移植

实现照搬 `leg_main` 提交 `12b4395`。现有底盘、云台、自瞄、射击和温控配置按上述规则换算，
只改变时间单位，保留旧配置的逐拍等效作用；省略周期的旧配置按原默认 1 ms 显式写为 `0.001f`。
这不表示重新标定了实际调度周期：云台位置环仍按原分频运行，温控仍约每 750 ms 更新。
底盘功率回灌还会直接修改积分，其单位换算见 [底盘说明](../../../application/chassis/README.md)。
