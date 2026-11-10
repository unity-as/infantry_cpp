# RLS 参数估计

通用线性参数估计 `y = phi^T p`，接口与数组布局见 [rls.hpp](rls.hpp)。移植来源为 `leg_main` 的 `codex/rls` 分支提交 `e4b09f5e8102dadf89f7a079b4828839cd530874`，保留估计公式和失败处理语义；本项目按纯模板约定合并为 `.hpp`，按 [C++ 规范](../../../../.claude/docs/cpp/cpp_conventions.md#实例类成员排列)排列声明。模块无堆分配，不依赖机器人、HAL、RTOS 或矩阵库。

## 使用与状态

`RLS<参数数>` 在编译期确定实例大小。参数、协方差及更新暂存区归对象持有；长期对象使用全局或静态存储，同一对象不支持并发调用。

```cpp
#include "rls.hpp"

static RLS<2> estimator;

bool initEstimator() {
    static const float initial[] = {0.2f, 1.0f};
    static const float covariance[] = {1.0f, 1.0f};
    return estimator.init({
        .forgetting_factor = 1.0f,
        .initial_parameters = initial,
        .initial_covariance_diag = covariance,
    });
}

bool consumeSample(const float phi[2], float observation) {
    return estimator.update(phi, &observation, 1);
}
```

示例参数仅说明接口，不是本车功率模型标定值。维数必须为正且能由 `uint16_t` 表示，还须满足目标平台的存储预算。`parameterCount()` 初始化前也可用；未初始化时 `parameters()` 返回空指针。

`init()` 检查有限值、`0 < forgetting_factor <= 1` 和正对角先验。初始参数是先验中心，对角协方差的倒数是先验权重，不是物理参数边界。`reset()` 保留维数和遗忘因子，按调用者提供的参数及对角协方差重置；空指针分别表示零参数和单位协方差，不恢复历史默认值。

每次 `update()` 表示同一拍的一组标量方程：先对旧信息遗忘一次，再以单位观测权重吸收各行，整组成功后提交。等价目标为“遗忘因子 × 上一拍二次目标 + 本拍各行残差平方和”。非法配置、非有限输入或数值失败返回 `false`，保留原参数与协方差。`predict()` 根据给定回归量计算预测，无效输入或结果返回 NaN。

## 数值与应用边界

状态使用 float，内积及更新中间量使用 double。零回归行不提供新信息，但仍执行本拍遗忘；缺少有效数据时调用者应停止更新。持续缺少激励时，指数遗忘可能使协方差膨胀；模块拒绝数值溢出，不自动重置。

来源版本已知的低激励、相关回归量下 float 协方差退化及静默停滞风险仍然保留；返回成功不表示参数已收敛、物理有效或系统可辨识。调用者负责回归量与观测同步、尺度归一化、激励及样本门控、物理约束和结果采用。每拍更新按维数和方程数执行直接计算，不包含等待收敛的内层求解循环；实际 Cortex-M4 执行耗时尚未测量。

当前只提供通用库，没有在底盘控制中创建估计器或接入在线辨识。底盘功率模型、参考实现错误与输入缺口见 [RLS 功率模型移植核对](../../../../.claude/plans/feedforward.md#rls-功率模型移植核对2026-11-10)，采样事实以该记录链接的硬件文档为准。

## 验证入口

从仓库根目录执行，需支持 C++20 的主机 g++：

```powershell
New-Item -ItemType Directory -Force build/rls-tests | Out-Null
g++ -std=c++20 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti -I project/modules/alg/rls tools/tests/rls_test.cpp -o build/rls-tests/rls_test.exe
./build/rls-tests/rls_test.exe
```

[测试源码](../../../../tools/tests/rls_test.cpp)同样来自上述提交，仅适配头文件名。测试以独立 double 信息矩阵的批量解对照参数及协方差，覆盖单/多方程遗忘、收敛与参数变化跟踪、预测、非法配置与输入、整组失败回滚、重置、不同维数、溢出保护及连续 20000 拍更新。模板编译与本项目验证结果统一见 [移植验证记录](../../../../.claude/records/validation.md#rls-通用库移植2026-11-10)。
