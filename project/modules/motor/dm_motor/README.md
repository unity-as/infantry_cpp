# 达妙电机

通过本项目 `bsp_can` 经典 CAN 收发，用户量单位为 rad、rad/s、N·m。本项目当前应用层未实例化达妙电机；驱动已编译及主机验证，实机未验证。

## 接口与反馈

`Config.mode` 设置初始模式；对应接口为 `Current → setCurrent`、`Mit → setMit`、`Position → setAngle`、`Velocity → setVelocity`。`init()` 写电调 CTRL_MODE，并忙等 2 ms，须在 CAN/TIM 就绪后初始化。CTRL_MODE 临时生效，不保存 Flash。

反馈 D[0] 高四位由原 `err_` 改为 `state_`：0 失能、1 工作中；识别、读取异常及故障定义见 [dm_motor.h](dm_motor.h)。`running()` 判断工作状态，`faulted()` 判断异常，`shouldAutoClear()` 仅标记原有高位故障范围，不自行清错。未知状态原样保留。

`angle_raw_` 是减速后、方向与零偏变换前的角度；`angle_` 为用户角度。反馈内电机 ID 不匹配时不更新、不喂狗。通信超时及配置单位见 [Daemon](../../daemon/README.md#调用方)；离线仅清 `motor_valid_`，上层仍须据此决定使能。

## 异步模式切换

```cpp
motor.requestMode(DMMotor::Mode::Position);
// 后续周期查询，不阻塞当前回调。
if (!motor.modeSwitching() && motor.mode() == DMMotor::Mode::Position)
    motor.setAngle(target, speed_limit);
```

定时回调依次失能、写 CTRL_MODE、等待 2 个心跳，并按切换前状态决定是否重新使能；期间不发送控制帧。发送失败停留当前阶段，下拍重试。切换中再次请求会更新待切模式或排队，当前切完继续处理。

- `mode()` 返回当前本地模式；`modeSwitchStatus()` 返回阶段；`Ready` 只表示本地发送时序完成，不代表已收到电调写入确认。
- 新模式生效时目标重置为当前位置、零速度、零力矩；完成后再写新目标。
- 切换中的 `setEnable()` 只更新完成后是否恢复使能。
- `setAngle(angle)` 使用不限速配置；双参数形式采用 `abs(velocity)`，速度上限不随电机方向变负。

## 失能、量程与发送

失能心跳持续发送 DISABLE 管理帧，不再发送零力矩控制帧；重复 `setEnable(0)` 不立即重发，下一心跳仍发送 DISABLE。使能发送 ENABLE 后转发控制帧。所有发送等待上限采用源版 0.01 ms（本项目 `CAN::transmit()` 单位为毫秒）。

默认 MIT/反馈量程的唯一值定义在 [dm_motor_def.h](dm_motor_def.h)：本轮同步源项目的 DM-J8009 力矩映射。其他型号或助手改过 PMAX/VMAX/TMAX 时，必须通过实例 `Config.p_min/max`、`v_min/max`、`t_min/max` 匹配电调内部量程；本次不修改电调参数或认定本板连接了该型号。位置速度/速度帧使用小端 float。

主机验证入口与范围见 [模块测试](../../../../tools/tests/leg_sync/README.md)。
