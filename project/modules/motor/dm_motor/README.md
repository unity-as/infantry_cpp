# 达妙电机

通过本项目 `bsp_can` 经典 CAN 收发，用户量单位为 rad、rad/s、N·m。本项目当前应用层未实例化达妙电机；驱动已编译及主机验证，实机未验证。

## 接口与反馈

`Config.mode` 设置初始模式；对应接口为 `Current → setCurrent`、`Mit → setMit`、`Position → setAngle`、`Velocity → setVelocity`。`init()` 写电调 CTRL_MODE，并忙等 2 ms，须在 CAN/TIM 就绪后初始化。CTRL_MODE 临时生效，不保存 Flash。

反馈 D[0] 高四位由原 `err_` 改为 `state_`：0 失能、1 工作中；识别、读取异常及故障定义见 [dm_motor.h](dm_motor.h)。`running()` 判断工作状态，`faulted()` 判断异常，`shouldAutoClear()` 仅标记原有高位故障范围，不自行清错。未知状态原样保留。

`angle_raw_` 是协议位置经额外传动比换算后、方向与零偏变换前的角度；`angle_` 为连续多圈用户角。电调反馈已经是内置减速后的输出轴量，`Config.reduction_ratio` 只填写额外机械传动比，无额外传动时用 1。

解码与 DJI 一样直接在 `decodeCallback()` 中完成：解析原始字段、换算用户速度/力矩、修正位置差后累计用户角、更新 `ecd_last_`、喂狗并置通信有效。达妙位置码是完整 16 位，差值显式转换为 `int16_t`，利用 C++20 整数转换的模 65536 规则得到最短增量，无须再手动补偿；增量按配置的位置映射范围换算为 rad，再处理额外传动比和方向。首帧直接以协议位置经方向、零偏变换后的值建立起点，不把首帧误判为跨界，也不每圈按 2π 取模。回绕模型参考 [HXC 达妙驱动的接收回调](https://github.com/CQUPTHXC/DMMotorCtrl/blob/main/include/DMCtrlESP.hpp#L403)，不是本型号固件回绕行为的实测证明。

使用连续角须同时检查 `motor_valid_` 和 `angle_valid_`。前提是相邻反馈间实际位移小于半个报文位置周期；恰好半量程时方向不唯一，标记角度无效。通信超时或置零命令发送成功后同样清 `angle_valid_`；无效期间冻结 `angle_`，协议角、速度、力矩仍随有效帧更新，通信恢复不会自动恢复圈数。调用 `resetAngleTracking()` 丢弃旧圈数，下一匹配反馈重新以协议角建立起点；上层须在确认静止/基准后调用。电调置零须等其执行完成、旧反馈排空后再重建；发送成功不等于电调确认。置零发送失败保留已有跟踪。

反馈内电机 ID 不匹配时不更新、不喂狗，也不更新展开历史。通信超时及配置单位见 [Daemon](../../daemon/README.md#调用方)；上层仍须据有效状态决定使能。不能恢复断电或失联期间的历史圈数，也不能单靠短间隔反馈识别所有电调重启；已知重启后应重新建立基准。与 DJI 一样使用 float 用户角逐帧累计，长时间单向旋转的浮点精度边界尚未实测。

## 异步模式切换

```cpp
motor.requestMode(DMMotor::Mode::Position);
// 后续周期查询，不阻塞当前回调。
if (!motor.modeSwitching() && motor.mode() == DMMotor::Mode::Position)
    motor.setAngle(target, speed_limit);
```

定时回调依次失能、写 CTRL_MODE、等待 2 个心跳，并按切换前状态决定是否重新使能；期间不发送控制帧。发送失败停留当前阶段，下拍重试。切换中再次请求会更新待切模式或排队，当前切完继续处理。

- `mode()` 返回当前本地模式；`modeSwitchStatus()` 返回阶段；`Ready` 只表示本地发送时序完成，不代表已收到电调写入确认。
- 新模式生效时目标重置为当前协议位置经方向、零偏变换后的值、零速度、零力矩；完成后再写新目标，不把连续多圈角直接送入电调位置字段。
- 切换中的 `setEnable()` 只更新完成后是否恢复使能。
- `setAngle(angle)` 使用不限速配置；双参数形式采用 `abs(velocity)`，速度上限不随电机方向变负。
- `setAngle()` / `setMit()` 的位置目标仍采用电调位置坐标经方向、零偏变换后的值；不是无限多圈定位接口。主控多圈闭环可读取连续 `angle_`，通过速度或力矩接口输出；MIT 位置仍受映射量程限幅，不自动对目标取模。

## 失能、量程与发送

失能心跳持续发送 DISABLE 管理帧，不再发送零力矩控制帧；重复 `setEnable(0)` 不立即重发，下一心跳仍发送 DISABLE。使能发送 ENABLE 后转发控制帧。所有发送等待上限采用源版 0.01 ms（本项目 `CAN::transmit()` 单位为毫秒）。

默认 MIT/反馈量程的唯一值定义在 [dm_motor_def.h](dm_motor_def.h)：本轮同步源项目的 DM-J8009 力矩映射。其他型号或助手改过 PMAX/VMAX/TMAX 时，必须通过实例 `Config.p_min/max`、`v_min/max`、`t_min/max` 匹配电调内部量程；本次不修改电调参数或认定本板连接了该型号。位置速度/速度帧使用小端 float。

主机验证入口与范围见 [模块测试](../../../../tools/tests/leg_sync/README.md)。
