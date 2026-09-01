# msg_bus 解耦（cmd 只发指令，chassis/gimbal 应用层）

## 背景 / 目标

现状耦合是"改着改着"带出来的，不是原设计：cmd 越界直接写 `chassis_inst->v/theta`、算功率并调 `Chassis_SetPowerLimit`、在 cmd 里管 gimbal 自瞄/手动状态机、桥接 `chassis_cmd.yaw_motor_angle`。

目标：恢复"cmd 发指令驱动、chassis/gimbal/shoot 各自消费并拥有逻辑"；数据（裁判、子系统状态）经消息总线广播，谁订阅谁读，cmd 不中转。

## msg_bus 模块（通用、动态，不写死枚举）

位置 `project/modules/msg_bus/`。bus 不感知任何业务类型/topic；topic 运行时按名字 get-or-create，内部 `malloc` 链表（项目已用 malloc）。

```c
typedef struct MsgBus_Topic MsgBus_Topic;

MsgBus_Topic *MsgBus_TopicGet(const char *name, size_t msg_size);          // 同名 get-or-create
void          MsgBus_Publish(MsgBus_Topic *t, const void *data, size_t size); // 覆盖写 + version++
const void   *MsgBus_Read(MsgBus_Topic *t);          // 订阅读最新
uint32_t      MsgBus_GetVersion(MsgBus_Topic *t);    // 判断是否有新数据
```

- 拉模型（最新值 + 版本号）：各模块在自己 RTOS task 轮询读，不用回调 push（避免回调跑在发布者上下文）。
- topic 名 + 消息类型由**发布者在自己头文件定义**（如 `referee.h`: `#define REFEREE_TOPIC "referee"` + `referee_info_t`），订阅者 include 发布者头拿名和类型；加 topic 不改 bus 代码。

## 拓扑

| topic | 消息类型 | 发布者 | 订阅者 |
|------|---------|-------|-------|
| chassis_cmd | `Chassis_Cmd` | cmd | chassis |
| gimbal_cmd | `Gimbal_Cmd` | cmd | gimbal |
| shoot_cmd | `Shoot_Cmd` | cmd | shoot |
| referee | `referee_info_t` | referee 模块 | chassis(功率)、cmd(RGB/debug)、shoot(热量，后续) |
| gimbal_state | `Gimbal_State` | gimbal | cmd(算 θ)、chassis(follow) |
| chassis_state | `Chassis_State` | chassis | cmd(回传/调试) |

## 各模块改动

### cmd（纯指令转换）
- 读遥控/键鼠/minipc → 发布 chassis_cmd / gimbal_cmd / shoot_cmd。
- θ 场心系换算：订阅 gimbal_state 拿 yaw。
- RGB + referee debug：订阅 referee。
- 删掉：`chassis_inst->v/theta` 直写、`Chassis_SetMode`/`Chassis_SetPowerLimit` 调用、`chassis_cmd.yaw_motor_angle` 桥接、gimbal 的 5 个直调 + 自瞄状态机。

### chassis（应用层）
- 订阅 chassis_cmd + referee + gimbal_state，task 里：
  - 使能 + 写 `chassis_inst->v/theta`；
  - 模式边沿检测（切 NO_ROTATION 清 w_rot）；
  - 功率二值化（读 referee）→ `Chassis_SetPowerLimit`；
  - follow 读 gimbal_state.yaw。
- 发布 chassis_state（回传）。

### gimbal（应用层）
- 订阅 gimbal_cmd，task 里自瞄/手动状态机（统一 remote/mouse 的 last_aim）。
- 发布 gimbal_state（yaw）。

### shoot（应用层）
- 订阅 shoot_cmd（enable + fire_count）；`Shoot_Fire` 改为 cmd 发布 fire_count。

## 分步实施（每步保持可编译）

1. 新建 `msg_bus` 模块 + 编译。
2. referee 模块发布 referee topic（Init 建 topic，ParseFrame 末尾 Publish）。
3. chassis 切订阅（chassis_cmd/referee/gimbal_state），搬功率/运动逻辑进 task。
4. gimbal 切订阅（gimbal_cmd），状态机搬入，发布 gimbal_state。
5. shoot 切订阅（shoot_cmd）。
6. cmd 收尾成纯转换，删越界代码。

## 备注
- 命令结构体类型仍定义在各自模块头（chassis.h/gimbal.h/shoot.h），作为消息 payload。
- referee 无独立 task，数据在 UART RX 回调更新；topic 在 `Referee_Init` 建好，`ParseFrame` 里只 memcpy 发布（ISR 安全）。
