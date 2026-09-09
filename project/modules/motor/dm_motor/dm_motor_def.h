/**
 * @file    dm_motor_def.h
 * @brief   达妙电机协议常量（管理命令 / 控制帧偏移 / 参数帧 / 寄存器 RID / 故障码）
 * @note    来源：DM-J8009 手册 + 驱动控制协议（开源库 / hzu_leg 交叉核对）。
 *          管理命令帧：ID=control_id，前 7 字节 0xFF，第 8 字节为命令字。
 *          参数帧：ID 常为 0x7FF，载荷 {id_lo,id_hi,op,RID,...}（与管理命令格式不同）。
 *          量程默认值须与调试助手一致；P/V/T 可由助手改写（RID PMAX/VMAX/TMAX）。
 */
#pragma once

#include <stdint.h>

#define DM_MOTOR_MAX_INSTANCE 8

enum DMMotor_Direction : uint8_t {
    DM_DIRECTION_NORMAL = 0,
    DM_DIRECTION_REVERT,
};

/** 参数读写常用 CAN 标准帧 ID（开源库惯例；以驱动协议为准） */
#define DM_PARAM_CAN_ID (0x7FFu)

/** MIT / 反馈定点映射默认量程（驱动统一；可被 Config 覆盖） */
#define DM_P_MIN (-12.5f)
#define DM_P_MAX (12.5f)
#define DM_V_MIN (-45.0f)
#define DM_V_MAX (45.0f)
/** 位置速度模式 v_des 用很大值 ≈ 不限速（梯形匀速段上限，非 MIT 映射量程） */
#define DM_VEL_UNLIMITED (1000.0f)
#define DM_T_MIN (-18.0f)
#define DM_T_MAX (18.0f)

#define DM_KP_MIN (0.0f)
#define DM_KP_MAX (500.0f)
#define DM_KD_MIN (0.0f)
#define DM_KD_MAX (5.0f)

/**
 * 控制帧 ID 相对 control_id 的偏移（新固件）。
 * MIT：control_id + 0；位置速度：+0x100；速度：+0x200；位置力控：+0x300（本库 Mode 暂未暴露）。
 * 旧固件使能：帧 ID = ((mode-1)<<2)+control_id，命令字仍为 0xFC（一般不需要）。
 */
enum DMMotor_CtrlIdOffset : uint16_t {
    DM_CTRL_ID_MIT = 0x000,
    DM_CTRL_ID_POS_VEL = 0x100,
    DM_CTRL_ID_VEL = 0x200,
    DM_CTRL_ID_POS_FORCE = 0x300,
};

/**
 * 管理命令（帧 ID = control_id，data[0..6]=0xFF，data[7]=命令字）
 * hzu_leg / 开源库 / 手册交叉一致；无第五种「0xFF 填充」管理命令。
 */
enum DMMotor_Cmd : uint8_t {
    DM_CMD_CLEAR_ERROR = 0xFB,  ///< 清错
    DM_CMD_ENABLE = 0xFC,       ///< 使能（进入可响应控制）
    DM_CMD_DISABLE = 0xFD,      ///< 失能 / 停止
    DM_CMD_SET_ZERO = 0xFE,     ///< 将当前位置存为编码器零位
};

/**
 * 参数类帧操作码（帧 ID 多为 DM_PARAM_CAN_ID=0x7FF）
 * data[0]=control_id 低 8 位，data[1]=高 8 位，data[2]=本枚举。
 * 与管理命令不是同一套打包；运行时控制一般不依赖。
 */
enum DMMotor_ParamOp : uint8_t {
    DM_PARAM_READ = 0x33,     ///< 读：RID=data[3]；应答同 RID + 4 字节值
    DM_PARAM_WRITE = 0x55,    ///< 写：RID=data[3]，data[4..7]=值（float 或 uint32 小端）
    DM_PARAM_SAVE = 0xAA,     ///< 存 Flash：常见 data[3]=0x01
    DM_PARAM_REFRESH = 0xCC,  ///< 刷新状态（主动要一帧反馈）
};

/**
 * 电调内部寄存器 RID（调试助手 / 驱动协议；开源库 damiao::DM_REG）
 * 写 CTRL_MODE 可切换电调控制模式（1=MIT,2=位置速度,3=速度,4=位置力控）。
 * 部分 RID 为 uint32（如 MST_ID/ESC_ID/TIMEOUT/CTRL_MODE/版本/波特率），其余多为 float。
 */
enum DMMotor_Reg : uint8_t {
    DM_REG_UV_Value = 0,
    DM_REG_KT_Value = 1,
    DM_REG_OT_Value = 2,
    DM_REG_OC_Value = 3,
    DM_REG_ACC = 4,
    DM_REG_DEC = 5,
    DM_REG_MAX_SPD = 6,
    DM_REG_MST_ID = 7,
    DM_REG_ESC_ID = 8,
    DM_REG_TIMEOUT = 9,
    DM_REG_CTRL_MODE = 10,
    DM_REG_Damp = 11,
    DM_REG_Inertia = 12,
    DM_REG_hw_ver = 13,
    DM_REG_sw_ver = 14,
    DM_REG_SN = 15,
    DM_REG_NPP = 16,
    DM_REG_Rs = 17,
    DM_REG_LS = 18,
    DM_REG_Flux = 19,
    DM_REG_Gr = 20,
    DM_REG_PMAX = 21,
    DM_REG_VMAX = 22,
    DM_REG_TMAX = 23,
    DM_REG_I_BW = 24,
    DM_REG_KP_ASR = 25,
    DM_REG_KI_ASR = 26,
    DM_REG_KP_APR = 27,
    DM_REG_KI_APR = 28,
    DM_REG_OV_Value = 29,
    DM_REG_GREF = 30,
    DM_REG_Deta = 31,
    DM_REG_V_BW = 32,
    DM_REG_IQ_c1 = 33,
    DM_REG_VL_c1 = 34,
    DM_REG_can_br = 35,
    DM_REG_sub_ver = 36,
    DM_REG_u_off = 50,
    DM_REG_v_off = 51,
    DM_REG_k1 = 52,
    DM_REG_k2 = 53,
    DM_REG_m_off = 54,
    DM_REG_dir = 55,
    DM_REG_p_m = 80,
    DM_REG_xout = 81,
};

/** CTRL_MODE（RID=10）取值，与助手 ControlMode 一致 */
enum DMMotor_EscCtrlMode : uint8_t {
    DM_ESC_MIT = 1,
    DM_ESC_POS_VEL = 2,
    DM_ESC_VEL = 3,
    DM_ESC_POS_FORCE = 4,
};
