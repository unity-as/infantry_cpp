#include "dji_motor.h"

#include "daemon.h"
#include "bsp_tim.h"

#include <stdlib.h>
#include <string.h>

static TIM_Instance *djim_tim=NULL;
static DJIMotor_Instance *djim_instance[DJIM_MAX_INSTANCE]={0};
static uint8_t motor_idx=0;

static CAN_Instance *djim_tx_group[DJIM_MAX_GROUP] ={0};
static uint8_t group_idx=0;

static void DecodeDJIMotor(void* device)
{
    DJIMotor_Instance *instance = (DJIMotor_Instance *)device;

    //解码CAN数据，大端模式
    uint8_t *CAN_ReceiveData = instance->djim_can->rx_buff;
    instance->feedback_raw.ecd = (uint16_t)(CAN_ReceiveData[0] << 8 | CAN_ReceiveData[1]);
    instance->feedback_raw.rpm = (int16_t)(CAN_ReceiveData[2] << 8 | CAN_ReceiveData[3]);
    instance->feedback_raw.curr = (int16_t)(CAN_ReceiveData[4] << 8 | CAN_ReceiveData[5]);
    instance->feedback_raw.temp = CAN_ReceiveData[6];

    //速度 → 用户坐标系
    float velocity_user = DJIM_VELOCITY(instance->feedback_raw.rpm) / instance->reduction_ratio;
    //电流 → 用户坐标系（电流不随减速比变化）
    float current_user = DJIM_CURRENT(instance->motor_type, instance->feedback_raw.curr);

    if (instance->direction == DJIM_DIRECTION_REVERT) {
        velocity_user = -velocity_user;
        current_user = -current_user;
    }

    //累计角度 → 用户坐标系
    int16_t error = instance->feedback_raw.ecd - instance->ecd_last;//角度处理，最短路径
    //角度最短路径处理，防止角度突变超过一半周期
    if(error > DJIM_ENCODER_LINES / 2)
        error -= DJIM_ENCODER_LINES;
    else if(error < -DJIM_ENCODER_LINES / 2)
        error += DJIM_ENCODER_LINES;

    float angle_delta = DJIM_ANGLE(error) / instance->reduction_ratio;
    if (instance->direction == DJIM_DIRECTION_REVERT)
        angle_delta = -angle_delta;

    instance->angle += angle_delta;

    instance->velocity_raw = velocity_user;   // 原始值（未滤波）
    instance->velocity += DJIM_VELOCITY_LPF_ALPHA * (velocity_user - instance->velocity);   // 一阶低通
    instance->current = current_user;
    instance->temperature = instance->feedback_raw.temp;

    //记录这次的位置给下次使用
    instance->ecd_last = instance->feedback_raw.ecd;

    Daemon_Reset(instance->daemon_lose);//重置失联守护计时器
    instance->motor_valid = 1;//收到反馈帧，标记电机通信有效
}

static void DJIM_Current_Command(DJIMotor_Instance *instance)
{
    int16_t current_cnt = DJIM_CURRENT_TO_CNT(instance->motor_type, instance->target_current);

    //用户坐标系 → 电机坐标系
    if (instance->direction == DJIM_DIRECTION_REVERT)
        current_cnt = -current_cnt;

    if(current_cnt > djim_current_cnt_range[instance->motor_type])
        current_cnt = djim_current_cnt_range[instance->motor_type];//限流保护
    else if(current_cnt < -djim_current_cnt_range[instance->motor_type])
        current_cnt = -djim_current_cnt_range[instance->motor_type];

    //发送电流指令，大端模式
    instance->command_ptr[0] = (uint8_t)(current_cnt >> 8);
    instance->command_ptr[1] = (uint8_t)(current_cnt & 0xFF);
}

//获取电机发送的CAN实例,如果没有就创建一个新的CAN实例
static CAN_Instance* DJIMotor_Get_Group(DJIMotor_Instance *instance)
{
    for(uint8_t i=0;i<group_idx;i++)
        if(djim_tx_group[i]->can_handle == instance->djim_can->can_handle &&
            djim_tx_group[i]->txconf.StdId == DJIM_TX_ID(DJIM_TX_GROUP(instance->motor_type, instance->motor_id)))
                return djim_tx_group[i];
    return NULL;
}

static void DJIMotor_Update(DJIMotor_Instance *instance)
{
    if(!instance->motor_enable || !instance->motor_valid)
    {
        instance->target_current = 0;
        DJIM_Current_Command(instance);
        return;
    }

    if(instance->pid_mode == FOC_POSITION_LOOP_CONTROL)//位置环，分频更新，更新频率=1000Hz/pos_freq_div
    {
        if(++instance->div_cnt == instance->pos_freq_div)
        {
            instance->div_cnt = 0;

            PID_Set_Setpoint(instance->pid_angle, instance->target_angle); //设定值前馈，角度目标
            PID_Update(instance->pid_angle,instance->angle);//角度控制
            instance->target_velocity = instance->pid_angle->output;
        }
    }

    if(instance->pid_mode != FOC_CURRENT_LOOP_CONTROL)//速度环，只要不处于开环控制状态（指非纯电流控制时），速度环都要更新 1000Hz
    {
        PID_Set_Setpoint(instance->pid_velocity, instance->target_velocity + instance->velocity_feedforward); //设定值前馈，速度前馈
        PID_Set_Feedforward(instance->pid_velocity, instance->current_feedforward); //输出值前馈，力矩前馈

        PID_Update(instance->pid_velocity, instance->velocity); 
        instance->target_current = instance->pid_velocity->output;
    }

    DJIM_Current_Command(instance);
}

void DJIMotor_Set_Enable(DJIMotor_Instance *instance, uint8_t motor_enable)//电机使能
{
    if (!instance) return;
    motor_enable = motor_enable!=0;
    instance->motor_enable=motor_enable;
}

static void DJIMotor_Lose(void* device)
{
    DJIMotor_Instance *instance = (DJIMotor_Instance *)device;
    instance->motor_valid = 0;//失联，标记通信无效（Update 会输出 0 电流）

    //用于debug检查失联电机的can通道、型号和ID，确认是哪个电机失联了。
    CAN_HandleTypeDef *channel = instance->djim_can->can_handle;
    DJIMotor_Type type = instance->motor_type;
    uint8_t id = instance->motor_id;
}

static void DJIMotor_TimHandler(void* device)
{
    for(uint8_t i=0;i<motor_idx;i++)
        DJIMotor_Update(djim_instance[i]);//定时器中断处理函数，更新所有注册的电机实例的PID

    for(uint8_t i=0;i<group_idx;i++)
        CANTransmit(djim_tx_group[i], 1.0f);
}

void DJIMotor_Set_Angle(DJIMotor_Instance *instance, float angle)//位置（角度）控制，用户坐标系
{
    if (!instance) return;
    instance->pid_mode=FOC_POSITION_LOOP_CONTROL;
    while(angle - instance->angle > 180.0f)
        angle -= 360.0f;
    while(angle - instance->angle < -180.0f)
        angle += 360.0f;
    instance->target_angle = angle;
}

void DJIMotor_Set_Angle_Circular(DJIMotor_Instance *instance, float angle)//位置（角度）控制，用户坐标系
{
    if (!instance) return;
    instance->pid_mode=FOC_POSITION_LOOP_CONTROL;
    instance->target_angle = angle;
}

void DJIMotor_Set_Angle_Increment(DJIMotor_Instance *instance, float angle_increment)//增量角度控制
{
    if (!instance) return;
    instance->pid_mode=FOC_POSITION_LOOP_CONTROL;
    instance->target_angle = instance->angle + angle_increment;
}

void DJIMotor_Set_Velocity(DJIMotor_Instance *instance, float velocity)//速度控制
{
    if (!instance) return;
    instance->pid_mode=FOC_VELOCITY_LOOP_CONTROL;
    instance->target_velocity = velocity;
}

void DJIMotor_Set_Current(DJIMotor_Instance *instance, float current)//开环控制
{
    if (!instance) return;
    instance->pid_mode=FOC_CURRENT_LOOP_CONTROL;
    instance->target_current = current;
}

void DJIMotor_Set_VelocityFF(DJIMotor_Instance *instance, float velocity)//速度前馈，用于补偿电机惯性和负载惯性的影响
{
    if (!instance) return;
    instance->velocity_feedforward = velocity;
}

void DJIMotor_Set_CurrentFF(DJIMotor_Instance *instance, float current)//力矩前馈，用于补偿负载惯性的影响
{
    if(!instance) return;
    instance->current_feedforward = current;
}

//自己配一个1000Hz的定时器，用于更新PID。硬实时可靠性比RTOS软件定时器更高，且不受其他代码的影响。DJI电机控制对实时性要求较高，尤其是位置环，建议使用定时器中断来更新PID。
void DJIMotor_TimbaseSelect(TIM_HandleTypeDef *htim)
{
    TIM_Init_Config_s tim_config =
    {
        .htim = htim,
        .tim_callback = DJIMotor_TimHandler,
        .device = NULL
    };

    djim_tim = TIM_Register(&tim_config);
}

DJIMotor_Instance *DJIMotor_Register(DJIMotor_Init_Config_s *config)
{
    if(motor_idx >= DJIM_MAX_INSTANCE)
        return NULL;

    if(config->motor_id >8 || config->motor_id == 0)
        return NULL;//能进这里家里得请高人了

    DJIMotor_Instance *instance = (DJIMotor_Instance *)malloc(sizeof(DJIMotor_Instance));
    memset(instance, 0, sizeof(DJIMotor_Instance));

    Daemon_Init_Config_s daemon_config =
    {
        .cycle = 100,//守护进程 ms
        .daemon_callback = DJIMotor_Lose,
        .device = instance
    };

    CAN_Init_Config_s can_config =
    {
        .can_handle = config->can_handle,
        .rx_id = DJIM_RX_ID(config->motor_type, config->motor_id),
        .can_module_callback = DecodeDJIMotor,
        .device = instance
    };

    instance->motor_type = config->motor_type;
    instance->motor_id = config->motor_id;
    instance->direction = config->direction;
    instance->reduction_ratio = config->reduction_ratio > 0.0f ? config->reduction_ratio : 1.0f;
    instance->angle_offset = -config->initial_angle;//取反：config 传入初始角度，offset 用于补偿

    float angle_init = config->initial_angle;
    while(angle_init < 0)
        angle_init += 360.0f;
    while(angle_init >= 360.0f)
        angle_init -= 360.0f;
    instance->ecd_last = angle_init * instance->reduction_ratio * DJIM_ENCODER_LINES / 360.0f;//初始角度对应的编码器值，用于补偿初始角度

    instance->pos_freq_div = config->pos_freq_div == 0 ? 1 : config->pos_freq_div;//分频不能为0，不能更新了,默认设为3
    if(instance->pos_freq_div > 20)//分频过大可能导致位置环响应过慢，默认最大设为20
        instance->pos_freq_div = 20;
    instance->div_cnt = 0;

    instance->pid_angle = PID_Init(&config->pid_angle);
    instance->pid_velocity = PID_Init(&config->pid_velocity);
    PID_Clear_Features(instance->pid_angle, PID_FEATURE_FEEDFORWARD);
    //位置环不启用输出量前馈，位置环更新频率低，为保证速度前馈更新频率，自行实现速度环设定值前馈
    PID_Set_Features(instance->pid_velocity, PID_FEATURE_FEEDFORWARD);
    instance->pid_velocity->feedforward_gain = 1.0f; //力矩前馈增益，默认1.0f

    instance->djim_can = CANRegister(&can_config);

    instance->daemon_lose = Daemon_Register(&daemon_config);

    CAN_Instance *djim_tx_instance = DJIMotor_Get_Group(instance);
    if(djim_tx_instance == NULL)
    {
        if(group_idx >= DJIM_MAX_GROUP)//目前设计最多支持DJIM_MAX_GROUP组发送实例，每组4个电机，共DJIM_MAX_GROUP*4个电机，已经远远超过实际需求了
            return NULL;

        CAN_Init_Config_s can_tx_config =
        {
            .can_handle = config->can_handle,
            .rx_id = 0,//发送不需要设置接收id
            .can_module_callback = NULL,//发送不需要回调函数
            .device = NULL
        };
        djim_tx_instance = CANRegister(&can_tx_config);
        djim_tx_instance -> txconf.StdId = DJIM_TX_ID(DJIM_TX_GROUP(config->motor_type, config->motor_id));

        djim_tx_group[group_idx++] = djim_tx_instance;
    }
    instance->command_ptr = djim_tx_instance->tx_buff + 2 * (((instance->motor_id)-1)%4);//每4个电机共用一个发送实例，每个电机占2个字节的发送数据
    //发送的buff长度为8字节，每两个uint8_t是一组电机的数据。初始化确定ptr后一直用即可。

    djim_instance[motor_idx++] = instance;
    return instance;
}