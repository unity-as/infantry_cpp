/**
 * @file bsp_soft_i2c.c
 * @author FuckingMaker
 * @brief 阻塞式基础功能IIC软件模拟驱动
 * @version 1.0
 * @date 2026-01-22
 * @details
 *              该文件采用8位的设备地址，固定时钟节拍实现软件I2C协议的基本功能。
 *              包括初始化I2C实例、起始/停止信号、读写位和字节、以及读写数据等功能。
 *              通过控制GPIO引脚模拟I2C总线的时序，实现与I2C设备的通信。
 * @note 该实现为阻塞式且无附加功能，仅适用于对实时性要求不高的应用场景。
 * @attention 该实现不包含错误处理和超时检测，请根据实际应用场景使用。
 *              确保GPIO引脚已正确配置为开漏输出模式，I2C节拍正确设置，如：100KHz时，设置为5（好像只能设置为5）。
 *
 *              总之这是一个基础的I2C软件模拟驱动，接下来会以此为基础进行优化和改良。
 */

#include "bsp_soft_i2c.h"

#include "bsp_dwt.h"
#include <stdlib.h>
#include <string.h>

#define SOFT_I2C_MAX_INSTANCES 10

static SOFT_I2C_Instance *soft_i2c[SOFT_I2C_MAX_INSTANCES];
static uint8_t idx;

/**
 * @brief 初始化软件I2C实例
 * 
 * 该函数分配内存并初始化一个软件模拟I2C实例，设置相关的GPIO端口和引脚，
 * 并将SCL和SDA线设置为高电平（空闲状态）。
 * 
 * @param config 指向SOFT_I2C_Config结构体的指针，包含I2C配置信息
 *               包括GPIO端口、SCL和SDA引脚、时钟节拍和设备地址等
 * @return 返回指向新创建的SOFT_I2C_Instance结构体的指针
 *         如果内存分配失败则返回NULL
 */
SOFT_I2C_Instance *SOFT_I2C_Init(SOFT_I2C_Config *config)
{
    if (idx >= SOFT_I2C_MAX_INSTANCES)
        return NULL;

    // 为一个新的SOFT_I2C实例分配内存并初始化
    SOFT_I2C_Instance *instance=(SOFT_I2C_Instance*)malloc(sizeof(SOFT_I2C_Instance));
    memset(instance, 0, sizeof(SOFT_I2C_Instance));
    instance->port = config->port;
    instance->scl = config->scl;
    instance->sda = config->sda;
    instance->i2c_tick = config->i2c_tick;
    instance->dev_addr = config->dev_addr;

    // 将SCL和SDA线设置为高电平（空闲状态）
    HAL_GPIO_WritePin(instance->port,instance->scl,1);
    HAL_GPIO_WritePin(instance->port,instance->sda,1);

    soft_i2c[idx++]=instance;
    // 返回初始化好的软件I2C实例指针
    return instance;
}
/**
 * @brief 发送软件I2C起始信号
 * 
 * 该函数通过控制SDA和SCL线产生I2C通信的起始条件。
 * 拉高SCL和SDA线，然后拉低SDA线，再拉低SCL线，完成起始信号发送。
 * 
 * @param instance 软件I2C实例指针，包含端口、引脚和时序信息
 */
static inline void SOFT_I2C_Start(SOFT_I2C_Instance *instance)
{
    // 拉高SCL和SDA线，再拉低SDA线，再拉低SCL线，完成起始信号发送
    HAL_GPIO_WritePin(instance->port,instance->scl,1);
    HAL_GPIO_WritePin(instance->port,instance->sda,1);
    DWT_Delay_us(instance->i2c_tick);
    HAL_GPIO_WritePin(instance->port,instance->sda,0);
    DWT_Delay_us(instance->i2c_tick);
    HAL_GPIO_WritePin(instance->port,instance->scl,0);
}
/**
 * @brief 发送软件I2C停止信号
 * 
 * 该函数通过控制SDA和SCL线产生I2C总线的停止信号。
 * 停止信号的时序：先拉低SDA线和SCL线，然后拉高SCL线，最后拉高SDA线。
 * 
 * @param instance 软件I2C实例指针，包含端口、引脚和时钟配置信息
 */
static inline void SOFT_I2C_Stop(SOFT_I2C_Instance *instance)
{
    // 拉低SDA线和SCL线，再拉高SCL线，再拉高SDA线，完成停止信号发送
    HAL_GPIO_WritePin(instance->port,instance->sda,0);
    DWT_Delay_us(instance->i2c_tick);
    HAL_GPIO_WritePin(instance->port,instance->scl,1);
    DWT_Delay_us(instance->i2c_tick);
    HAL_GPIO_WritePin(instance->port,instance->sda,1);
}
/**
 * @brief 向I2C总线写入一个数据位
 * 
 * 该函数负责向软件模拟的I2C总线写入单个数据位，通过控制SDA线的电平状态，
 * 并配合SCL时钟线的时序来完成数据传输
 * 
 * @param instance I2C实例指针，包含端口、引脚和时序配置信息
 * @param bit 要写入的数据位，0或1
 */
static inline void SOFT_I2C_Write_Bit(SOFT_I2C_Instance *instance, uint8_t bit)
{
    // 发送单个位，通过设置SDA线的电平，然后拉高SCL线，再拉低SCL线完成发送
    HAL_GPIO_WritePin(instance->port,instance->sda,bit);
    DWT_Delay_us(instance->i2c_tick);
    HAL_GPIO_WritePin(instance->port,instance->scl,1);
    DWT_Delay_us(instance->i2c_tick);
    HAL_GPIO_WritePin(instance->port,instance->scl,0);
}
/**
 * @brief 向I2C总线写入一个数据位
 * 
 * 该函数负责向软件模拟的I2C总线写入单个数据位，通过控制SDA线的电平状态，
 * 并配合SCL时钟线的时序来完成数据传输
 * 
 * @param instance I2C实例指针，包含端口、引脚和时序配置信息
 * @param bit 要写入的数据位，0或1
 */
static inline uint8_t SOFT_I2C_Read_Bit(SOFT_I2C_Instance *instance)
{
    //拉高SCL线再拉低，读取SDA线电平
    HAL_GPIO_WritePin(instance->port,instance->sda,1); // 释放SDA线，准备读取
    DWT_Delay_us(instance->i2c_tick);
    HAL_GPIO_WritePin(instance->port,instance->scl,1);
    DWT_Delay_us(instance->i2c_tick);
    uint8_t bit=HAL_GPIO_ReadPin(instance->port,instance->sda);
    HAL_GPIO_WritePin(instance->port,instance->scl,0);

    //返回读取到的位值
    return bit;
}
/**
 * @brief 向I2C总线写入一个字节数据
 * 
 * 该函数通过逐位发送的方式向I2C设备写入一个字节的数据，并读取从机应答位
 * 
 * @param instance I2C软件模拟实例指针，包含所需GPIO引脚信息
 * @param byte 要写入的字节数据
 * @return uint8_t 返回从机应答信号，0表示应答(ACK)，1表示非应答(NACK)
 */
uint8_t SOFT_I2C_Write_Byte(SOFT_I2C_Instance *instance, uint8_t byte)
{
    // 由高位到低位逐位发送数据
    for(uint8_t i=0;i<8;i++)
        //里面的括号其实可以不要，但是加上更清晰一些，这是一个good habit
        SOFT_I2C_Write_Bit(instance,byte >> (7 - i) & 0x01);
    // 读取从机应答位Ack/Nack
    return SOFT_I2C_Read_Bit(instance);
}
/**
 * @brief 从I2C总线上读取一个字节的数据
 * 
 * @param instance I2C实例指针，包含I2C引脚配置信息
 * @param ack 读取完成后发送的应答信号，1为发送NACK（非应答），0为发送ACK（应答）
 * @return uint8_t 读取到的一个字节数据
 */
uint8_t SOFT_I2C_Read_Byte(SOFT_I2C_Instance *instance, uint8_t ack)
{
    // 由高位到低位逐位读取数据
    uint8_t byte=0;
    for(uint8_t i=0;i<8;i++)
        byte = (byte << 1) + SOFT_I2C_Read_Bit(instance);
    SOFT_I2C_Write_Bit(instance,ack);
    // 发送ACK/NACK
    return byte;
}

/**
 * @brief 向I2C设备写入数据
 * 
 * 此函数用于向指定的I2C设备写入数据。它会先发送起始信号，
 * 接着直接发送要传输的数据字节，
 * 最后发送停止信号结束传输。
 * 
 * @param instance I2C实例指针，包含GPIO端口、引脚和设备地址等信息
 * @param data 要写入的数据缓冲区指针
 * @param length 要写入的数据长度（字节数）
 *
 * @note 不包含寄存器地址发送，需要用户自行处理
 */
void SOFT_I2C_Write(SOFT_I2C_Instance *instance, uint8_t *data, uint16_t length)
{
    // 发送起始信号
    SOFT_I2C_Start(instance);
    // 发送设备地址（写操作），检查应答
    if(SOFT_I2C_Write_Byte(instance,instance->dev_addr))
    {
        SOFT_I2C_Stop(instance);
        return;
    }
    // 逐字节发送数据，并检查每个字节的应答
    for(uint16_t i=0;i<length;i++)
    {
        if(SOFT_I2C_Write_Byte(instance,data[i]))
        {
            SOFT_I2C_Stop(instance);
            return;
        }
    }
    // 发送停止信号
    SOFT_I2C_Stop(instance);
}
/**
 * @brief 软件I2C读取数据函数
 * 
 * @param instance I2C实例指针，包含I2C配置信息
 * @param data 用于存储读取到的数据的缓冲区指针
 * @param length 需要读取的数据长度（字节数）
 * @return 无返回值
 * 
 * 此函数实现软件I2C的数据读取过程，包括发送起始信号、设备地址，
 * 按I2C协议读取指定长度的数据，并在最后发送停止信号。
 */
void SOFT_I2C_Read(SOFT_I2C_Instance *instance, uint8_t *data, uint16_t length)
{
    //发送起始信号
    SOFT_I2C_Start(instance);
    //发送设备地址（读操作），检查应答
    if(SOFT_I2C_Write_Byte(instance,instance->dev_addr|0x01))
    {
        SOFT_I2C_Stop(instance);
        return;
    }
    //逐字节读取数据，最后一个字节发送NACK，其余发送ACK
    for(uint16_t i=0;i<length;i++)
    {
        if(i==length-1)
            data[i]=SOFT_I2C_Read_Byte(instance,SOFT_I2C_NACK);
        else
            data[i]=SOFT_I2C_Read_Byte(instance,SOFT_I2C_ACK);
    }
    //发送停止信号
    SOFT_I2C_Stop(instance);
}
/**
 * @brief 向I2C设备的内存地址写入数据
 * 
 * @param instance I2C软件模拟实例指针
 * @param mem_addr 要写入的内存地址
 * @param data 指向要写入的数据缓冲区的指针
 * @param length 要写入的数据长度
 *
 * 这个函数实现了一个完整的I2C写入时序：首先发送设备地址进行写操作，
 * 然后逐字节发送数据，最后发送停止信号结束传输。
 */
void SOFT_I2C_Mem_Write(SOFT_I2C_Instance *instance, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    //发送起始信号
    SOFT_I2C_Start(instance);
    //发送设备地址（写操作），检查应答
    if(SOFT_I2C_Write_Byte(instance,instance->dev_addr))
    {
        SOFT_I2C_Stop(instance);
        return;
    }
    //发送寄存器地址，检查应答
    if(SOFT_I2C_Write_Byte(instance,reg_addr))
    {
        SOFT_I2C_Stop(instance);
        return;
    }
    // 逐字节发送数据，并检查每个字节的应答
    for(uint16_t i=0;i<length;i++)
    {
        if(SOFT_I2C_Write_Byte(instance,data[i]))
        {
            SOFT_I2C_Stop(instance);
            return;
        }
    }
    // 发送停止信号
    SOFT_I2C_Stop(instance);
}
/**
 * @brief 软件I2C读取函数，从指定设备的寄存器地址读取数据
 * 
 * 该函数实现了一个完整的I2C读取时序：首先发送设备地址进行写操作，
 * 然后指定要读取的寄存器地址，接着重新开始并执行读操作。
 * 
 * @param instance I2C实例指针，包含引脚配置和设备地址信息
 * @param reg_addr 要读取的寄存器地址
 * @param data 用于存储读取到的数据的缓冲区指针
 * @param length 要读取的数据长度（字节数）
 *
 * 这个函数通过发送起始信号、设备地址和寄存器地址，
 * 然后读取指定长度的数据，最后发送停止信号来完成读取操作。
 */
void SOFT_I2C_Mem_Read(SOFT_I2C_Instance *instance, uint8_t reg_addr,uint8_t *data, uint16_t length)
{
    //发送起始信号
    SOFT_I2C_Start(instance);
    //发送设备地址（写操作），检查应答
    if(SOFT_I2C_Write_Byte(instance,instance->dev_addr))
    {
        SOFT_I2C_Stop(instance);
        return;
    }
    //发送寄存器地址，检查应答
    if(SOFT_I2C_Write_Byte(instance,reg_addr))
    {
        SOFT_I2C_Stop(instance);
        return;
    }

    //重新发送起始信号
    SOFT_I2C_Start(instance);
    //发送设备地址（读操作），检查应答
    if(SOFT_I2C_Write_Byte(instance,instance->dev_addr|0x01))
    {
        SOFT_I2C_Stop(instance);
        return;
    }
    //逐字节读取数据，最后一个字节发送NACK，其余发送ACK
    for(uint16_t i=0;i<length;i++)
    {
        if(i==length-1)
            data[i]=SOFT_I2C_Read_Byte(instance,SOFT_I2C_NACK);
        else
            data[i]=SOFT_I2C_Read_Byte(instance,SOFT_I2C_ACK);
    }
    //发送停止信号
    SOFT_I2C_Stop(instance);
}