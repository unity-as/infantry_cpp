#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

struct TIM_TypeDef {};
// Match CMSIS peripheral addresses. Tests only compare these pointers, never dereference them.
#define APB1PERIPH_BASE 0x40000000u
#define APB2PERIPH_BASE 0x40010000u
#define TIM2 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x0000u)
#define TIM3 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x0400u)
#define TIM4 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x0800u)
#define TIM5 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x0C00u)
#define TIM6 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x1000u)
#define TIM7 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x1400u)
#define TIM12 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x1800u)
#define TIM13 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x1C00u)
#define TIM14 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x2000u)
#define TIM1 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x0000u)
#define TIM8 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x0400u)
#ifdef TEST_H7
#define TIM15 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x4000u)
#define TIM16 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x4400u)
#define TIM17 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x4800u)
#define TIM23 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0xE000u)
#define TIM24 reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0xE400u)
#else
#define TIM9 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x4000u)
#define TIM10 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x4400u)
#define TIM11 reinterpret_cast<TIM_TypeDef*>(APB2PERIPH_BASE + 0x4800u)
#endif
struct TIM_Base_InitTypeDef {
    uint32_t Prescaler, Period, RepetitionCounter;
};
struct TIM_HandleTypeDef {
    TIM_TypeDef* Instance;
    TIM_Base_InitTypeDef Init;
};
inline TIM_HandleTypeDef htim5{TIM5, {167, 999, 0}};
#ifdef TEST_H7
inline TIM_HandleTypeDef htim16{TIM16, {167, 999, 0}};
#endif
struct RCC_TypeDef { uint32_t CFGR, D2CFGR; };
inline RCC_TypeDef test_rcc{0x1000u | 0x8000u, 0x50u | 0x400u}; // APB1 /4, APB2 /2.
#define RCC (&test_rcc)
#define RCC_CFGR_PPRE1 0x1C00u
#define RCC_CFGR_PPRE2 0xE000u
#define RCC_CFGR_PPRE1_DIV1 0u
#define RCC_CFGR_PPRE2_DIV1 0u
#define RCC_D2CFGR_D2PPRE1 0x70u
#define RCC_D2CFGR_D2PPRE2 0x700u
#define RCC_D2CFGR_D2PPRE1_DIV1 0u
#define RCC_D2CFGR_D2PPRE2_DIV1 0u
inline uint32_t test_pclk1 = 42000000u, test_pclk2 = 84000000u;
inline uint32_t HAL_RCC_GetPCLK1Freq() { return test_pclk1; }
inline uint32_t HAL_RCC_GetPCLK2Freq() { return test_pclk2; }
#define RCC_HCLK_DIV1 0u
#define RCC_APB1_DIV1 0u
#define RCC_APB2_DIV1 0u
struct RCC_ClkInitTypeDef { uint32_t APB1CLKDivider, APB2CLKDivider; };
inline void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef* clocks, uint32_t* latency) {
#ifdef TEST_H7
    clocks->APB1CLKDivider = RCC->D2CFGR & RCC_D2CFGR_D2PPRE1;
    clocks->APB2CLKDivider = RCC->D2CFGR & RCC_D2CFGR_D2PPRE2;
#else
    clocks->APB1CLKDivider = RCC->CFGR & RCC_CFGR_PPRE1;
    clocks->APB2CLKDivider = (RCC->CFGR & RCC_CFGR_PPRE2) >> 3u;
#endif
    *latency = 0;
}
inline void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*) {}
inline void HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef*) {}

#define UART_WORDLENGTH_8B 8u
#define UART_WORDLENGTH_9B 9u
#define UART_PARITY_NONE 0u
#define UART_PARITY_EVEN 2u
#define UART_STOPBITS_1 1u
#define UART_MODE_RX 2u
struct UART_InitTypeDef {
    uint32_t BaudRate, WordLength, Parity, StopBits, Mode;
};
struct UART_HandleTypeDef { UART_InitTypeDef Init; };
class USART {
public:
    using Callback = void (*)(void*, uint8_t);
    struct Config { UART_HandleTypeDef* usart_handle; };
    uint8_t recv_buff_[256]{};
    inline static std::vector<USART*> ports;
    void init(const Config& config) { uart_ = config.usart_handle; ports.push_back(this); }
    void setCallback(Callback cb, void* dev) { cb_ = cb; dev_ = dev; }
    void send(uint8_t*, uint16_t) {}
    static void inject(UART_HandleTypeDef* uart, const uint8_t* data, unsigned len) {
        for (auto* port : ports) {
            if (port->uart_ != uart) continue;
            for (unsigned i = 0; i < len; ++i) {
                port->recv_buff_[port->pos_++] = data[i];
                if (port->pos_ == 256) {
                    port->pos_ = 0;
                    port->cb_(port->dev_, 0);
                }
            }
            if (port->pos_ != 0) port->cb_(port->dev_, port->pos_);
            return;
        }
    }
private:
    UART_HandleTypeDef* uart_{};
    Callback cb_{};
    void* dev_{};
    uint16_t pos_{};
};

struct CAN_HandleTypeDef {};
using FDCAN_HandleTypeDef = CAN_HandleTypeDef;
inline CAN_HandleTypeDef hcan1, hcan2;
struct TestTx {
    CAN_HandleTypeDef* bus;
    uint32_t id;
    std::array<uint8_t, 8> data;
    float timeout_ms;
    uint8_t ok;
};
class CAN {
public:
    using Handle = CAN_HandleTypeDef;
    using Callback = void (*)(void*);
    struct Config { CAN_HandleTypeDef* can_handle; uint32_t rx_id; };
    CAN_HandleTypeDef* can_handle_{};
    uint8_t tx_buff_[8]{}, rx_buff_[8]{}, rx_len_{};
    inline static std::vector<CAN*> instances;
    inline static std::vector<TestTx> sent;
    inline static std::deque<uint8_t> results;
    void init(const Config& cfg) {
        can_handle_ = cfg.can_handle; rx_id_ = cfg.rx_id; instances.push_back(this);
    }
    void setCallback(Callback cb, void* dev) { cb_ = cb; dev_ = dev; }
    void setTxId(uint32_t id) { tx_id_ = id; }
    uint32_t getTxId() const { return tx_id_; }
    uint8_t transmit(float timeout_ms) {
        uint8_t ok = 1;
        if (!results.empty()) { ok = results.front(); results.pop_front(); }
        TestTx tx{can_handle_, tx_id_, {}, timeout_ms, ok};
        std::memcpy(tx.data.data(), tx_buff_, 8);
        sent.push_back(tx);
        return ok;
    }
    static void inject(CAN_HandleTypeDef* bus, uint32_t id,
                       const std::array<uint8_t, 8>& data) {
        for (auto* can : instances) {
            if (can->can_handle_ == bus && can->rx_id_ == id && can->cb_) {
                std::memcpy(can->rx_buff_, data.data(), 8);
                can->rx_len_ = 8;
                can->cb_(can->dev_);
            }
        }
    }
private:
    uint32_t rx_id_{}, tx_id_{};
    Callback cb_{};
    void* dev_{};
};

inline void DWT_Delay_ms(float) {}
#define ROBOT_RTT_PRINT 0
#define ROBOT_RTT_REPORT_CLEAR_ERROR 0
#define ROBOT_RTT_REPORT_MOTOR_DETAIL 0
inline unsigned test_log_errors = 0;
#define LOG_ERR(...) (++test_log_errors)
