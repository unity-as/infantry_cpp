#include "hal_stubs.h"
#include "bsp_tim.h"
#include "daemon.h"
#include "serial.h"
#include "vt13.h"
#include "dt7.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

static unsigned checks = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { \
    std::cerr << __LINE__ << ": " << #expr << '\n'; std::exit(1); } } while (0)
static bool near(float a, float b, float tol = 0.001f) { return std::fabs(a - b) < tol; }
static void ticks(unsigned count, TIM_HandleTypeDef* timer = &htim5) {
    while (count--) TIM::periodElapsedCallback(timer);
}
static void lost(void* ptr) { ++*static_cast<unsigned*>(ptr); }

static void testTim() {
    CHECK(TIM::updatePeriodUs(&htim5) == 2000);
#ifdef TEST_H7
    const auto apb2_timers = {TIM1, TIM8, TIM15, TIM16, TIM17};
#else
    const auto apb2_timers = {TIM1, TIM8, TIM9, TIM10, TIM11};
#endif
    for (auto* instance : apb2_timers) {
        TIM_HandleTypeDef timer{instance, {167, 999, 0}};
        CHECK(TIM::updatePeriodUs(&timer) == 1000);
    }
    RCC->CFGR = 0;
    RCC->D2CFGR = 0;
    CHECK(TIM::updatePeriodUs(&htim5) == 4000);  // APB1 has no x2 when /1.
    TIM_HandleTypeDef apb2{TIM1, {167, 999, 0}};
    CHECK(TIM::updatePeriodUs(&apb2) == 2000);
    apb2.Init.RepetitionCounter = 1;
    CHECK(TIM::updatePeriodUs(&apb2) == 4000);
    CHECK(TIM::updatePeriodUs(nullptr) == 0);
    apb2.Instance = nullptr;
    CHECK(TIM::updatePeriodUs(&apb2) == 0);
    test_pclk1 = 0;
    CHECK(TIM::updatePeriodUs(&htim5) == 0);
}

static void testDaemon() {
    TIM_HandleTypeDef one_ms{TIM5, {83, 999, 0}};
    TIM_HandleTypeDef half_ms{TIM5, {83, 499, 0}};
    struct Case { TIM_HandleTypeDef* timer; uint32_t ms; unsigned n; };
    for (const auto& c : {Case{&htim5, 100, 50}, Case{&one_ms, 100, 100},
                         Case{&half_ms, 2, 4}, Case{&htim5, 2, 1},
                         Case{&htim5, 0, 1}, Case{&htim5, 3, 2},
                         Case{&one_ms, 1, 1}, Case{&htim5, 1, 1}}) {
        // Objects remain alive because TIM holds their addresses.
        static Daemon dogs[8];
        static unsigned calls[8]{};
        static unsigned i = 0;
        auto& dog = dogs[i]; auto& called = calls[i++];
        dog.init({{c.timer}, c.ms, lost, &called});
        ticks(c.n, c.timer);
        CHECK(dog.online_ && called == 0);
        ticks(1, c.timer);
        CHECK(!dog.online_ && called == 1);
        ticks(5, c.timer);
        CHECK(called == 1);
        dog.reset();
        CHECK(dog.online_);
        dog.online_ = 0;
    }
    Daemon late;
    unsigned called = 0;
    late.init({{nullptr}, 100, lost, &called});
    late.setTimbase(&htim5);
    ticks(50); CHECK(late.online_);
    late.setTimbase(&one_ms);  // Rebinding also reloads the converted timeout.
    ticks(100, &one_ms); CHECK(late.online_);
    ticks(1, &one_ms); CHECK(!late.online_ && called == 1);
    // No clock available: preserve source fallback of one tick per ms.
    test_pclk1 = 0;
    Daemon fallback;
    fallback.init({{&htim5}, 2, lost, &called});
    ticks(2); CHECK(fallback.online_);
    ticks(1); CHECK(!fallback.online_ && called == 2);
}

static std::vector<uint8_t> received;
static Serial serial;
static unsigned serial_calls = 0;
static void serialRx(uint16_t len) {
    ++serial_calls;
    received.assign(serial.recv_buf_, serial.recv_buf_ + len);
}
static void testSerial() {
    UART_HandleTypeDef uart{};
    serial.init({&uart, &htim5, serialRx});
    std::array<uint8_t, 256> full{};
    for (unsigned i = 0; i < full.size(); ++i) full[i] = i;
    USART::inject(&uart, full.data(), full.size());
    CHECK(serial_calls == 0);
    ticks(1); CHECK(serial_calls == 0);
    ticks(1); CHECK(serial_calls == 1 && received.size() == 256);
    CHECK(std::equal(received.begin(), received.end(), full.begin()));
    const uint8_t tail[]{10, 20, 30};
    USART::inject(&uart, tail, 3);
    CHECK(serial_calls == 2 && received == std::vector<uint8_t>({10, 20, 30}));
    ticks(5); CHECK(serial_calls == 2);  // Idle reception cancels full timeout.
    // Fill the rest of the ring, then receive Idle before expiry.
    USART::inject(&uart, full.data(), 253);
    ticks(1); CHECK(serial_calls == 2);
    USART::inject(&uart, tail, 2);
    CHECK(serial_calls == 3 && received.size() == 255);
    ticks(3); CHECK(serial_calls == 3);
    // Flush the remaining ring tail so the next scenario starts at offset zero.
    USART::inject(&uart, full.data(), 254);
    ticks(2); CHECK(serial_calls == 4 && received.size() == 254);
    // Another full event reloads the watchdog, even after it reached zero.
    USART::inject(&uart, full.data(), 256);
    ticks(1); CHECK(serial_calls == 4);
    USART::inject(&uart, full.data(), 256);
    ticks(1); CHECK(serial_calls == 4);
    ticks(1); CHECK(serial_calls == 5 && received.size() == 256);
    CHECK(std::equal(received.begin(), received.end(), full.begin()));
}

static void testSerialStartup() {
    UART_HandleTypeDef uart{};
    serial.init({&uart, &htim5, serialRx});
    ticks(1); CHECK(serial_calls == 0);
    ticks(1); CHECK(serial_calls == 1 && received.size() == 256);
    ticks(10); CHECK(serial_calls == 1);
}

template <size_t N>
static void bits(std::array<uint8_t, N>& frame, unsigned start, unsigned count, unsigned value) {
    for (unsigned b = 0; b < count; ++b) {
        const unsigned pos = start + b;
        frame[pos / 8] = (frame[pos / 8] & ~(1u << (pos % 8))) |
                        (((value >> b) & 1u) << (pos % 8));
    }
}
// Independent bitwise reference CRC, rather than using the production generator.
static void crc(std::array<uint8_t, 21>& f) {
    uint16_t value = 0xFFFF;
    for (unsigned i = 0; i < 19; ++i) {
        value ^= f[i];
        for (unsigned j = 0; j < 8; ++j)
            value = (value >> 1) ^ ((value & 1) ? 0x8408 : 0);
    }
    f[19] = value & 0xFF; f[20] = value >> 8;
}
static void testVt13() {
    UART_HandleTypeDef uart{{921600, 8, 0, 1, UART_MODE_RX}};
    CHECK(VT13_Init(nullptr) == nullptr && !VT13_Online());
    for (auto* field : {&uart.Init.BaudRate, &uart.Init.WordLength, &uart.Init.Parity,
                       &uart.Init.StopBits, &uart.Init.Mode}) {
        const auto value = *field; *field = 0xFFFFFFFFu;
        if (field == &uart.Init.Mode) *field = 0;
        CHECK(VT13_Init(&uart) == nullptr && !VT13_Online());
        *field = value;
    }
    CHECK(USART::ports.empty() && test_log_errors == 6);
    CHECK(VT13_Init(&uart) == vt13_data && !VT13_Online());
    ticks(60); CHECK(!VT13_Online());
    std::array<uint8_t, 21> frame{0xA9, 0x53};
    bits(frame, 16, 11, 1684); bits(frame, 27, 11, 364);
    bits(frame, 38, 11, 1200); bits(frame, 49, 11, 900);
    bits(frame, 60, 2, 2); bits(frame, 62, 1, 1);
    bits(frame, 63, 1, 1); bits(frame, 64, 1, 1);
    bits(frame, 65, 11, 1025); bits(frame, 76, 1, 1);
    bits(frame, 80, 16, static_cast<uint16_t>(-123)); bits(frame, 96, 16, 456);
    bits(frame, 112, 16, 789); bits(frame, 128, 2, 1);
    bits(frame, 130, 2, 2); bits(frame, 132, 2, 3);
    bits(frame, 136, 16, VT13_KEY_W | VT13_KEY_B); crc(frame);
    auto inject = [&](const auto& f, unsigned n = 21) { USART::inject(&uart, f.data(), n); };
    auto bad = frame; bad[0] = 0; crc(bad); inject(bad);
    CHECK(!VT13_Online());
    bad = frame; bad[8] ^= 1; inject(bad); CHECK(!VT13_Online());
    inject(frame, 20); CHECK(!VT13_Online());
    inject(frame);
    CHECK(VT13_Online() && vt13_data_flag);
    CHECK(VT13_RC_RH() == 660 && VT13_RC_RV() == -660);
    CHECK(VT13_RC_LV() == 176 && VT13_RC_LH() == -124);
    CHECK(VT13_RC_SWITCH() == 2 && VT13_RC_WHEEL() == 1 && VT13_RC_TRIGGER());
    CHECK(VT13_RC_FN_LEFT() && VT13_RC_FN_RIGHT() && VT13_RC_PAUSE());
    CHECK(vt13_data->mouse_x == -123 && vt13_data->mouse_y == 456 && vt13_data->mouse_z == 789);
    CHECK(VT13_MOUSE_LEFT_PRESSED() == 1 && VT13_MOUSE_RIGHT_PRESSED() == 2 && VT13_MOUSE_MIDDLE_PRESSED() == 3);
    CHECK(VT13_KEY_PRESSED(VT13_KEY_W) && VT13_KEY_PRESSED(VT13_KEY_B));
    ticks(25); inject(bad); CHECK(VT13_RC_RH() == 660);  // Bad packet neither replaces nor feeds.
    ticks(25); CHECK(VT13_Online()); ticks(1); CHECK(!VT13_Online() && !vt13_data_flag);
    std::array<uint8_t, 21> zero{};
    CHECK(std::memcmp(vt13_data, zero.data(), zero.size()) == 0);
    inject(frame); CHECK(VT13_Online() && vt13_data_flag);
}

static void testDt7() {
    UART_HandleTypeDef uart{{100000, 9, UART_PARITY_EVEN, 1, UART_MODE_RX}};
    CHECK(DT7_Init(nullptr) == nullptr && !DT7_Online());
    for (auto* field : {&uart.Init.BaudRate, &uart.Init.WordLength, &uart.Init.Parity,
                       &uart.Init.StopBits, &uart.Init.Mode}) {
        const auto value = *field; *field = 0;
        CHECK(DT7_Init(&uart) == nullptr && !DT7_Online()); *field = value;
    }
    CHECK(USART::ports.empty() && test_log_errors == 6);
    CHECK(DT7_Init(&uart) == dt7_data && !DT7_Online());
    std::array<uint8_t, 18> frame{};
    bits(frame, 0, 11, 364); bits(frame, 11, 11, 1684);
    bits(frame, 22, 11, 900); bits(frame, 33, 11, 1200);
    bits(frame, 44, 2, 1); bits(frame, 46, 2, 3); bits(frame, 128, 11, 1684);
    auto inject = [&](const auto& f, unsigned n = 18) { USART::inject(&uart, f.data(), n); };
    inject(frame, 17); CHECK(!DT7_Online());
    // Exercise both boundaries on all five channels, including the dial.
    for (unsigned pos : {0u, 11u, 22u, 33u, 128u}) {
        for (unsigned value : {363u, 1685u}) {
            auto bad = frame; bits(bad, pos, 11, value); inject(bad); CHECK(!DT7_Online());
        }
    }
    for (unsigned pos : {44u, 46u}) {
        auto bad = frame; bits(bad, pos, 2, 0); inject(bad); CHECK(!DT7_Online());
    }
    inject(frame); CHECK(DT7_Online() && dt7_data_flag);
    CHECK(DT7_RC_RH() == -660 && DT7_RC_RV() == 660);
    CHECK(DT7_RC_LH() == -124 && DT7_RC_LV() == 176 && DT7_RC_WHEEL() == 660);
    CHECK(dt7_data->switch_right == 1 && dt7_data->switch_left == 3);
    ticks(25); auto bad = frame; bits(bad, 128, 11, 0); inject(bad);
    CHECK(DT7_RC_WHEEL() == 660);
    ticks(25); CHECK(DT7_Online()); ticks(1); CHECK(!DT7_Online() && !dt7_data_flag);
    CHECK(DT7_RC_RH() == 0 && DT7_RC_RV() == 0 && DT7_RC_WHEEL() == 0);
    inject(frame); CHECK(DT7_Online() && dt7_data_flag);
}

static int16_t currentCount(const TestTx& tx, unsigned slot = 0) {
    return static_cast<int16_t>((tx.data[2 * slot] << 8) | tx.data[2 * slot + 1]);
}
static void testDji() {
    DJIMotor a, b, other;
    DJIMotor::Config cfg{};
    cfg.can_handle = &hcan1; cfg.motor_type = DJIMotor_3508;
    cfg.motor_id = 1; cfg.reduction_ratio = 2;
    a.init(cfg);  // Before timebase selection.
    DJIMotor::timbaseSelect(&htim5);
    cfg.motor_id = 2; cfg.direction = DJIM_DIRECTION_REVERT; b.init(cfg);
    cfg.can_handle = &hcan2; cfg.direction = DJIM_DIRECTION_NORMAL; other.init(cfg);
    a.setEnable(1); b.setEnable(1); other.setEnable(1);
    a.setCurrent(10); ticks(1); CHECK(a.command_current_cnt_ == 0); // No first feedback.
    const std::array<uint8_t, 8> feedback{0x04, 0, 0, 100, 0x20, 0, 42, 0};
    CAN::inject(&hcan1, 0x201, feedback); CAN::inject(&hcan1, 0x202, feedback);
    CAN::inject(&hcan2, 0x202, feedback);
    CHECK(a.motor_valid_ && b.motor_valid_ && other.motor_valid_);
    CHECK(near(a.velocity_, 300) && near(b.velocity_, -300)); // First frame is unfiltered.
    CHECK(near(a.current_, 10) && near(b.current_, -10));
    a.setCurrent(10); b.setCurrent(30); other.setCurrent(-5);
    CAN::sent.clear(); CAN::results = {0, 1}; ticks(1);
    CHECK(CAN::sent.size() == 2);
    CHECK(currentCount(CAN::sent[0]) == 8192 && currentCount(CAN::sent[0], 1) == -16384);
    CHECK(a.command_current_cnt_ == 8192 && b.command_current_cnt_ == -16384);
    CHECK(!a.command_tx_ok_ && !b.command_tx_ok_ && other.command_tx_ok_);
    CHECK(a.command_tx_fail_count_ == 1 && b.command_tx_fail_count_ == 1 && other.command_tx_fail_count_ == 0);
    for (const auto& tx : CAN::sent) CHECK(near(tx.timeout_ms, 0.01f, 0.000001f));
    ticks(1); CHECK(a.command_tx_ok_ && a.command_tx_fail_count_ == 1);
    a.setEnable(0); ticks(1); CHECK(a.command_current_cnt_ == 0);
    ticks(47); CHECK(a.motor_valid_ && b.motor_valid_ && other.motor_valid_);
    ticks(1); CHECK(!a.motor_valid_ && !b.motor_valid_ && !other.motor_valid_);
    ticks(1); // A later-registered watchdog may run after this tick's TX group.
    CHECK(b.command_current_cnt_ == 0 && other.command_current_cnt_ == 0);
    CAN::inject(&hcan1, 0x202, feedback); b.setCurrent(1); ticks(1);
    CHECK(b.motor_valid_ && b.command_current_cnt_ < 0);
}

static void testDm() {
    DMMotor motor;
    DMMotor::Config cfg{};
    cfg.can_handle = &hcan1; cfg.control_id = 5; cfg.feedback_id = 0x15;
    cfg.htim = &htim5; cfg.reduction_ratio = 2; cfg.initial_angle = 0.5f;
    cfg.direction = DM_DIRECTION_REVERT;
    motor.init(cfg);
    CHECK(CAN::sent.back().id == 0x7FF && CAN::sent.back().data[3] == 10 && CAN::sent.back().data[4] == 1);
    ticks(1); CHECK(CAN::sent.back().data[7] == 0xFD); // Disabled heartbeat.
    const auto n = CAN::sent.size(); motor.setEnable(0); CHECK(CAN::sent.size() == n);
    std::array<uint8_t, 8> fb{0x15, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 40, 50};
    CAN::inject(&hcan1, 0x15, fb);
    CHECK(motor.running() && !motor.faulted() && !motor.shouldAutoClear());
    CHECK(near(motor.angle_raw_, 6.25f) && near(motor.angle_, -6.75f));
    CHECK(near(motor.velocity_, -22.5f) && near(motor.current_, -54));
    CHECK(motor.mos_temp_ == 40 && motor.rotor_temp_ == 50);
    fb[0] = 0x25; CAN::inject(&hcan1, 0x15, fb);
    CHECK(motor.faulted() && !motor.shouldAutoClear());
    fb[0] = 0xA5; CAN::inject(&hcan1, 0x15, fb);
    CHECK(motor.faulted() && motor.shouldAutoClear());
    fb[0] = 0x45; CAN::inject(&hcan1, 0x15, fb);
    CHECK(static_cast<uint8_t>(motor.state_) == 4 && !motor.faulted());
    fb[0] = 0x16; CAN::inject(&hcan1, 0x15, fb); // Wrong embedded motor ID.
    CHECK(static_cast<uint8_t>(motor.state_) == 4);
    motor.setEnable(1); CHECK(CAN::sent.back().data[7] == 0xFC);
    motor.setCurrent(54); ticks(1);
    CHECK(CAN::sent.back().id == 5 && (CAN::sent.back().data[6] & 15) == 0 && CAN::sent.back().data[7] == 0);
    motor.requestMode(DMMotor::Mode::Position);
    CAN::results = {0}; ticks(1);
    CHECK(motor.modeSwitchStatus() == DMMotor::ModeSwitchStatus::Disabling);
    ticks(1); CHECK(motor.modeSwitchStatus() == DMMotor::ModeSwitchStatus::Writing);
    CAN::results = {0}; ticks(1); CHECK(motor.modeSwitchStatus() == DMMotor::ModeSwitchStatus::Writing);
    ticks(1); CHECK(motor.modeSwitchStatus() == DMMotor::ModeSwitchStatus::Waiting);
    CHECK(CAN::sent.back().id == 0x7FF && CAN::sent.back().data[4] == 2);
    const auto waiting = CAN::sent.size(); ticks(1); CHECK(CAN::sent.size() == waiting);
    CAN::results = {0}; ticks(1); CHECK(motor.modeSwitchStatus() == DMMotor::ModeSwitchStatus::Enabling);
    ticks(1); CHECK(!motor.modeSwitching() && motor.mode() == DMMotor::Mode::Position && motor.motor_enable_);
    CHECK(near(motor.target_angle_, motor.angle_) && motor.target_velocity_ == 0 && motor.target_current_ == 0);
    motor.setAngle(1, -3); ticks(1);
    float p, v; std::memcpy(&p, CAN::sent.back().data.data(), 4);
    std::memcpy(&v, CAN::sent.back().data.data() + 4, 4);
    CHECK(CAN::sent.back().id == 0x105 && near(p, -3) && near(v, 6));
    motor.requestMode(DMMotor::Mode::Velocity); ticks(2);
    motor.requestMode(DMMotor::Mode::Mit);  // Queued after the current register write.
    ticks(2); CHECK(motor.mode() == DMMotor::Mode::Velocity && motor.modeSwitching());
    motor.setEnable(0); ticks(4);
    CHECK(motor.mode() == DMMotor::Mode::Mit && !motor.modeSwitching() && !motor.motor_enable_);
    ticks(1); CHECK(CAN::sent.back().data[7] == 0xFD);
    for (const auto& tx : CAN::sent) CHECK(near(tx.timeout_ms, 0.01f, 0.000001f));
    fb[0] = 0x15; CAN::inject(&hcan1, 0x15, fb);
    ticks(50); CHECK(motor.motor_valid_);
    ticks(1); CHECK(!motor.motor_valid_);
    CAN::inject(&hcan1, 0x15, fb); CHECK(motor.motor_valid_);
    motor.clearError(); CHECK(CAN::sent.back().data[7] == 0xFB);
    motor.setZero(); CHECK(CAN::sent.back().data[7] == 0xFE);
    DMMotor custom;
    cfg.control_id = 6; cfg.feedback_id = 0x16;
    cfg.direction = DM_DIRECTION_NORMAL; cfg.t_min = -18; cfg.t_max = 18;
    custom.init(cfg);
    fb[0] = 0x16; CAN::inject(&hcan1, 0x16, fb);
    CHECK(near(custom.current_, 18)); // Per-motor TMAX overrides remain supported.
}

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const std::string name = argv[1];
    if (name == "tim") testTim();
    else if (name == "daemon") testDaemon();
    else if (name == "serial") testSerial();
    else if (name == "serial_1ms") {
        htim5.Init.Prescaler = 83;
        testSerial();
    }
    else if (name == "serial_startup") testSerialStartup();
    else if (name == "vt13") testVt13();
    else if (name == "dt7") testDt7();
    else if (name == "dji") testDji();
    else if (name == "dm") testDm();
    else return 2;
    std::cout << name << ": " << checks << " checks passed\n";
}
