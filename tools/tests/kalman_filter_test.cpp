/** @file kalman_filter_test.cpp @brief EKF 稠密与紧凑 H 的数值回归，链接真实 CMSIS 标量实现。 */
#include "kalman_filter.hpp"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks = 0;
static double max_state_error = 0.0, max_covariance_error = 0.0;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

// 保留优化前的运算顺序，检查实际模板实现，而不是另写一份优化公式充当被测对象。
template <int N, int M>
static bool originalUpdate(KalmanFilter<N, M>& kf, const Matrixf<M, 1>& z,
                           const Matrixf<M, 1>& hx, const Matrixf<M, N>& h, const Matrixf<M, M>& r) {
    Matrixf<M, 1> y = z - hx;
    Matrixf<M, M> s = h * kf.P_ * h.trans() + r;
    Matrixf<M, M> inverse = s.inv();
    if (inverse == Matrixf<M, M>::zeros()) return false;
    Matrixf<N, M> k = kf.P_ * h.trans() * inverse;
    kf.x_ += k * y;
    kf.P_ = (Matrixf<N, N>::eye() - k * h) * kf.P_;
    return true;
}

template <uint16_t Rows, uint16_t Cols>
static double scaledError(const Matrixf<Rows, Cols>& actual, const Matrixf<Rows, Cols>& expected) {
    double scale = 1.0, error = 0.0;
    for (int i = 0; i < Rows; ++i) {
        for (int j = 0; j < Cols; ++j) {
            CHECK(isfinite(actual[i][j]) && isfinite(expected[i][j]));
            scale = fmax(scale, fabs(double(expected[i][j])));
            error = fmax(error, fabs(double(actual[i][j]) - expected[i][j]));
        }
    }
    return error / scale;
}

template <int N, int M>
static void compare(const KalmanFilter<N, M>& actual, const KalmanFilter<N, M>& expected) {
    const double state_error = scaledError(actual.x_, expected.x_);
    const double covariance_error = scaledError(actual.P_, expected.P_);
    max_state_error = fmax(max_state_error, state_error);
    max_covariance_error = fmax(max_covariance_error, covariance_error);
    if (state_error > 0.0 || covariance_error > 0.0) {
        fprintf(stderr, "dimensions=%d,%d state_error=%g covariance_error=%g\n",
                N, M, state_error, covariance_error);
    }
    CHECK(memcmp(actual.x_[0], expected.x_[0], N * sizeof(float)) == 0);
    CHECK(memcmp(actual.P_[0], expected.P_[0], N * N * sizeof(float)) == 0);
}

static uint32_t random_state = 0x739ac12u;
static float randomValue() {
    random_state = 1664525u * random_state + 1013904223u;
    return float(random_state >> 8) / 8388608.0f - 1.0f;
}

static void analyticExample() {
    // 非对称 P 专门检查 hp 不能被 (P*H^T)^T 替代。
    KalmanFilter<2, 1> kf;
    float x[] = {2.0f, -1.0f}, p[] = {4.0f, 2.0f, 1.0f, 9.0f};
    kf.init({x, p, 1.0f});
    Matrixf<1, 2> h;
    Matrixf<1, 1> z, hx, r;
    h[0][0] = 1.0f; z[0][0] = 5.0f; hx[0][0] = 2.0f; r[0][0] = 4.0f;
    CHECK(kf.updateEKF(z, hx, h, r));
    CHECK(kf.x_[0][0] == 3.5f && kf.x_[1][0] == -0.625f);
    CHECK(kf.P_[0][0] == 2.0f && kf.P_[0][1] == 1.0f);
    CHECK(kf.P_[1][0] == 0.5f && kf.P_[1][1] == 8.75f);
    kf.reset(x, p);
    const auto compact_h = h.block<1, 1>(0, 0);
    CHECK(kf.updateEKF(z, hx, compact_h, r));
    // 第二个状态没有直接观测，仍通过非对称交叉协方差更新。
    CHECK(kf.x_[0][0] == 3.5f && kf.x_[1][0] == -0.625f);
    CHECK(kf.P_[0][0] == 2.0f && kf.P_[0][1] == 1.0f);
    CHECK(kf.P_[1][0] == 0.5f && kf.P_[1][1] == 8.75f);
}

template <int N, int M, uint16_t HCols = N>
static void randomInputs() {
    for (int trial = 0; trial < 600; ++trial) {
        KalmanFilter<N, M> actual, expected;
        actual.init({nullptr, nullptr, 1.0f});
        Matrixf<N, N> a;
        Matrixf<M, N> h;
        Matrixf<M, M> r;
        Matrixf<M, 1> z, hx;
        const float scale = (trial % 3 == 0) ? 0.001f : ((trial % 3 == 1) ? 1.0f : 100000.0f);
        for (int i = 0; i < N; ++i) {
            actual.x_[i][0] = randomValue();
            for (int j = 0; j < N; ++j) a[i][j] = randomValue();
        }
        actual.P_ = a * a.trans();
        for (int i = 0; i < N; ++i) actual.P_[i][i] += 1.0f;
        // 一半输入带小幅非对称，仍保持良好条件，避免隐含对称性假设。
        if (trial % 2) {
            for (int i = 0; i < N; ++i)
                for (int j = 0; j < N; ++j) actual.P_[i][j] += 0.02f * randomValue();
        }
        actual.P_ *= scale;
        for (int i = 0; i < M; ++i) {
            z[i][0] = randomValue(); hx[i][0] = randomValue();
            r[i][i] = scale * (1.0f + fabsf(randomValue()));
            for (int j = 0; j < HCols; ++j) h[i][j] = randomValue();
        }
        expected.reset(actual.x_[0], actual.P_[0]);
        const Matrixf<M, N> h_before = h;
        const Matrixf<M, M> r_before = r;
        CHECK((originalUpdate<N, M>(expected, z, hx, h, r)));
        if constexpr (HCols < N) {
            const auto compact_h = h.template block<M, HCols>(0, 0);
            CHECK(actual.updateEKF(z, hx, compact_h, r));
        } else {
            CHECK(actual.updateEKF(z, hx, h, r));
        }
        compare(actual, expected);
        CHECK(h == h_before && r == r_before);
    }
}

static void singularRollback() {
    KalmanFilter<6, 3> kf;
    kf.init({nullptr, nullptr, 1.0f});
    kf.x_[0][0] = 0.75f;
    Matrixf<3, 6> h;
    Matrixf<3, 1> z, hx;
    Matrixf<3, 3> r;
    z[0][0] = 1.0f;
    for (int case_id = 0; case_id < 2; ++case_id) {
        if (case_id) h[0][0] = h[1][0] = h[2][0] = 1.0f;
        const auto x_before = kf.x_;
        const auto p_before = kf.P_;
        CHECK(!kf.updateEKF(z, hx, h, r));
        const auto compact_h = h.block<3, 4>(0, 0);
        CHECK(!kf.updateEKF(z, hx, compact_h, r));
        CHECK(memcmp(kf.x_[0], x_before[0], 6 * sizeof(float)) == 0);
        CHECK(memcmp(kf.P_[0], p_before[0], 36 * sizeof(float)) == 0);
    }
    // 失败后下一拍恢复正常观测，重新计算本拍 H 的转置。
    r = Matrixf<3, 3>::eye();
    KalmanFilter<6, 3> expected;
    expected.init({kf.x_[0], kf.P_[0], 1.0f});
    CHECK((originalUpdate<6, 3>(expected, z, hx, h, r)));
    CHECK(kf.updateEKF(z, hx, h.block<3, 4>(0, 0), r));
    compare(kf, expected);
}

static void longSequence(float measurement_noise, bool compact = false) {
    KalmanFilter<6, 3> actual, expected;
    float p[36] = {};
    for (int i = 0; i < 6; ++i) p[i * 6 + i] = (i < 4) ? 100000.0f : 100.0f;
    actual.init({nullptr, p, 0.9996f});
    expected.init({nullptr, p, 0.9996f});
    Matrixf<6, 6> f = Matrixf<6, 6>::eye(), q;
    Matrixf<3, 3> r;
    for (int i = 0; i < 6; ++i) q[i][i] = (i < 4) ? 0.01f : 0.000001f;
    for (int i = 0; i < 3; ++i) r[i][i] = measurement_noise;
    for (int step = 0; step < 10000; ++step) {
        const float t = step * 0.001f;
        // 外部变化模型夹具，不声称复现完整 AHRS 或实车噪声。
        f[0][1] = 0.0002f * cosf(t); f[1][0] = -f[0][1];
        f[2][4] = 0.0005f; f[3][5] = -0.0005f;
        const auto actual_prediction = f * actual.x_;
        const auto expected_prediction = f * expected.x_;
        actual.predictEKF(actual_prediction, f, q);
        expected.predictEKF(expected_prediction, f, q);
        Matrixf<3, 6> h;
        Matrixf<3, 1> z;
        for (int i = 0; i < 3; ++i) {
            h[i][i] = 1.0f; h[i][3] = 0.2f * sinf(t + i);
            z[i][0] = 0.3f * sinf(0.7f * t + i);
        }
        const auto actual_hx = h * actual.x_;
        const auto expected_hx = h * expected.x_;
        CHECK((originalUpdate<6, 3>(expected, z, expected_hx, h, r)));
        if (compact) {
            CHECK(actual.updateEKF(z, actual_hx, h.block<3, 4>(0, 0), r));
        } else {
            CHECK(actual.updateEKF(z, actual_hx, h, r));
        }
        if (scaledError(actual.x_, expected.x_) > 0.0 || scaledError(actual.P_, expected.P_) > 0.0)
            fprintf(stderr, "long sequence: R=%g step=%d\n", double(measurement_noise), step);
        compare(actual, expected);
    }
    printf("sequence R=%g compact=%d passed; max scaled state error=%.9g covariance error=%.9g\n",
           double(measurement_noise), int(compact), max_state_error, max_covariance_error);
}

static void chiSquareExamples() {
    KalmanFilter<1, 1> scalar;
    const float x[] = {2.0f}, p[] = {3.0f};
    scalar.init({x, p, 1.0f});
    Matrixf<1, 1> z, hx, h, r;
    // h(x)=x^2，Jacobian H=2x；h(x) 与 H*x 不同。
    z[0][0] = 6.0f; hx[0][0] = 4.0f; h[0][0] = 4.0f; r[0][0] = 2.0f;
    const auto x_before = scalar.x_;
    const auto p_before = scalar.P_;
    CHECK(fabsf(scalar.chiSquare(z - hx, h, r) - 4.0f / 50.0f) < 1e-7f);
    r[0][0] = 32.0f;
    CHECK(fabsf(scalar.chiSquare(z - hx, h, r) - 4.0f / 80.0f) < 1e-7f);
    CHECK(scalar.x_ == x_before && scalar.P_ == p_before);

    KalmanFilter<2, 2> dense;
    const float covariance[] = {4.0f, 1.0f, 1.0f, 9.0f};
    dense.init({nullptr, covariance, 1.0f});
    Matrixf<2, 2> jacobian, noise;
    Matrixf<2, 1> innovation;
    jacobian[0][0] = 1.0f; jacobian[0][1] = 2.0f; jacobian[1][1] = 1.0f;
    noise[0][0] = 2.0f; noise[0][1] = noise[1][0] = 0.5f; noise[1][1] = 3.0f;
    innovation[0][0] = 2.0f; innovation[1][0] = -1.0f;
    // 手算 S=[46,19.5;19.5,12]，det(S)=171.75，二次型分子=172。
    CHECK(fabs(double(dense.chiSquare(innovation, jacobian, noise)) - 172.0 / 171.75) < 2e-6);

    KalmanFilter<6, 2> compact;
    compact.init({nullptr, nullptr, 1.0f});
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < 2; ++col) compact.P_[row][col] = covariance[row * 2 + col];
    Matrixf<2, 6> full_h;
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < 2; ++col) full_h[row][col] = jacobian[row][col];
    const auto compact_x = compact.x_;
    const auto compact_p = compact.P_;
    CHECK(fabsf(compact.chiSquare(innovation, jacobian, noise)
              - compact.chiSquare(innovation, full_h, noise)) < 1e-7f);
    innovation = Matrixf<2, 1>::zeros();
    CHECK(compact.chiSquare(innovation, jacobian, noise) == 0.0f);
    CHECK(compact.x_ == compact_x && compact.P_ == compact_p);

    // m>n、HPH^T 秩亏，正定 R 仍使 S 可逆，不能误判奇异。
    KalmanFilter<1, 2> tall;
    tall.init({nullptr, nullptr, 1.0f});
    Matrixf<2, 1> tall_h;
    tall_h[0][0] = 1.0f; tall_h[1][0] = 2.0f;
    innovation[0][0] = 3.0f; innovation[1][0] = -1.0f;
    noise = Matrixf<2, 2>::eye();
    // S=[2,2;2,5]，二次型=59/6。
    CHECK(fabs(double(tall.chiSquare(innovation, tall_h, noise)) - 59.0 / 6.0) < 2e-6);
    noise = Matrixf<2, 2>::zeros();
    CHECK(tall.chiSquare(innovation, tall_h, noise) == -1.0f);
}

int main() {
    chiSquareExamples();
    analyticExample();
    randomInputs<1, 1>(); randomInputs<3, 2>(); randomInputs<2, 3>();
    randomInputs<6, 3>(); randomInputs<9, 4>();
    randomInputs<3, 2, 1>(); randomInputs<2, 3, 1>();
    randomInputs<6, 3, 4>(); randomInputs<9, 4, 5>();
    singularRollback();
    longSequence(1000000.0f);
    longSequence(0.1f);
    longSequence(1000000.0f, true);
    longSequence(0.1f, true);
    printf("PASS: %u checks; 5400 random updates, 40000 recursive updates; "
           "max scaled state error=%.9g covariance error=%.9g\n",
           checks, max_state_error, max_covariance_error);
}
