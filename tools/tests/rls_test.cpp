/** @file rls_test.cpp @brief 主机 RLS 单测；独立信息矩阵解作为数值基准。 */
#include "rls.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

static bool near(double actual, double expected, double tolerance = 2e-4) {
    return isfinite(actual) && fabs(actual - expected) <= tolerance * (1.0 + fabs(expected));
}

// 独立求逆：Gauss-Jordan，不复用生产 RLS 的秩一更新。
static void inverse3(const double matrix[3][3], double inverse[3][3]) {
    double augmented[3][6] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            augmented[i][j] = matrix[i][j];
        augmented[i][i + 3] = 1.0;
    }
    for (int column = 0; column < 3; ++column) {
        int pivot = column;
        for (int i = column + 1; i < 3; ++i)
            if (fabs(augmented[i][column]) > fabs(augmented[pivot][column])) pivot = i;
        CHECK(fabs(augmented[pivot][column]) > 1e-12);
        for (int j = 0; j < 6; ++j) {
            const double tmp = augmented[column][j];
            augmented[column][j] = augmented[pivot][j];
            augmented[pivot][j] = tmp;
        }
        const double divisor = augmented[column][column];
        for (int j = 0; j < 6; ++j) augmented[column][j] /= divisor;
        for (int i = 0; i < 3; ++i) {
            if (i == column) continue;
            const double factor = augmented[i][column];
            for (int j = 0; j < 6; ++j) augmented[i][j] -= factor * augmented[column][j];
        }
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) inverse[i][j] = augmented[i][j + 3];
}

static void batchReference(float lambda, uint16_t rows) {
    RLS<3> rls;
    float initial[] = {0.5f, -0.8f, 0.2f};
    float diagonal[] = {2.0f, 4.0f, 3.0f};
    CHECK(rls.init({lambda, initial, diagonal}));
    double information[3][3] = {};
    double rhs[3] = {};
    for (int i = 0; i < 3; ++i) {
        information[i][i] = 1.0 / diagonal[i];
        rhs[i] = initial[i] / diagonal[i];
    }
    for (int tick = 0; tick < 200; ++tick) {
        float phi[9] = {};
        float y[3] = {};
        for (int i = 0; i < 3; ++i) {
            rhs[i] *= lambda;
            for (int j = 0; j < 3; ++j) information[i][j] *= lambda;
        }
        for (uint16_t row = 0; row < rows; ++row) {
            const double t = tick * 0.13 + row * 0.73;
            float* p = phi + row * 3;
            p[0] = static_cast<float>(sin(t));
            p[1] = static_cast<float>(cos(t * 1.71));
            p[2] = 1.0f;
            y[row] = static_cast<float>(2.3 * p[0] - 1.7 * p[1] + 0.4 + 0.03 * sin(t * 2.19));
            for (int i = 0; i < 3; ++i) {
                rhs[i] += static_cast<double>(p[i]) * y[row];
                for (int j = 0; j < 3; ++j)
                    information[i][j] += static_cast<double>(p[i]) * p[j];
            }
        }
        CHECK(rls.update(phi, y, rows));
        double inverse[3][3];
        inverse3(information, inverse);
        for (int i = 0; i < 3; ++i) {
            double expected = 0.0;
            for (int j = 0; j < 3; ++j) expected += inverse[i][j] * rhs[j];
            CHECK(near(rls.parameters()[i], expected));
            CHECK(near(rls.covarianceDiagonal(static_cast<uint16_t>(i)), inverse[i][i], 2e-5));
        }
    }
    puts("PASS independent weighted batch reference (parameters and covariance)");
}

static void convergenceAndTracking() {
    RLS<3> fixed, tracking;
    float diagonal[] = {100.0f, 100.0f, 100.0f};
    CHECK(fixed.init({1.0f, nullptr, diagonal}));
    CHECK(tracking.init({0.98f, nullptr, diagonal}));
    for (int tick = 0; tick < 2400; ++tick) {
        const float phi[] = {static_cast<float>(sin(tick * 0.17)),
                             static_cast<float>(cos(tick * 0.37)), 1.0f};
        const bool changed = tick >= 1200;
        const float truth[] = {changed ? -1.0f : 2.0f, changed ? 3.0f : -0.5f, changed ? 0.2f : 0.8f};
        const float y = phi[0] * truth[0] + phi[1] * truth[1] + truth[2];
        CHECK(fixed.update(phi, &y, 1));
        CHECK(tracking.update(phi, &y, 1));
        if (tick == 1199)
            for (int i = 0; i < 3; ++i) CHECK(near(fixed.parameters()[i], truth[i], 1e-4));
    }
    const float truth[] = {-1.0f, 3.0f, 0.2f};
    for (int i = 0; i < 3; ++i) CHECK(near(tracking.parameters()[i], truth[i], 1e-4));
    CHECK(fabs(fixed.parameters()[0] - truth[0]) > 1.0f);
    const float query[] = {0.3f, -0.2f, 1.0f};
    CHECK(near(tracking.predict(query), -0.7));
    puts("PASS convergence, constant offset, prediction and parameter-change tracking");
}

static void invalidInputsAndRollback() {
    RLS<3> rls, twin;
    float phi[] = {0.7f, -0.2f, 1.0f};
    float y = 2.0f;
    CHECK(!rls.update(phi, &y, 1));
    CHECK(!rls.reset());
    CHECK(rls.parameters() == nullptr && rls.parameterCount() == 3);
    CHECK(isnan(rls.predict(phi)) && isnan(rls.covarianceDiagonal(0)));
    CHECK(!rls.init({0.0f, nullptr, nullptr}));
    CHECK(!rls.init({1.1f, nullptr, nullptr}));
    CHECK(!rls.init({NAN, nullptr, nullptr}));
    CHECK(rls.init({0.97f, nullptr, nullptr}));
    CHECK(twin.init({0.97f, nullptr, nullptr}));
    CHECK(rls.update(phi, &y, 1) && twin.update(phi, &y, 1));
    CHECK(!rls.update(nullptr, &y, 1));
    CHECK(!rls.update(phi, nullptr, 1));
    CHECK(!rls.update(phi, &y, 0));
    float bad_diag[] = {1.0f, 0.0f, 1.0f};
    CHECK(!rls.reset(nullptr, bad_diag));
    bad_diag[1] = -1.0f;
    CHECK(!rls.init({1.0f, nullptr, bad_diag}));
    bad_diag[1] = INFINITY;
    CHECK(!rls.reset(nullptr, bad_diag));
    float bad_initial[] = {0.0f, NAN, 1.0f};
    CHECK(!rls.reset(bad_initial));
    CHECK(isnan(rls.predict(nullptr)) && isnan(rls.predict(bad_initial)));
    CHECK(isnan(rls.covarianceDiagonal(3)));
    // 第一行有效，第二行坏数据；不能提交第一行或遗忘后的状态。
    float group[] = {1.0f, 0.3f, -0.2f, 0.0f, NAN, 1.0f};
    float observations[] = {0.4f, 0.5f};
    CHECK(!rls.update(group, observations, 2));
    group[4] = 0.0f;
    observations[1] = INFINITY;
    CHECK(!rls.update(group, observations, 2));
    // 输入有限，但第二行会使协方差数值坍缩；同样整组回滚。
    group[3] = FLT_MAX;
    observations[1] = 0.0f;
    CHECK(!rls.update(group, observations, 2));
    // 后续多方向更新与未经历失败的对象完全一致，覆盖不可见的非对角协方差。
    for (int tick = 0; tick < 30; ++tick) {
        phi[0] = static_cast<float>(sin(tick * 0.41));
        phi[1] = static_cast<float>(cos(tick * 0.77));
        CHECK(rls.update(phi, &y, 1) && twin.update(phi, &y, 1));
        for (uint16_t i = 0; i < 3; ++i) {
            CHECK(rls.parameters()[i] == twin.parameters()[i]);
            CHECK(rls.covarianceDiagonal(i) == twin.covarianceDiagonal(i));
        }
    }
    // reset 可使用对象自己的只读参数指针。
    const float saved = rls.parameters()[0];
    CHECK(rls.reset(rls.parameters()));
    CHECK(rls.parameters()[0] == saved && rls.covarianceDiagonal(0) == 1.0f);
    CHECK(rls.reset());
    CHECK(rls.parameters()[0] == 0.0f);
    puts("PASS invalid configuration/data, reset and whole-batch rollback");
}

static void capacityAndForgetting() {
    RLS<17> rls;
    float diagonal[RLS<17>::kParameterCount];
    float phi[RLS<17>::kParameterCount * RLS<17>::kParameterCount] = {};
    float y[RLS<17>::kParameterCount];
    for (uint16_t i = 0; i < RLS<17>::kParameterCount; ++i) {
        diagonal[i] = 1000.0f;
        phi[i * RLS<17>::kParameterCount + i] = 1.0f;
        y[i] = static_cast<float>(i) - 4.0f;
    }
    CHECK(rls.init({1.0f, nullptr, diagonal}));
    CHECK(rls.parameterCount() == RLS<17>::kParameterCount);
    for (int tick = 0; tick < 50; ++tick) CHECK(rls.update(phi, y, RLS<17>::kParameterCount));
    for (uint16_t i = 0; i < RLS<17>::kParameterCount; ++i)
        CHECK(near(rls.parameters()[i], y[i], 3e-5));
    RLS<1> scalar;
    CHECK(scalar.init({0.5f, nullptr, nullptr}));
    const float zero_phi[] = {0.0f, 0.0f, 0.0f};
    const float zero_y[] = {0.0f, 0.0f, 0.0f};
    CHECK(scalar.update(zero_phi, zero_y, 3));
    CHECK(scalar.covarianceDiagonal(0) == 2.0f); // 一拍三行，只遗忘一次。
    for (int tick = 0; tick < 126; ++tick) CHECK(scalar.update(zero_phi, zero_y, 1));
    const float covariance_before = scalar.covarianceDiagonal(0);
    CHECK(!scalar.update(zero_phi, zero_y, 1)); // 无激励时膨胀溢出，保留旧状态。
    CHECK(scalar.covarianceDiagonal(0) == covariance_before);
    float initial[] = {FLT_MAX};
    CHECK(scalar.reset(initial));
    float large_phi[] = {2.0f};
    CHECK(isnan(scalar.predict(large_phi)));
    puts("PASS template dimension beyond old limit, once-per-batch forgetting and overflow rollback");
}

static void sustainedUpdates() {
    RLS<3> rls;
    float diagonal[] = {10.0f, 10.0f, 10.0f};
    CHECK(rls.init({0.999f, nullptr, diagonal}));
    for (int tick = 0; tick < 20000; ++tick) {
        const float phi[] = {static_cast<float>(sin(tick * 0.071)),
                             static_cast<float>(cos(tick * 0.113)), 1.0f};
        const float y = 1.2f * phi[0] - 0.7f * phi[1] + 0.3f;
        CHECK(rls.update(phi, &y, 1));
        for (uint16_t i = 0; i < 3; ++i)
            CHECK(isfinite(rls.covarianceDiagonal(i)) && rls.covarianceDiagonal(i) > 0.0f);
    }
    const float truth[] = {1.2f, -0.7f, 0.3f};
    for (int i = 0; i < 3; ++i) CHECK(near(rls.parameters()[i], truth[i], 1e-4));
    puts("PASS 20000 sustained updates with finite positive covariance diagonals");
}

int main() {
    static_assert(sizeof(RLS<1>) < sizeof(RLS<3>));
    static_assert(sizeof(RLS<3>) < sizeof(RLS<10>));
    static_assert(sizeof(RLS<10>) < sizeof(RLS<17>));
    batchReference(1.0f, 1);
    batchReference(0.97f, 3);
    convergenceAndTracking();
    invalidInputsAndRollback();
    capacityAndForgetting();
    sustainedUpdates();
    printf("PASS all RLS tests (%u checks); sizeof(RLS<10>)=%zu bytes\n", checks, sizeof(RLS<10>));
    return 0;
}
