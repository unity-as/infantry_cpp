//测试PID_Set_Parameters函数的单元测试，没啥用，就是想学学怎么写单元测试
#ifdef UNIT_TEST

#include "pid_test.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

// Test counter
static int tests_run = 0;
static int tests_passed = 0;

#define TEST_ASSERT(condition, message) \
    do { \
        tests_run++; \
        if (condition) { \
            tests_passed++; \
            printf("[PASS] %s\n", message); \
        } else { \
            printf("[FAIL] %s\n", message); \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_EQ(actual, expected, message) \
    do { \
        tests_run++; \
        float diff = (actual) - (expected); \
        if (diff < 0) diff = -diff; \
        if (diff < 1e-6f) { \
            tests_passed++; \
            printf("[PASS] %s\n", message); \
        } else { \
            printf("[FAIL] %s (expected: %f, actual: %f)\n", message, (float)(expected), (float)(actual)); \
        } \
    } while(0)

// Forward declare the function under test
void PID_Set_Parameters(PID_Instance *instance, float kp, float ki, float kd);

// ==================== Test Cases ====================

void test_pid_set_parameters_null_instance(void)
{
    // Test: NULL pointer should cause early return (no crash, no modification)
    PID_Set_Parameters(NULL, 1.0f, 2.0f, 3.0f);
    TEST_ASSERT(1, "Null instance handling - no crash");
}

void test_pid_set_parameters_normal_values(void)
{
    // Test: Normal positive values
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));
    instance.kp = 0.0f;
    instance.ki = 0.0f;
    instance.kd = 0.0f;

    PID_Set_Parameters(&instance, 1.5f, 2.5f, 3.5f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 1.5f, "Normal values - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 2.5f, "Normal values - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 3.5f, "Normal values - kd set correctly");
}

void test_pid_set_parameters_zero_values(void)
{
    // Test: All zero values
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));
    instance.kp = 5.0f;
    instance.ki = 10.0f;
    instance.kd = 15.0f;

    PID_Set_Parameters(&instance, 0.0f, 0.0f, 0.0f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 0.0f, "Zero values - kp set to zero");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 0.0f, "Zero values - ki set to zero");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 0.0f, "Zero values - kd set to zero");
}

void test_pid_set_parameters_negative_values(void)
{
    // Test: Negative values
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, -1.0f, -2.0f, -3.0f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, -1.0f, "Negative values - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, -2.0f, "Negative values - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, -3.0f, "Negative values - kd set correctly");
}

void test_pid_set_parameters_large_values(void)
{
    // Test: Large positive values
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, 1e10f, 1e8f, 1e6f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 1e10f, "Large values - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 1e8f, "Large values - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 1e6f, "Large values - kd set correctly");
}

void test_pid_set_parameters_small_values(void)
{
    // Test: Small positive values (near zero but not zero)
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, 1e-10f, 1e-9f, 1e-8f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 1e-10f, "Small values - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 1e-9f, "Small values - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 1e-8f, "Small values - kd set correctly");
}

void test_pid_set_parameters_mixed_values(void)
{
    // Test: Mixed positive, negative, and zero
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, 100.5f, -50.25f, 0.0f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 100.5f, "Mixed values - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, -50.25f, "Mixed values - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 0.0f, "Mixed values - kd set to zero");
}

void test_pid_set_parameters_only_kp_negative(void)
{
    // Test: Only kp is negative, ki and kd positive
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, -10.0f, 5.0f, 2.0f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, -10.0f, "Only kp negative - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 5.0f, "Only kp negative - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 2.0f, "Only kp negative - kd set correctly");
}

void test_pid_set_parameters_only_ki_negative(void)
{
    // Test: Only ki is negative, kp and kd positive
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, 10.0f, -5.0f, 2.0f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 10.0f, "Only ki negative - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, -5.0f, "Only ki negative - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 2.0f, "Only ki negative - kd set correctly");
}

void test_pid_set_parameters_only_kd_negative(void)
{
    // Test: Only kd is negative, kp and ki positive
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, 10.0f, 5.0f, -2.0f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 10.0f, "Only kd negative - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 5.0f, "Only kd negative - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, -2.0f, "Only kd negative - kd set correctly");
}

void test_pid_set_parameters_existing_instance_state(void)
{
    // Test: Instance with existing state - parameters should be overwritten
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));
    instance.kp = 99.0f;
    instance.ki = 88.0f;
    instance.kd = 77.0f;
    instance.output = 100.0f;
    instance.setpoint = 50.0f;
    instance.integral = 25.0f;

    PID_Set_Parameters(&instance, 1.0f, 2.0f, 3.0f);

    // Verify parameters are updated
    TEST_ASSERT_FLOAT_EQ(instance.kp, 1.0f, "Existing state - kp overwritten");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 2.0f, "Existing state - ki overwritten");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 3.0f, "Existing state - kd overwritten");
    // Verify other fields not affected
    TEST_ASSERT_FLOAT_EQ(instance.output, 100.0f, "Existing state - output unchanged");
    TEST_ASSERT_FLOAT_EQ(instance.setpoint, 50.0f, "Existing state - setpoint unchanged");
    TEST_ASSERT_FLOAT_EQ(instance.integral, 25.0f, "Existing state - integral unchanged");
}

void test_pid_set_parameters_special_float_values(void)
{
    // Test: Special float values (infinity, NaN behavior depends on implementation)
    PID_Instance instance;
    memset(&instance, 0, sizeof(PID_Instance));

    PID_Set_Parameters(&instance, 1.0f, 1.0f, 1.0f);

    TEST_ASSERT_FLOAT_EQ(instance.kp, 1.0f, "Normal float - kp set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.ki, 1.0f, "Normal float - ki set correctly");
    TEST_ASSERT_FLOAT_EQ(instance.kd, 1.0f, "Normal float - kd set correctly");
}

// ==================== Main ====================

int main(void)
{
    printf("========================================\n");
    printf("  PID_Set_Parameters Unit Tests\n");
    printf("========================================\n\n");

    printf("[TEST] Null instance handling\n");
    test_pid_set_parameters_null_instance();
    printf("\n");

    printf("[TEST] Normal values (positive)\n");
    test_pid_set_parameters_normal_values();
    printf("\n");

    printf("[TEST] Zero values\n");
    test_pid_set_parameters_zero_values();
    printf("\n");

    printf("[TEST] Negative values\n");
    test_pid_set_parameters_negative_values();
    printf("\n");

    printf("[TEST] Large values\n");
    test_pid_set_parameters_large_values();
    printf("\n");

    printf("[TEST] Small values\n");
    test_pid_set_parameters_small_values();
    printf("\n");

    printf("[TEST] Mixed values (positive, negative, zero)\n");
    test_pid_set_parameters_mixed_values();
    printf("\n");

    printf("[TEST] Only kp negative\n");
    test_pid_set_parameters_only_kp_negative();
    printf("\n");

    printf("[TEST] Only ki negative\n");
    test_pid_set_parameters_only_ki_negative();
    printf("\n");

    printf("[TEST] Only kd negative\n");
    test_pid_set_parameters_only_kd_negative();
    printf("\n");

    printf("[TEST] Existing instance state preservation\n");
    test_pid_set_parameters_existing_instance_state();
    printf("\n");

    printf("[TEST] Special float values\n");
    test_pid_set_parameters_special_float_values();
    printf("\n");

    printf("========================================\n");
    printf("  Test Results: %d/%d passed\n", tests_passed, tests_run);
    printf("========================================\n");

    return (tests_passed == tests_run) ? 0 : 1;
}

#endif