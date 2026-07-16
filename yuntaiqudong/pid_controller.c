#include "pid_controller.h"

/* PID implementation shared by the two gimbal axes. */
/*
 * 本文件的通用数值约定：pid 与 0 比较表示空指针检查；0.0f 表示
 * 对应的误差、积分、微分或输出分量为零，不产生该分量的控制作用。
 */

/* 初始化时输出限制为 -1000.0～+1000.0，云台初始化后会改为速度上限。 */
#define PID_DEFAULT_OUTPUT_LIMIT     (1000.0f)
/* 初始化时积分累计值限制为 -1000.0～+1000.0，防止积分无限增长。 */
#define PID_DEFAULT_INTEGRAL_LIMIT   (1000.0f)

static float PID_Abs(float value)
{
    /* 0.0f 是正负数的分界，返回参数绝对值。 */
    return (value < 0.0f) ? -value : value;
}

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = -PID_DEFAULT_OUTPUT_LIMIT;
    pid->output_max = PID_DEFAULT_OUTPUT_LIMIT;
    pid->integral_min = -PID_DEFAULT_INTEGRAL_LIMIT;
    pid->integral_max = PID_DEFAULT_INTEGRAL_LIMIT;
    /* 0.0f 表示默认不设置误差死区。 */
    pid->deadband = 0.0f;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == 0) {
        return;
    }

    /* 两个 0.0f 分别清除积分累计值和上一次误差。 */
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->first_update = true;
}

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimit(PID_Controller *pid, float min_output, float max_output)
{
    if ((pid == 0) || (min_output >= max_output)) {
        return;
    }

    pid->output_min = min_output;
    pid->output_max = max_output;
}

void PID_SetIntegralLimit(
    PID_Controller *pid, float min_integral, float max_integral)
{
    if ((pid == 0) || (min_integral >= max_integral)) {
        return;
    }

    pid->integral_min = min_integral;
    pid->integral_max = max_integral;
    pid->integral =
        PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
}

void PID_SetDeadband(PID_Controller *pid, float deadband)
{
    if (pid == 0) {
        return;
    }

    pid->deadband = PID_Abs(deadband);
}

float PID_Update(
    PID_Controller *pid, float target, float current, float dt_sec)
{
    float error;
    /* 0.0f 表示本次默认不加入微分项，取得有效历史误差后再计算。 */
    float derivative = 0.0f;
    float output;

    if (pid == 0) {
        /* PID 对象无效时返回 0.0f，即不给电机速度命令。 */
        return 0.0f;
    }

    /*
     * 位置式 PID 计算公式：
     *   误差：       e(k) = target(k) - current(k)
     *   积分：       I(k) = clamp(I(k-1) + e(k) * dt)
     *   微分：       D(k) = (e(k) - e(k-1)) / dt
     *   原始输出：   u(k) = Kp*e(k) + Ki*I(k) + Kd*D(k)
     *   最终输出：   output = clamp(u(k), output_min, output_max)
     *
     * 云台角度控制中，误差单位为 deg、dt 单位为 s，PID 输出作为
     * 电机角速度命令使用，单位为 deg/s。
     */
    error = target - current;
    if (PID_Abs(error) <= pid->deadband) {
        /* 死区内把误差置为 0.0f，抑制零点附近反复动作。 */
        error = 0.0f;
    }

    /* dt_sec 必须大于 0.0f；零或负周期无法进行积分和除法求微分。 */
    if (dt_sec > 0.0f) {
        pid->integral += error * dt_sec;
        pid->integral =
            PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);

        if (!pid->first_update) {
            derivative = (error - pid->previous_error) / dt_sec;
        }
    }

    output = (pid->kp * error) + (pid->ki * pid->integral) +
             (pid->kd * derivative);
    output = PID_Clamp(output, pid->output_min, pid->output_max);

    pid->previous_error = error;
    pid->first_update = false;

    return output;
}
