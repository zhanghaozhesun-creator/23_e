#include "pid_controller.h"

/* PID implementation shared by the two gimbal axes. */

#define PID_DEFAULT_OUTPUT_LIMIT     (1000.0f)
#define PID_DEFAULT_INTEGRAL_LIMIT   (1000.0f)

static float PID_Abs(float value)
{
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
    pid->deadband = 0.0f;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == 0) {
        return;
    }

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
    float derivative = 0.0f;
    float output;

    if (pid == 0) {
        return 0.0f;
    }

    error = target - current;
    if (PID_Abs(error) <= pid->deadband) {
        error = 0.0f;
    }

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
