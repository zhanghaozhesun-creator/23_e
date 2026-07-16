#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

/* Controller state is owned by the gimbal control module. */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    bool first_update;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
void PID_Reset(PID_Controller *pid);
void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);
void PID_SetOutputLimit(PID_Controller *pid, float min_output, float max_output);
void PID_SetIntegralLimit(
    PID_Controller *pid, float min_integral, float max_integral);
void PID_SetDeadband(PID_Controller *pid, float deadband);
float PID_Update(
    PID_Controller *pid, float target, float current, float dt_sec);

#endif
