#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdbool.h>

#include "pid_controller.h"
#include "step_motor.h"

typedef enum {
    GIMBAL_CONTROL_MODE_IDLE = 0,       /* 0：空闲并停止两轴。 */
    GIMBAL_CONTROL_MODE_ANGLE,          /* 1：角度闭环。 */
    GIMBAL_CONTROL_MODE_VISION_TRACK,   /* 2：视觉目标跟踪。 */
    GIMBAL_CONTROL_MODE_AIM,            /* 3：激光点瞄准。 */
    GIMBAL_CONTROL_MODE_IMU_STABILIZE,  /* 4：IMU 增稳。 */
    GIMBAL_CONTROL_MODE_SPEED_SMOOTH    /* 5：速度平滑闭环。 */
} GimbalControlMode;

typedef struct {
    float yaw;
    float pitch;
} GimbalAngle;

typedef struct {
    float x;
    float y;
    bool valid;
} GimbalPoint;

typedef struct {
    PID_Controller yaw_pid;
    PID_Controller pitch_pid;
    GimbalAngle angle_error;
    float min_effective_speed_deg_s;
    float max_speed_deg_s;
    GimbalControlMode mode;
    StepMotorDir yaw_last_dir;
    StepMotorDir pitch_last_dir;
    bool yaw_running;
    bool pitch_running;
} GimbalControl;

void GimbalControl_Init(GimbalControl *control);
void GimbalControl_SetMode(GimbalControl *control, GimbalControlMode mode);
void GimbalControl_SetYawPid(
    GimbalControl *control, float kp, float ki, float kd);
void GimbalControl_SetPitchPid(
    GimbalControl *control, float kp, float ki, float kd);
void GimbalControl_SetSpeedLimit(
    GimbalControl *control, float min_speed_deg_s, float max_speed_deg_s);
void GimbalControl_SetDeadband(GimbalControl *control, float deadband_deg);
void GimbalControl_Reset(GimbalControl *control);
void GimbalControl_Stop(GimbalControl *control);
void GimbalControl_ApplySpeed(
    GimbalControl *control, float yaw_speed_deg_s, float pitch_speed_deg_s);
GimbalAngle GimbalControl_GetAngleError(const GimbalControl *control);

void GimbalControl_UpdateAngle(GimbalControl *control, GimbalAngle target,
    GimbalAngle current, float dt_sec);
void GimbalControl_UpdateVisionTrack(GimbalControl *control,
    GimbalPoint target_px, GimbalPoint image_center_px, float yaw_deg_per_px,
    float pitch_deg_per_px, float dt_sec);
void GimbalControl_UpdateAim(GimbalControl *control, GimbalPoint target_px,
    GimbalPoint laser_px, GimbalPoint image_center_px, float yaw_deg_per_px,
    float pitch_deg_per_px, float dt_sec);
/*
 * 输入目标点减激光点的平面距离误差，X 向右、Y 向下为正，单位为 mm。
 * 函数按固定目标距离换算为角度误差后进入瞄准 PID。
 */
void GimbalControl_UpdateAimDistanceError(GimbalControl *control,
    float error_x_mm, float error_y_mm, float dt_sec);
void GimbalControl_UpdateImuStabilize(GimbalControl *control,
    GimbalAngle stable_reference, GimbalAngle imu_angle, float dt_sec);
void GimbalControl_UpdateSpeedSmooth(GimbalControl *control,
    GimbalAngle target_speed, GimbalAngle current_speed, float dt_sec);

#endif
