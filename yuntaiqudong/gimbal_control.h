#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdbool.h>

#include "pid_controller.h"
#include "step_motor.h"

typedef enum {
    GIMBAL_CONTROL_MODE_IDLE = 0,
    GIMBAL_CONTROL_MODE_ANGLE,
    GIMBAL_CONTROL_MODE_VISION_TRACK,
    GIMBAL_CONTROL_MODE_AIM,
    GIMBAL_CONTROL_MODE_IMU_STABILIZE,
    GIMBAL_CONTROL_MODE_SPEED_SMOOTH
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

void GimbalControl_UpdateAngle(GimbalControl *control, GimbalAngle target,
    GimbalAngle current, float dt_sec);
void GimbalControl_UpdateVisionTrack(GimbalControl *control,
    GimbalPoint target_px, GimbalPoint image_center_px, float yaw_deg_per_px,
    float pitch_deg_per_px, float dt_sec);
void GimbalControl_UpdateAim(GimbalControl *control, GimbalPoint target_px,
    GimbalPoint laser_px, GimbalPoint image_center_px, float yaw_deg_per_px,
    float pitch_deg_per_px, float dt_sec);
void GimbalControl_UpdateImuStabilize(GimbalControl *control,
    GimbalAngle stable_reference, GimbalAngle imu_angle, float dt_sec);
void GimbalControl_UpdateSpeedSmooth(GimbalControl *control,
    GimbalAngle target_speed, GimbalAngle current_speed, float dt_sec);

#endif
