#include "gimbal_control.h"

#include <stdint.h>

/*
 * 本文件的通用数值约定：指针与 0 比较表示空指针检查；速度 0.0f
 * 表示停机命令；有符号速度大于等于 0 表示顺时针，小于 0 表示逆时针。
 */
/* 低于 1 deg/s 的命令视为无效，以避免电机在低频下抖动或失步。 */
#define GIMBAL_CONTROL_DEFAULT_MIN_SPEED_DEG_S   (1.0f)
/* PID 角速度命令的默认软件上限为 360 deg/s，即每秒 1 圈。 */
#define GIMBAL_CONTROL_DEFAULT_MAX_SPEED_DEG_S   (360.0f)

static float GimbalControl_Abs(float value)
{
    /* 0.0f 是正负数的分界，返回速度或误差的绝对值。 */
    return (value < 0.0f) ? -value : value;
}

static float GimbalControl_Clamp(
    float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static uint32_t GimbalControl_ToSpeedUint(float speed_deg_s)
{
    /* 非正速度统一转换为 0，底层把 0 解释为停止。 */
    if (speed_deg_s <= 0.0f) {
        return 0U;
    }

    /* 超出 uint32_t 范围时饱和到 UINT32_MAX，防止强制转换回绕。 */
    if (speed_deg_s >= (float) UINT32_MAX) {
        return UINT32_MAX;
    }

    /* 正数加 0.5f 后取整，实现四舍五入而不是直接截断。 */
    return (uint32_t) (speed_deg_s + 0.5f);
}

static void GimbalControl_ApplyAxisSpeed(uint8_t id, float speed_deg_s,
    float min_speed_deg_s, float max_speed_deg_s, StepMotorDir *last_dir,
    bool *running)
{
    float abs_speed = GimbalControl_Abs(speed_deg_s);
    StepMotorDir dir;

    if (!StepMotor_IsBusy(id)) {
        *running = false;
    }

    if (abs_speed < min_speed_deg_s) {
        StepMotor_Stop(id);
        *running = false;
        return;
    }

    abs_speed = GimbalControl_Clamp(abs_speed, min_speed_deg_s, max_speed_deg_s);
    /* 0.0f 归入顺时针分支；实际零速已被最小有效速度判断拦截。 */
    dir = (speed_deg_s >= 0.0f) ? STEP_MOTOR_DIR_CW : STEP_MOTOR_DIR_CCW;

    /* 反向前先停 PWM，避免底层忙状态下拒绝修改 DIR。 */
    if ((*running) && (*last_dir != dir)) {
        StepMotor_Stop(id);
        *running = false;
    }

    if ((*running) && (*last_dir == dir)) {
        StepMotor_SetSpeedDeg(id, GimbalControl_ToSpeedUint(abs_speed));
        return;
    }

    StepMotor_SetSpeedDeg(id, GimbalControl_ToSpeedUint(abs_speed));
    StepMotor_RunContinuous(id, dir);
    *last_dir = dir;
    *running = StepMotor_IsBusy(id);
}

static void GimbalControl_UpdateByAngleError(GimbalControl *control,
    float yaw_error_deg, float pitch_error_deg, float dt_sec)
{
    float yaw_speed;
    float pitch_speed;

    /*
     * PID_Update 内部使用 e = target - current。
     * 此处令 target=0、current=-angle_error，因此 e=angle_error；
     * PID 输出 yaw_speed/pitch_speed 的单位为 deg/s。
     */
    yaw_speed = PID_Update(&control->yaw_pid, 0.0f, -yaw_error_deg, dt_sec);
    pitch_speed =
        PID_Update(&control->pitch_pid, 0.0f, -pitch_error_deg, dt_sec);
    GimbalControl_ApplySpeed(control, yaw_speed, pitch_speed);
}

void GimbalControl_Init(GimbalControl *control)
{
    if (control == 0) {
        return;
    }

    /* 三个 0.0f 分别表示 Kp、Ki、Kd 初始均关闭，由 main.c 随后写入。 */
    PID_Init(&control->yaw_pid, 0.0f, 0.0f, 0.0f);
    PID_Init(&control->pitch_pid, 0.0f, 0.0f, 0.0f);
    control->min_effective_speed_deg_s =
        GIMBAL_CONTROL_DEFAULT_MIN_SPEED_DEG_S;
    control->max_speed_deg_s = GIMBAL_CONTROL_DEFAULT_MAX_SPEED_DEG_S;
    PID_SetOutputLimit(
        &control->yaw_pid, -control->max_speed_deg_s, control->max_speed_deg_s);
    PID_SetOutputLimit(&control->pitch_pid, -control->max_speed_deg_s,
        control->max_speed_deg_s);
    control->mode = GIMBAL_CONTROL_MODE_IDLE;
    control->yaw_last_dir = STEP_MOTOR_DIR_CW;
    control->pitch_last_dir = STEP_MOTOR_DIR_CW;
    control->yaw_running = false;
    control->pitch_running = false;
}

void GimbalControl_SetMode(GimbalControl *control, GimbalControlMode mode)
{
    if (control == 0) {
        return;
    }

    if (control->mode != mode) {
        control->mode = mode;
        GimbalControl_Reset(control);
    }
}

void GimbalControl_SetYawPid(
    GimbalControl *control, float kp, float ki, float kd)
{
    if (control == 0) {
        return;
    }

    PID_SetTunings(&control->yaw_pid, kp, ki, kd);
}

void GimbalControl_SetPitchPid(
    GimbalControl *control, float kp, float ki, float kd)
{
    if (control == 0) {
        return;
    }

    PID_SetTunings(&control->pitch_pid, kp, ki, kd);
}

void GimbalControl_SetSpeedLimit(
    GimbalControl *control, float min_speed_deg_s, float max_speed_deg_s)
{
    /* 最大速度必须大于 0.0f，否则该速度区间无效。 */
    if ((control == 0) || (max_speed_deg_s <= 0.0f)) {
        return;
    }

    min_speed_deg_s = GimbalControl_Abs(min_speed_deg_s);
    if (min_speed_deg_s >= max_speed_deg_s) {
        /* 最小值不小于最大值时，以 0.0f 关闭最小有效速度门槛。 */
        min_speed_deg_s = 0.0f;
    }

    control->min_effective_speed_deg_s = min_speed_deg_s;
    control->max_speed_deg_s = max_speed_deg_s;
    PID_SetOutputLimit(&control->yaw_pid, -max_speed_deg_s, max_speed_deg_s);
    PID_SetOutputLimit(&control->pitch_pid, -max_speed_deg_s, max_speed_deg_s);
}

void GimbalControl_SetDeadband(GimbalControl *control, float deadband_deg)
{
    if (control == 0) {
        return;
    }

    PID_SetDeadband(&control->yaw_pid, deadband_deg);
    PID_SetDeadband(&control->pitch_pid, deadband_deg);
}

void GimbalControl_Reset(GimbalControl *control)
{
    if (control == 0) {
        return;
    }

    PID_Reset(&control->yaw_pid);
    PID_Reset(&control->pitch_pid);
}

void GimbalControl_Stop(GimbalControl *control)
{
    if (control == 0) {
        return;
    }

    StepMotor_Stop(STEP_MOTOR_ID_YAW);
    StepMotor_Stop(STEP_MOTOR_ID_PITCH);
    control->yaw_running = false;
    control->pitch_running = false;
    control->mode = GIMBAL_CONTROL_MODE_IDLE;
}

void GimbalControl_ApplySpeed(
    GimbalControl *control, float yaw_speed_deg_s, float pitch_speed_deg_s)
{
    if (control == 0) {
        return;
    }

    GimbalControl_ApplyAxisSpeed(STEP_MOTOR_ID_YAW, yaw_speed_deg_s,
        control->min_effective_speed_deg_s, control->max_speed_deg_s,
        &control->yaw_last_dir, &control->yaw_running);
    GimbalControl_ApplyAxisSpeed(STEP_MOTOR_ID_PITCH, pitch_speed_deg_s,
        control->min_effective_speed_deg_s, control->max_speed_deg_s,
        &control->pitch_last_dir, &control->pitch_running);
}

void GimbalControl_UpdateAngle(GimbalControl *control, GimbalAngle target,
    GimbalAngle current, float dt_sec)
{
    if (control == 0) {
        return;
    }

    GimbalControl_SetMode(control, GIMBAL_CONTROL_MODE_ANGLE);
    GimbalControl_UpdateByAngleError(
        control, target.yaw - current.yaw, target.pitch - current.pitch, dt_sec);
}

void GimbalControl_UpdateVisionTrack(GimbalControl *control,
    GimbalPoint target_px, GimbalPoint image_center_px, float yaw_deg_per_px,
    float pitch_deg_per_px, float dt_sec)
{
    float yaw_error_deg;
    float pitch_error_deg;

    if (control == 0) {
        return;
    }

    GimbalControl_SetMode(control, GIMBAL_CONTROL_MODE_VISION_TRACK);
    if ((!target_px.valid) || (!image_center_px.valid)) {
        /* 两个 0.0f 分别让 Yaw、Pitch 立即停止。 */
        GimbalControl_ApplySpeed(control, 0.0f, 0.0f);
        return;
    }

    /*
     * 图像像素误差转换为角度误差：
     * yaw_error_deg   = (target_x - center_x) * yaw_deg_per_px
     * pitch_error_deg = (center_y - target_y) * pitch_deg_per_px
     * Pitch 使用相反的像素差顺序，是因为图像坐标 Y 轴通常向下为正。
     */
    yaw_error_deg = (target_px.x - image_center_px.x) * yaw_deg_per_px;
    pitch_error_deg = (image_center_px.y - target_px.y) * pitch_deg_per_px;
    GimbalControl_UpdateByAngleError(
        control, yaw_error_deg, pitch_error_deg, dt_sec);
}

void GimbalControl_UpdateAim(GimbalControl *control, GimbalPoint target_px,
    GimbalPoint laser_px, GimbalPoint image_center_px, float yaw_deg_per_px,
    float pitch_deg_per_px, float dt_sec)
{
    GimbalPoint reference_px;
    float yaw_error_deg;
    float pitch_error_deg;

    if (control == 0) {
        return;
    }

    GimbalControl_SetMode(control, GIMBAL_CONTROL_MODE_AIM);
    if (!target_px.valid) {
        /* 两个 0.0f 分别让 Yaw、Pitch 立即停止。 */
        GimbalControl_ApplySpeed(control, 0.0f, 0.0f);
        return;
    }

    reference_px = laser_px.valid ? laser_px : image_center_px;
    if (!reference_px.valid) {
        /* 两个 0.0f 分别让 Yaw、Pitch 立即停止。 */
        GimbalControl_ApplySpeed(control, 0.0f, 0.0f);
        return;
    }

    /*
     * 瞄准模式转换公式与视觉跟踪相同，但 reference 为激光点；
     * 激光点无效时退化为图像中心点：
     * yaw_error_deg   = (target_x - reference_x) * yaw_deg_per_px
     * pitch_error_deg = (reference_y - target_y) * pitch_deg_per_px
     */
    yaw_error_deg = (target_px.x - reference_px.x) * yaw_deg_per_px;
    pitch_error_deg = (reference_px.y - target_px.y) * pitch_deg_per_px;
    GimbalControl_UpdateByAngleError(
        control, yaw_error_deg, pitch_error_deg, dt_sec);
}

void GimbalControl_UpdateImuStabilize(GimbalControl *control,
    GimbalAngle stable_reference, GimbalAngle imu_angle, float dt_sec)
{
    if (control == 0) {
        return;
    }

    GimbalControl_SetMode(control, GIMBAL_CONTROL_MODE_IMU_STABILIZE);
    GimbalControl_UpdateByAngleError(control,
        stable_reference.yaw - imu_angle.yaw,
        stable_reference.pitch - imu_angle.pitch, dt_sec);
}

void GimbalControl_UpdateSpeedSmooth(GimbalControl *control,
    GimbalAngle target_speed, GimbalAngle current_speed, float dt_sec)
{
    float yaw_speed;
    float pitch_speed;

    if (control == 0) {
        return;
    }

    GimbalControl_SetMode(control, GIMBAL_CONTROL_MODE_SPEED_SMOOTH);
    yaw_speed =
        PID_Update(&control->yaw_pid, target_speed.yaw, current_speed.yaw, dt_sec);
    pitch_speed = PID_Update(
        &control->pitch_pid, target_speed.pitch, current_speed.pitch, dt_sec);
    GimbalControl_ApplySpeed(control, yaw_speed, pitch_speed);
}
