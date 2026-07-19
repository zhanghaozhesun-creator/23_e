#include "gimbal_control.h"

#include <math.h>
#include <stdint.h>

/*
 * 本文件的通用数值约定：指针与 0 比较表示空指针检查；速度 0.0f
 * 表示停机命令；有符号速度大于等于 0 表示顺时针，小于 0 表示逆时针。
 */
/* 死区外的非零命令至少提升到 1 deg/s，保证步进电机能够稳定执行。 */
#define GIMBAL_CONTROL_DEFAULT_MIN_SPEED_DEG_S   (1.0f)
/* 1 MHz 定时器、800 tick 最短周期下的可靠软件上限取 70 deg/s。 */
#define GIMBAL_CONTROL_DEFAULT_MAX_SPEED_DEG_S   (70.0f)
/* 进入此范围时停轴；停轴后超过启动阈值才重新动作，避免中心附近抖动。 */
#define GIMBAL_CONTROL_DEFAULT_STOP_DEADBAND_DEG  (0.15f)
#define GIMBAL_CONTROL_DEFAULT_START_DEADBAND_DEG (0.25f)
/* 激光笔到目标平面的水平距离固定按 1050 mm（1.05 m）计算。 */
#define GIMBAL_CONTROL_TARGET_DISTANCE_MM        (1050.0f)
/* 弧度乘以 180/π 后转换为角度。 */
#define GIMBAL_CONTROL_RAD_TO_DEG                (57.2957795f)

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

static float GimbalControl_GetActiveError(float error_deg, bool running,
    float stop_deadband_deg, float start_deadband_deg)
{
    float threshold = running ? stop_deadband_deg : start_deadband_deg;

    if (GimbalControl_Abs(error_deg) <= threshold) {
        return 0.0f;
    }

    return error_deg;
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

    if (abs_speed <= 0.0f) {
        StepMotor_Stop(id);
        *running = false;
        return;
    }

    /* 死区外的非零命令至少提升到电机可稳定执行的 1 deg/s。 */
    abs_speed = GimbalControl_Clamp(abs_speed, min_speed_deg_s, max_speed_deg_s);
    /* 零速已在上方处理，此处只根据符号选择实际运动方向。 */
    dir = (speed_deg_s >= 0.0f) ? STEP_MOTOR_DIR_CW : STEP_MOTOR_DIR_CCW;

    /* 反向前先停 PWM，避免底层忙状态下拒绝修改 DIR。 */
    if ((*running) && (*last_dir != dir)) {
        StepMotor_Stop(id);
        *running = false;
    }

    if ((*running) && (*last_dir == dir)) {
        StepMotor_SetSpeedDeg(id, abs_speed);
        return;
    }

    StepMotor_SetSpeedDeg(id, abs_speed);
    StepMotor_RunContinuous(id, dir);
    *last_dir = dir;
    *running = StepMotor_IsBusy(id);
}

static void GimbalControl_UpdateByAngleError(GimbalControl *control,
    float yaw_error_deg, float pitch_error_deg, float dt_sec)
{
    float active_yaw_error_deg;
    float active_pitch_error_deg;
    float yaw_speed;
    float pitch_speed;

    /*
     * PID_Update 内部使用 e = target - current。
     * 此处令 target=0、current=-angle_error，因此 e=angle_error；
     * PID 输出 yaw_speed/pitch_speed 的单位为 deg/s。
     */
    /* 保存死区处理前的两轴角度误差，供 UART_PC 实时遥测。 */
    control->angle_error.yaw = yaw_error_deg;
    control->angle_error.pitch = pitch_error_deg;
    active_yaw_error_deg = GimbalControl_GetActiveError(yaw_error_deg,
        control->yaw_running, control->stop_deadband_deg,
        control->start_deadband_deg);
    active_pitch_error_deg = GimbalControl_GetActiveError(pitch_error_deg,
        control->pitch_running, control->stop_deadband_deg,
        control->start_deadband_deg);

    if (active_yaw_error_deg == 0.0f) {
        PID_Reset(&control->yaw_pid);
        yaw_speed = 0.0f;
    } else {
        yaw_speed = PID_Update(
            &control->yaw_pid, 0.0f, -active_yaw_error_deg, dt_sec);
    }

    if (active_pitch_error_deg == 0.0f) {
        PID_Reset(&control->pitch_pid);
        pitch_speed = 0.0f;
    } else {
        pitch_speed = PID_Update(
            &control->pitch_pid, 0.0f, -active_pitch_error_deg, dt_sec);
    }

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
    /* 启停迟滞在云台层处理，通用 PID 内部死区保持为 0。 */
    PID_SetDeadband(&control->yaw_pid, 0.0f);
    PID_SetDeadband(&control->pitch_pid, 0.0f);
    /* 两个 0.0f 表示上电后尚未获得 Yaw/Pitch 角度误差。 */
    control->angle_error.yaw = 0.0f;
    control->angle_error.pitch = 0.0f;
    control->min_effective_speed_deg_s =
        GIMBAL_CONTROL_DEFAULT_MIN_SPEED_DEG_S;
    control->max_speed_deg_s = GIMBAL_CONTROL_DEFAULT_MAX_SPEED_DEG_S;
    control->stop_deadband_deg =
        GIMBAL_CONTROL_DEFAULT_STOP_DEADBAND_DEG;
    control->start_deadband_deg =
        GIMBAL_CONTROL_DEFAULT_START_DEADBAND_DEG;
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
    GimbalControl_SetDeadbandHysteresis(
        control, deadband_deg, deadband_deg);
}

void GimbalControl_SetDeadbandHysteresis(GimbalControl *control,
    float stop_deadband_deg, float start_deadband_deg)
{
    if (control == 0) {
        return;
    }

    stop_deadband_deg = GimbalControl_Abs(stop_deadband_deg);
    start_deadband_deg = GimbalControl_Abs(start_deadband_deg);
    if (start_deadband_deg < stop_deadband_deg) {
        start_deadband_deg = stop_deadband_deg;
    }

    control->stop_deadband_deg = stop_deadband_deg;
    control->start_deadband_deg = start_deadband_deg;
    PID_SetDeadband(&control->yaw_pid, 0.0f);
    PID_SetDeadband(&control->pitch_pid, 0.0f);
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

GimbalAngle GimbalControl_GetAngleError(const GimbalControl *control)
{
    GimbalAngle error;

    /* 控制对象无效时，两个通道均返回 0.0°。 */
    error.yaw = 0.0f;
    error.pitch = 0.0f;
    if (control != 0) {
        error = control->angle_error;
    }

    return error;
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

void GimbalControl_UpdateAimDistanceError(GimbalControl *control,
    float error_x_mm, float error_y_mm, float dt_sec)
{
    float yaw_error_deg;
    float pitch_error_deg;

    if (control == 0) {
        return;
    }

    GimbalControl_SetMode(control, GIMBAL_CONTROL_MODE_AIM);

    /*
     * MaixCAM2 误差定义为“目标点 - 激光点”，目标平面距离为 1050 mm：
     *   yaw_error   = atan2(error_x_mm, 1050) * 180/π
     *   pitch_error = -atan2(error_y_mm, 1050) * 180/π
     * 在此处修改pitch和yaw两个轴的正反转方向
     */
    yaw_error_deg =
        atan2f(error_x_mm, GIMBAL_CONTROL_TARGET_DISTANCE_MM) *
        GIMBAL_CONTROL_RAD_TO_DEG;
    pitch_error_deg =
        atan2f(error_y_mm, GIMBAL_CONTROL_TARGET_DISTANCE_MM) *
        GIMBAL_CONTROL_RAD_TO_DEG;

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
