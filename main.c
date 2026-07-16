/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* TI includes. */
#include "ti_msp_dl_config.h"

/* Application includes. */
#include "yuntaiqudong/gimbal_control.h"

/*-----------------------------------------------------------*/

/* 512 个 StackType_t；当前 32 位端口下约为 2048 字节任务栈。 */
#define GIMBAL_TASK_STACK_WORDS       (512U)
/* 云台任务优先级比空闲任务高 3 级。 */
#define GIMBAL_TASK_PRIORITY          (tskIDLE_PRIORITY + 3U)
/* 每次等待通知的最长时间为 100 ms，超时后可执行后续安全巡检。 */
#define GIMBAL_TASK_WAIT_MS           (100U)

/* Yaw 顺、逆时针均最多偏离零点 3600°，即各 10 圈。 */
#define YAW_CW_LIMIT_DEG              (3600U)
#define YAW_CCW_LIMIT_DEG             (3600U)
/* Pitch 顺、逆时针均最多偏离零点 360°，即各 1 圈。 */
#define PITCH_CW_LIMIT_DEG            (360U)
#define PITCH_CCW_LIMIT_DEG           (360U)

/*
 * ======================== PID 参数调节区 ========================
 * Kp：比例系数，增大后响应更快，过大会振荡。
 * Ki：积分系数，用于消除稳态误差，过大会产生积分饱和。
 * Kd：微分系数，用于抑制快速变化和超调，过大会放大测量噪声。
 *
 * PID 参数与步进电机转速的关系：
 *   e              = target_angle - current_angle                  [deg]
 *   u              = Kp*e + Ki*∫e dt + Kd*de/dt                  [deg/s]
 *   speed_deg_s    = 0，若 |u| < min_speed_deg_s；否则为
 *                    clamp(|u|, min_speed_deg_s, max_speed_deg_s) [deg/s]
 *   steps_per_sec  = round(speed_deg_s * steps_per_rev / 360)     [step/s]
 *   actual_deg_s   = steps_per_sec * 360 / steps_per_rev          [deg/s]
 *   u 的正负号决定电机转动方向，以上公式使用 |u| 计算速度大小。
 *
 * 当前默认 steps_per_rev = 6400，因此：
 *   steps_per_sec ≈ speed_deg_s * 17.7778
 *   1 deg/s   ≈ 17.78 step/s；60 deg/s  ≈ 1066.67 step/s；
 *   90 deg/s  = 1600 step/s；180 deg/s = 3200 step/s；
 *   360 deg/s = 6400 step/s。
 *
 * 仅使用比例项时 u = Kp*e。例如角度误差 e=10 deg、Kp=2.0，
 * 则 PID 命令约为 20 deg/s，对应约 356 step/s。Ki、Kd 非零时还会
 * 叠加累计误差和误差变化率，不能仅由单个参数直接确定最终转速。
 * 以角度 deg 为输入、角速度 deg/s 为输出时，Kp 单位为 1/s，
 * Ki 单位为 1/s^2，Kd 为无量纲。
 *
 * 定时器还会限制实际最大转速：
 *   max_steps_per_sec = timer_clock_hz / min_period_ticks
 * 当前 min_period_ticks=800：Yaw 约 3125 step/s（175.78 deg/s），
 * Pitch 约 6250 step/s（351.56 deg/s）。即使 PID 输出设为 360 deg/s，
 * 底层也会按各轴定时器能力进行限幅。
 *
 * 当前六个参数均为 0，因此上电后 PID 不会产生转速命令。
 * 实际调参时建议先设置 Kp，再逐步加入 Kd，最后按需要加入 Ki。
 */
#define GIMBAL_YAW_PID_KP             (0.0f)
#define GIMBAL_YAW_PID_KI             (0.0f)
#define GIMBAL_YAW_PID_KD             (0.0f)
#define GIMBAL_PITCH_PID_KP           (0.0f)
#define GIMBAL_PITCH_PID_KI           (0.0f)
#define GIMBAL_PITCH_PID_KD           (0.0f)

static StaticTask_t g_gimbalTaskTcb;
static StackType_t g_gimbalTaskStack[GIMBAL_TASK_STACK_WORDS];
static GimbalControl g_gimbalControl;

static void GimbalEnterSafeState(void)
{
    StepMotor_EStop(STEP_MOTOR_ID_YAW);
    StepMotor_EStop(STEP_MOTOR_ID_PITCH);
}

static void GimbalTask(void *argument)
{
    (void) argument;

    /* SysConfig must leave all motor power outputs inactive at reset. */
    StepMotor_InitAll();
    StepMotor_SetLimitDeg(
        STEP_MOTOR_ID_YAW, STEP_MOTOR_DIR_CW, YAW_CW_LIMIT_DEG);
    StepMotor_SetLimitDeg(
        STEP_MOTOR_ID_YAW, STEP_MOTOR_DIR_CCW, YAW_CCW_LIMIT_DEG);
    StepMotor_SetLimitDeg(
        STEP_MOTOR_ID_PITCH, STEP_MOTOR_DIR_CW, PITCH_CW_LIMIT_DEG);
    StepMotor_SetLimitDeg(
        STEP_MOTOR_ID_PITCH, STEP_MOTOR_DIR_CCW, PITCH_CCW_LIMIT_DEG);
    GimbalControl_Init(&g_gimbalControl);
    /* 将上方 PID 调参区的参数写入 Yaw/Pitch 两个控制器。 */
    GimbalControl_SetYawPid(&g_gimbalControl, GIMBAL_YAW_PID_KP,
        GIMBAL_YAW_PID_KI, GIMBAL_YAW_PID_KD);
    GimbalControl_SetPitchPid(&g_gimbalControl, GIMBAL_PITCH_PID_KP,
        GIMBAL_PITCH_PID_KI, GIMBAL_PITCH_PID_KD);
    GimbalControl_Stop(&g_gimbalControl);

    /*
     * The command path will notify this task after communication and safety
     * checks are integrated. A finite wait leaves room for future supervision.
     */
    for (;;) {
        (void) ulTaskNotifyTake(
            pdTRUE, pdMS_TO_TICKS(GIMBAL_TASK_WAIT_MS));
    }
}

/*-----------------------------------------------------------*/

int main(void)
{
    TaskHandle_t gimbalTaskHandle;

    SYSCFG_DL_init();

    gimbalTaskHandle = xTaskCreateStatic(GimbalTask, "Gimbal",
        GIMBAL_TASK_STACK_WORDS, NULL, GIMBAL_TASK_PRIORITY,
        g_gimbalTaskStack, &g_gimbalTaskTcb);
    if (gimbalTaskHandle == NULL) {
        GimbalEnterSafeState();
        for (;;) {
        }
    }

    vTaskStartScheduler();

    /* The scheduler only returns when it cannot be started. */
    GimbalEnterSafeState();
    for (;;) {
    }
}

/*-----------------------------------------------------------*/

#if (configCHECK_FOR_STACK_OVERFLOW)
/*
 *  ======== vApplicationStackOverflowHook ========
 *  The default weak hook stops execution when a task stack overflows.
 */
#if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(
    TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__TI_COMPILER_VERSION__))
#pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__GNUC__) || defined(__ti_version__))
void __attribute__((weak))
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#endif
{
    (void) pxTask;
    (void) pcTaskName;

    taskDISABLE_INTERRUPTS();
    GimbalEnterSafeState();
    /* 1 表示条件恒真：栈溢出后永久停机，等待复位。 */
    while (1) {
    }
}
#endif

/*-----------------------------------------------------------*/
