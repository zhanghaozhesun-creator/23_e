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
#include "queue.h"
#include "task.h"

/* Standard C includes. */
#include <stdio.h>

/* TI includes. */
#include "ti_msp_dl_config.h"

/* Application includes. */
#include "yuntaiqudong/gimbal_control.h"

/*-----------------------------------------------------------*/

/* 512 个 StackType_t；当前 32 位端口下约为 2048 字节任务栈。 */
#define GIMBAL_TASK_STACK_WORDS       (512U)
/* 云台任务优先级比空闲任务高 3 级。 */
#define GIMBAL_TASK_PRIORITY          (tskIDLE_PRIORITY + 3U)
/* 自动跟踪控制周期为 2 ms，对应 500 Hz。 */
#define GIMBAL_CONTROL_PERIOD_MS      (2U)
/* 2 ms 换算为 PID 使用的时间步长是 0.002 s。 */
#define GIMBAL_CONTROL_DT_SEC         (0.002f)
/* 连续 50 ms 没有合法视觉误差时停止两轴，禁止继续使用旧误差。 */
#define VISION_TRACK_TIMEOUT_MS       (50U)
/* 电脑手动控制时，两轴均使用 1 deg/s 的顺时针速度命令。 */
#define PC_MANUAL_SPEED_DEG_S         (1.0f)
/* UART_PC 接收 ASCII 字符 '1'：Yaw 轴顺时针运动。 */
#define PC_COMMAND_YAW_CW             ('1')
/* UART_PC 接收 ASCII 字符 '2'：Pitch 轴顺时针运动。 */
#define PC_COMMAND_PITCH_CW           ('2')
/* UART_PC 接收 ASCII 字符 '3'：停止两轴。 */
#define PC_COMMAND_STOP_ALL           ('3')
/* UART_PC 接收 ASCII 字符 '4'：执行通信就绪检查。 */
#define PC_COMMAND_CHECK_READY        ('4')
/* UART_PC 接收 ASCII 字符 'a'：启动自动跟 踪。 */
#define PC_COMMAND_AUTO_TRACK         ('a')

/* VOFA+ FireWater 遥测任务使用 512 个栈字，当前端口下约 2048 字节。 */
#define TELEMETRY_TASK_STACK_WORDS    (512U)
/* 遥测优先级比云台控制任务低 1 级。 */
#define TELEMETRY_TASK_PRIORITY       (tskIDLE_PRIORITY + 2U)
/* 50 帧/s 对应每 20 ms 发送一帧。 */
#define TELEMETRY_PERIOD_MS           (20U)
/* FireWater 文本帧最多使用 64 字节。 */
#define TELEMETRY_FRAME_BUFFER_SIZE   (64U)
/* 单条电脑命令回复最多使用 96 字节。 */
#define PC_REPLY_FRAME_BUFFER_SIZE    (96U)
/* UART_PC 独立发送任务使用 256 个栈字，当前端口下约为 1024 字节。 */
#define UART_PC_TX_TASK_STACK_WORDS   (256U)
/* 发送任务低于控制和遥测任务，避免轮询 TX 状态影响 500 Hz 控制。 */
#define UART_PC_TX_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)
/* 静态发送队列最多缓存 8 个完整文本帧。 */
#define UART_PC_TX_QUEUE_DEPTH        (8U)
/* 单个队列项按最长电脑命令回复预留 96 字节。 */
#define UART_PC_TX_FRAME_MAX_BYTES    (PC_REPLY_FRAME_BUFFER_SIZE)
/* TX 连续 20 ms 没有成功写入新字节时判定 UART 异常。 */
#define UART_PC_TX_TIMEOUT_MS         (20U)

/* 视觉串口固定帧的第 1 个帧头字节为 0xAA。 */
#define VISION_FRAME_HEADER_FIRST     (0xAAU)
/* 视觉串口固定帧的第 2 个帧头字节为 0x55。 */
#define VISION_FRAME_HEADER_SECOND    (0x55U)
/* 每个视觉数据帧固定包含 8 字节。 */
#define VISION_FRAME_LENGTH           (8U)
/* control 的 bit0 为误差有效标志。 */
#define VISION_CONTROL_VALID_MASK     (0x01U)
/* control 的 bit1 为停止标志。 */
#define VISION_CONTROL_STOP_MASK      (0x02U)
/* error_x/error_y 每个原始计数代表 0.1 mm。 */
#define VISION_ERROR_SCALE_MM         (0.1f)
/* 自动跟踪状态 0 表示尚未收到可供控制使用的新帧。 */
#define VISION_TRACK_STATE_NONE       (0U)
/* 自动跟踪状态 1 表示最新正确帧包含有效误差。 */
#define VISION_TRACK_STATE_VALID      (1U)
/* 自动跟踪状态 2 表示最新正确帧要求停止或误差无效。 */
#define VISION_TRACK_STATE_NO_DATA    (2U)

typedef struct {
    uint16_t length;
    uint8_t data[UART_PC_TX_FRAME_MAX_BYTES];
} UartPcTxFrame;

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
 *   speed_deg_s    = 0，若误差进入停止死区；否则为
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
 * 两轴定时器均配置为 1 MHz；定时器还会限制实际最大转速：
 *   max_steps_per_sec = timer_clock_hz / min_period_ticks
 * 当前 min_period_ticks=800：两轴约 1250 step/s（70.31 deg/s）。
 * 1 deg/s 对应 18 step/s、周期约 55556 tick，未超过 16 位上限。
 *
 * 当前两轴 Kp 均为 1.0，Ki、Kd 均为 0.0；实际调参时建议先调整 Kp，
 * 再逐步加入 Kd，最后按需要加入 Ki。
 */
#define GIMBAL_YAW_PID_KP             (1.0f)
#define GIMBAL_YAW_PID_KI             (0.0f)
#define GIMBAL_YAW_PID_KD             (0.0f)
#define GIMBAL_PITCH_PID_KP           (1.0f)
#define GIMBAL_PITCH_PID_KI           (0.0f)
#define GIMBAL_PITCH_PID_KD           (0.0f)

static StaticTask_t g_gimbalTaskTcb;
static StackType_t g_gimbalTaskStack[GIMBAL_TASK_STACK_WORDS];
static StaticTask_t g_telemetryTaskTcb;
static StackType_t g_telemetryTaskStack[TELEMETRY_TASK_STACK_WORDS];
static StaticTask_t g_uartPcTxTaskTcb;
static StackType_t g_uartPcTxTaskStack[UART_PC_TX_TASK_STACK_WORDS];
static StaticQueue_t g_uartPcTxQueueBuffer;
static uint8_t g_uartPcTxQueueStorage[
    UART_PC_TX_QUEUE_DEPTH * sizeof(UartPcTxFrame)];
static QueueHandle_t g_uartPcTxQueue;
static TaskHandle_t g_gimbalTaskHandle;
static GimbalControl g_gimbalControl;
static uint8_t g_visionFrame[VISION_FRAME_LENGTH];
static uint8_t g_visionFrameIndex;
/* 以下标志由 UART_VISION 中断置位，由遥测任务每 20 ms 读取并清零。 */
static volatile uint8_t g_visionBytesSinceReport;
static volatile uint8_t g_visionValidSinceReport;
static volatile uint8_t g_visionWrongSinceReport;
static volatile uint8_t g_visionNoMeasurementSinceReport;
static volatile int16_t g_visionErrorXRaw;
static volatile int16_t g_visionErrorYRaw;
/* 每收到一个格式正确的视觉帧就递增，用于把最新状态交给控制任务。 */
static volatile uint32_t g_visionTrackingUpdateCount;
static volatile uint8_t g_visionTrackingState;
/* valid=0 或 stop=1 时置位，直到控制任务确实执行一次停轴。 */
static volatile uint8_t g_visionTrackingStopPending;
static bool g_autoTrackingEnabled;
/* 供调试器观察：队列已满或帧过长时累计丢弃帧数。 */
static volatile uint32_t g_uartPcTxDroppedFrameCount;
/* 供调试器观察：UART 连续 20 ms 无发送进展的累计次数。 */
static volatile uint32_t g_uartPcTxTimeoutCount;

static void UartPcReplyCommand(uint8_t command, const char *action);
static void UartPcReplyReady(void);

static void VisionProtocolFinishFrame(void)
{
    uint8_t checksum;
    uint8_t control;

    /* XOR 只覆盖 control 和两轴误差，即帧中的第 2～6 号字节。 */
    checksum = g_visionFrame[2] ^ g_visionFrame[3] ^
        g_visionFrame[4] ^ g_visionFrame[5] ^ g_visionFrame[6];
    if (checksum != g_visionFrame[7]) {
        g_visionWrongSinceReport = 1U;
        return;
    }

    control = g_visionFrame[2];
    if (((control & VISION_CONTROL_VALID_MASK) == 0U) ||
        ((control & VISION_CONTROL_STOP_MASK) != 0U)) {
        /* 帧格式正确，但视觉误差无效或要求停止，本周期按 nodadata 上报。 */
        g_visionNoMeasurementSinceReport = 1U;
        g_visionTrackingState = VISION_TRACK_STATE_NO_DATA;
        g_visionTrackingStopPending = 1U;
        g_visionTrackingUpdateCount++;
        return;
    }

    /* X、Y 都是低字节在前的有符号 16 位整数，原始单位为 0.1 mm。 */
    g_visionErrorXRaw = (int16_t) ((uint16_t) g_visionFrame[3] |
        ((uint16_t) g_visionFrame[4] << 8U));
    g_visionErrorYRaw = (int16_t) ((uint16_t) g_visionFrame[5] |
        ((uint16_t) g_visionFrame[6] << 8U));
    g_visionValidSinceReport = 1U;
    g_visionTrackingState = VISION_TRACK_STATE_VALID;
    g_visionTrackingUpdateCount++;
}

static void VisionProtocolProcessByte(uint8_t received_byte)
{
    g_visionBytesSinceReport = 1U;

    if (g_visionFrameIndex == 0U) {
        if (received_byte == VISION_FRAME_HEADER_FIRST) {
            g_visionFrame[0] = received_byte;
            g_visionFrameIndex = 1U;
        } else {
            g_visionWrongSinceReport = 1U;
        }
        return;
    }

    if (g_visionFrameIndex == 1U) {
        if (received_byte == VISION_FRAME_HEADER_SECOND) {
            g_visionFrame[1] = received_byte;
            g_visionFrameIndex = 2U;
        } else {
            g_visionWrongSinceReport = 1U;
            if (received_byte == VISION_FRAME_HEADER_FIRST) {
                /* 连续出现 0xAA 时，把最新字节作为下一帧的起点。 */
                g_visionFrame[0] = received_byte;
                g_visionFrameIndex = 1U;
            } else {
                g_visionFrameIndex = 0U;
            }
        }
        return;
    }

    g_visionFrame[g_visionFrameIndex] = received_byte;
    g_visionFrameIndex++;
    if (g_visionFrameIndex >= VISION_FRAME_LENGTH) {
        VisionProtocolFinishFrame();
        g_visionFrameIndex = 0U;
    }
}

static void GimbalEnterSafeState(void)
{
    StepMotor_EStop(STEP_MOTOR_ID_YAW);
    StepMotor_EStop(STEP_MOTOR_ID_PITCH);
}

static void GimbalHandlePcCommand(uint8_t command)
{
    switch (command) {
    case PC_COMMAND_YAW_CW:
        /* 手动速度命令接管控制权，防止自动 PID 在下一周期覆盖该命令。 */
        g_autoTrackingEnabled = false;
        /* Yaw 以 +1 deg/s 顺时针运行，同时停止 Pitch。 */
        GimbalControl_ApplySpeed(
            &g_gimbalControl, PC_MANUAL_SPEED_DEG_S, 0.0f);
        UartPcReplyCommand(command, "YAW_CW");
        break;

    case PC_COMMAND_PITCH_CW:
        /* 手动速度命令接管控制权，防止自动 PID 在下一周期覆盖该命令。 */
        g_autoTrackingEnabled = false;
        /* Pitch 以 +1 deg/s 顺时针运行，同时停止 Yaw。 */
        GimbalControl_ApplySpeed(
            &g_gimbalControl, 0.0f, PC_MANUAL_SPEED_DEG_S);
        UartPcReplyCommand(command, "PITCH_CW");
        break;

    case PC_COMMAND_STOP_ALL:
        /* 字符 '3' 退出自动跟踪，并无条件停止两个轴。 */
        g_autoTrackingEnabled = false;
        GimbalControl_Stop(&g_gimbalControl);
        GimbalControl_Reset(&g_gimbalControl);
        UartPcReplyCommand(command, "STOP_ALL");
        break;

    case PC_COMMAND_AUTO_TRACK:
        /* 进入自动模式后先停轴并清空 PID，等待下一帧合法视觉误差。 */
        g_autoTrackingEnabled = true;
        GimbalControl_Stop(&g_gimbalControl);
        GimbalControl_Reset(&g_gimbalControl);
        UartPcReplyCommand(command, "AUTO_TRACK");
        break;

    case PC_COMMAND_CHECK_READY:
        /* 字符 '4' 只检查 UART_PC 通信，不改变控制模式或电机状态。 */
        UartPcReplyReady();
        break;

    default:
        /* 非 '1'、'2'、'3'、'4'、'a' 字符不改变当前运动状态。 */
        break;
    }
}

static void UartPcSendBuffer(const char *buffer, uint32_t length)
{
    UartPcTxFrame frame;
    uint32_t index;

    if ((buffer == NULL) || (length == 0U) ||
        (length > UART_PC_TX_FRAME_MAX_BYTES) ||
        (g_uartPcTxQueue == NULL)) {
        g_uartPcTxDroppedFrameCount++;
        return;
    }

    frame.length = (uint16_t) length;
    for (index = 0U; index < length; index++) {
        frame.data[index] = (uint8_t) buffer[index];
    }

    /* 发送方只复制入队，队列满时丢帧但绝不阻塞控制或遥测任务。 */
    if (xQueueSend(g_uartPcTxQueue, &frame, 0U) != pdPASS) {
        g_uartPcTxDroppedFrameCount++;
    }
}

static void UartPcTxTask(void *argument)
{
    UartPcTxFrame frame;
    TickType_t last_progress_time;
    TickType_t current_time;
    uint32_t index;

    (void) argument;

    for (;;) {
        if (xQueueReceive(
                g_uartPcTxQueue, &frame, portMAX_DELAY) != pdPASS) {
            continue;
        }

        index = 0U;
        last_progress_time = xTaskGetTickCount();
        while (index < (uint32_t) frame.length) {
            if (DL_UART_Main_transmitDataCheck(
                    UART_PC_INST, frame.data[index])) {
                index++;
                last_progress_time = xTaskGetTickCount();
                continue;
            }

            current_time = xTaskGetTickCount();
            if ((current_time - last_progress_time) >=
                pdMS_TO_TICKS(UART_PC_TX_TIMEOUT_MS)) {
                /* 超时后丢弃当前帧并重新使能 UART，后续队列仍可继续运行。 */
                g_uartPcTxTimeoutCount++;
                DL_UART_Main_disable(UART_PC_INST);
                DL_UART_Main_enable(UART_PC_INST);
                break;
            }

            /* 非阻塞检查失败时主动让出 CPU，等待硬件移出 TX 数据。 */
            taskYIELD();
        }
    }
}

static void UartPcReplyReady(void)
{
    static const char ready_reply[] = "ready\n";

    /* sizeof 包含末尾 1 个 '\0'，发送长度需减去该终止符。 */
    UartPcSendBuffer(
        ready_reply, (uint32_t) (sizeof(ready_reply) - 1U));
}

static const char *StepMotorStateToText(StepMotorState state)
{
    switch (state) {
    case STEP_MOTOR_STATE_IDLE:
        return "IDLE";
    case STEP_MOTOR_STATE_RUNNING:
        return "RUNNING";
    case STEP_MOTOR_STATE_MOVING:
        return "MOVING";
    case STEP_MOTOR_STATE_ACCEL:
        return "ACCEL";
    case STEP_MOTOR_STATE_DECEL:
        return "DECEL";
    case STEP_MOTOR_STATE_STOPPING:
        return "STOPPING";
    case STEP_MOTOR_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

static void UartPcReplyCommand(uint8_t command, const char *action)
{
    char reply[PC_REPLY_FRAME_BUFFER_SIZE];
    int reply_length;

    /*
     * 每个合法命令只在执行后调用一次本函数。回复同时包含原命令、
     * 命令动作，以及 Yaw/Pitch 两轴执行后的实际状态。
     */
    reply_length = snprintf(reply, sizeof(reply),
        "reply:cmd=%c,action=%s,yaw=%s,pitch=%s\n", (int) command, action,
        StepMotorStateToText(StepMotor_GetState(STEP_MOTOR_ID_YAW)),
        StepMotorStateToText(StepMotor_GetState(STEP_MOTOR_ID_PITCH)));
    if ((reply_length > 0) &&
        ((uint32_t) reply_length < (uint32_t) sizeof(reply))) {
        UartPcSendBuffer(reply, (uint32_t) reply_length);
    }
}

static void TelemetryTask(void *argument)
{
    TickType_t last_wake_time;
    int16_t error_x_raw;
    int16_t error_y_raw;
    uint8_t bytes_received;
    uint8_t valid_received;
    uint8_t wrong_received;
    uint8_t no_measurement_received;
    char frame[TELEMETRY_FRAME_BUFFER_SIZE];
    int frame_length;

    (void) argument;
    last_wake_time = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(
            &last_wake_time, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));

        /* 关中断时间只覆盖状态快照，避免与 UART_VISION ISR 同时读写。 */
        taskENTER_CRITICAL();
        bytes_received = g_visionBytesSinceReport;
        valid_received = g_visionValidSinceReport;
        wrong_received = g_visionWrongSinceReport;
        no_measurement_received = g_visionNoMeasurementSinceReport;
        error_x_raw = g_visionErrorXRaw;
        error_y_raw = g_visionErrorYRaw;
        g_visionBytesSinceReport = 0U;
        g_visionValidSinceReport = 0U;
        g_visionWrongSinceReport = 0U;
        g_visionNoMeasurementSinceReport = 0U;
        taskEXIT_CRITICAL();

        /*
         * VOFA+ FireWater 帧格式：errors:ch0,ch1\n
         * 通道 0：视觉给出的水平方向 X 距离误差，单位 mm。
         * 通道 1：视觉给出的竖直方向 Y 距离误差，单位 mm。
         * 换行符 \n 是 FireWater 判定一帧结束的必要标志。
         */
        if (valid_received != 0U) {
            frame_length = snprintf(frame, sizeof(frame),
                "errors:%.1f,%.1f\n",
                (double) ((float) error_x_raw * VISION_ERROR_SCALE_MM),
                (double) ((float) error_y_raw * VISION_ERROR_SCALE_MM));
        } else if ((wrong_received != 0U) ||
            ((bytes_received != 0U) && (no_measurement_received == 0U))) {
            /* 本周期收到过字节，但没有得到格式正确的完整数据帧。 */
            frame_length = snprintf(frame, sizeof(frame), "wrongdata\n");
        } else {
            /* 本周期没有数据，或正确数据帧明确表示误差无效/停止。 */
            frame_length = snprintf(frame, sizeof(frame), "nodadata\n");
        }

        if ((frame_length > 0) &&
            ((uint32_t) frame_length < (uint32_t) sizeof(frame))) {
            UartPcSendBuffer(frame, (uint32_t) frame_length);
        }
    }
}

static void GimbalTask(void *argument)
{
    TickType_t current_time;
    TickType_t last_valid_time;
    TickType_t last_wake_time;
    uint32_t pc_command;
    uint32_t tracking_update_count;
    uint32_t seen_tracking_update_count = 0U;
    int16_t tracking_error_x_raw = 0;
    int16_t tracking_error_y_raw = 0;
    uint8_t tracking_state;
    uint8_t stop_pending;
    bool tracking_error_available = false;

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

    /* UART_PC 使用 UART1；优先级 1，随后开启其 NVIC 中断入口。 */
    NVIC_SetPriority(UART_PC_INST_INT_IRQN, 1U);
    NVIC_EnableIRQ(UART_PC_INST_INT_IRQN);
    /* UART_VISION 使用 UART0；SysConfig 已开启 RX 源，此处开启 NVIC 入口。 */
    NVIC_SetPriority(UART_VISION_INST_INT_IRQN, 1U);
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);

    last_wake_time = xTaskGetTickCount();
    last_valid_time = last_wake_time;

    for (;;) {
        if (xTaskNotifyWait(0U, 0U, &pc_command, 0U) == pdTRUE) {
            GimbalHandlePcCommand((uint8_t) pc_command);

            if ((pc_command == (uint32_t) PC_COMMAND_YAW_CW) ||
                (pc_command == (uint32_t) PC_COMMAND_PITCH_CW) ||
                (pc_command == (uint32_t) PC_COMMAND_STOP_ALL) ||
                (pc_command == (uint32_t) PC_COMMAND_AUTO_TRACK)) {
                /* 任一模式命令都废弃自动控制任务中缓存的旧误差。 */
                tracking_error_available = false;
            }

            if (pc_command == (uint32_t) PC_COMMAND_AUTO_TRACK) {
                /* 启动点只接受 a 命令之后到达的新视觉帧。 */
                taskENTER_CRITICAL();
                seen_tracking_update_count = g_visionTrackingUpdateCount;
                g_visionTrackingStopPending = 0U;
                taskEXIT_CRITICAL();
                last_valid_time = xTaskGetTickCount();
            }
        }

        current_time = xTaskGetTickCount();
        if (g_autoTrackingEnabled) {
            /* 原子取得最新视觉控制状态；停轴请求读取后由本任务消费。 */
            taskENTER_CRITICAL();
            tracking_update_count = g_visionTrackingUpdateCount;
            tracking_state = g_visionTrackingState;
            stop_pending = g_visionTrackingStopPending;
            tracking_error_x_raw = g_visionErrorXRaw;
            tracking_error_y_raw = g_visionErrorYRaw;
            g_visionTrackingStopPending = 0U;
            taskEXIT_CRITICAL();

            if (stop_pending != 0U) {
                /* valid=0 或 stop=1 必须先停轴，不能被同批次有效帧覆盖。 */
                seen_tracking_update_count = tracking_update_count;
                tracking_error_available = false;
                GimbalControl_Stop(&g_gimbalControl);
                GimbalControl_Reset(&g_gimbalControl);
            } else if (tracking_update_count != seen_tracking_update_count) {
                seen_tracking_update_count = tracking_update_count;
                if (tracking_state == VISION_TRACK_STATE_VALID) {
                    tracking_error_available = true;
                    last_valid_time = current_time;
                } else {
                    tracking_error_available = false;
                    GimbalControl_Stop(&g_gimbalControl);
                    GimbalControl_Reset(&g_gimbalControl);
                }
            }

            if (tracking_error_available) {
                if ((current_time - last_valid_time) <
                    pdMS_TO_TICKS(VISION_TRACK_TIMEOUT_MS)) {
                    /* 0.1 mm 原始误差转成 mm，再按 1.05 m 距离转角度进入 PID。 */
                    GimbalControl_UpdateAimDistanceError(&g_gimbalControl,
                        (float) tracking_error_x_raw * VISION_ERROR_SCALE_MM,
                        (float) tracking_error_y_raw * VISION_ERROR_SCALE_MM,
                        GIMBAL_CONTROL_DT_SEC);
                } else {
                    /* 超过 50 ms 没有合法新帧，停止并清空 PID 旧状态。 */
                    tracking_error_available = false;
                    GimbalControl_Stop(&g_gimbalControl);
                    GimbalControl_Reset(&g_gimbalControl);
                }
            }
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(GIMBAL_CONTROL_PERIOD_MS));
    }
}

/*-----------------------------------------------------------*/

int main(void)
{
    TaskHandle_t telemetry_task_handle;
    TaskHandle_t uart_pc_tx_task_handle;

    SYSCFG_DL_init();

    /* 命令回复和 50 Hz 遥测只入队，由 UART_PC TX 任务顺序发送。 */
    g_uartPcTxQueue = xQueueCreateStatic(UART_PC_TX_QUEUE_DEPTH,
        sizeof(UartPcTxFrame), g_uartPcTxQueueStorage,
        &g_uartPcTxQueueBuffer);

    g_gimbalTaskHandle = xTaskCreateStatic(GimbalTask, "Gimbal",
        GIMBAL_TASK_STACK_WORDS, NULL, GIMBAL_TASK_PRIORITY,
        g_gimbalTaskStack, &g_gimbalTaskTcb);
    telemetry_task_handle = xTaskCreateStatic(TelemetryTask, "Telemetry",
        TELEMETRY_TASK_STACK_WORDS, NULL, TELEMETRY_TASK_PRIORITY,
        g_telemetryTaskStack, &g_telemetryTaskTcb);
    uart_pc_tx_task_handle = xTaskCreateStatic(UartPcTxTask, "UartPcTx",
        UART_PC_TX_TASK_STACK_WORDS, NULL, UART_PC_TX_TASK_PRIORITY,
        g_uartPcTxTaskStack, &g_uartPcTxTaskTcb);
    if ((g_uartPcTxQueue == NULL) || (g_gimbalTaskHandle == NULL) ||
        (telemetry_task_handle == NULL) ||
        (uart_pc_tx_task_handle == NULL)) {
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

void UART_PC_INST_IRQHandler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    BaseType_t command_received = pdFALSE;
    uint8_t received_byte;
    uint32_t received_command = 0U;

    switch (DL_UART_Main_getPendingInterrupt(UART_PC_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        /* 一次读空 RX FIFO，连续到达时以最后一个合法命令为准。 */
        while (!DL_UART_Main_isRXFIFOEmpty(UART_PC_INST)) {
            received_byte = DL_UART_Main_receiveData(UART_PC_INST);
            if ((received_byte == PC_COMMAND_YAW_CW) ||
                (received_byte == PC_COMMAND_PITCH_CW) ||
                (received_byte == PC_COMMAND_STOP_ALL) ||
                (received_byte == PC_COMMAND_CHECK_READY) ||
                (received_byte == PC_COMMAND_AUTO_TRACK)) {
                received_command = received_byte;
                command_received = pdTRUE;
            }
        }

        if ((command_received == pdTRUE) && (g_gimbalTaskHandle != NULL)) {
            /* 覆盖尚未处理的旧命令，保证任务执行 FIFO 中最新的合法命令。 */
            (void) xTaskNotifyFromISR(g_gimbalTaskHandle, received_command,
                eSetValueWithOverwrite, &higher_priority_task_woken);
        }
        break;

    default:
        break;
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/*-----------------------------------------------------------*/

void UART_VISION_INST_IRQHandler(void)
{
    uint8_t received_byte;

    switch (DL_UART_Main_getPendingInterrupt(UART_VISION_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        /* 一次读空 RX FIFO，每个字节依次交给固定 8 字节帧解析器。 */
        while (!DL_UART_Main_isRXFIFOEmpty(UART_VISION_INST)) {
            received_byte = DL_UART_Main_receiveData(UART_VISION_INST);
            VisionProtocolProcessByte(received_byte);
        }
        break;

    default:
        break;
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
