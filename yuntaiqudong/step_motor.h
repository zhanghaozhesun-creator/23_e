#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define STEP_MOTOR_ID_YAW     (0U)  /* 0 号轴：Yaw 水平旋转轴。 */
#define STEP_MOTOR_ID_PITCH   (1U)  /* 1 号轴：Pitch 俯仰轴。 */
#define STEP_MOTOR_COUNT      (2U)  /* 驱动中共维护 2 个电机轴。 */

/* 方向约定：顺时针为正方向，逆时针为负方向。 */
typedef enum {
    STEP_MOTOR_DIR_CW = 0,  /* 0：顺时针，位置和角度增加。 */
    STEP_MOTOR_DIR_CCW      /* 1：逆时针，位置和角度减小。 */
} StepMotorDir;

/* 驱动层统一维护状态，业务层不要直接启停底层定时器。 */
typedef enum {
    STEP_MOTOR_STATE_IDLE = 0,  /* 0：空闲，PWM 已停止。 */
    STEP_MOTOR_STATE_RUNNING,   /* 1：连续运行。 */
    STEP_MOTOR_STATE_MOVING,    /* 2：按指定步数运行。 */
    STEP_MOTOR_STATE_ACCEL,     /* 3：预留的加速状态。 */
    STEP_MOTOR_STATE_DECEL,     /* 4：预留的减速状态。 */
    STEP_MOTOR_STATE_STOPPING,  /* 5：预留的停止过渡状态。 */
    STEP_MOTOR_STATE_ERROR      /* 6：参数或急停导致的错误状态。 */
} StepMotorState;

/* 初始化后，当前机械位置被视为 0 度位置。 */
void StepMotor_Init(uint8_t id);
void StepMotor_InitAll(void);
void StepMotor_Enable(uint8_t id);
void StepMotor_Disable(uint8_t id);

/* 速度和限位都使用整数单位，避免在中断中引入浮点运算。 */
void StepMotor_SetDir(uint8_t id, StepMotorDir dir);
void StepMotor_SetSpeed(uint8_t id, uint32_t speed_steps_per_sec);
void StepMotor_SetSpeedDeg(uint8_t id, uint32_t speed_deg_per_sec);
void StepMotor_SetLimitDeg(uint8_t id, StepMotorDir dir, uint16_t limit_deg);
uint16_t StepMotor_GetLimitDeg(uint8_t id, StepMotorDir dir);

/* 正角度/正步数表示顺时针，负角度/负步数表示逆时针。 */
void StepMotor_MoveSteps(uint8_t id, int32_t steps);
void StepMotor_MoveAngle(uint8_t id, int32_t angle_deg);
void StepMotor_RunContinuous(uint8_t id, StepMotorDir dir);
void StepMotor_Stop(uint8_t id);
void StepMotor_EStop(uint8_t id);

bool StepMotor_IsBusy(uint8_t id);
int32_t StepMotor_GetPositionSteps(uint8_t id);
/* 将当前位置重新标定为 0 度，可由外部中断、按键或串口命令触发。 */
void StepMotor_ResetInitialPosition(uint8_t id);
void StepMotor_ResetInitialPositionAll(void);
void StepMotor_SetPositionZero(uint8_t id);
StepMotorState StepMotor_GetState(uint8_t id);

#endif
