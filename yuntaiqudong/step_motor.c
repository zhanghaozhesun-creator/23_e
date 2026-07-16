#include "step_motor.h"

#include "ti_msp_dl_config.h"

#define STEP_MOTOR_DEG_PER_REV             (360U)
#define STEP_MOTOR_DEFAULT_STEPS_PER_REV   (6400U)
#define STEP_MOTOR_DEFAULT_LIMIT_DEG       (UINT16_MAX)
#define STEP_MOTOR_MIN_PERIOD_TICKS        (800U)
#define STEP_MOTOR_MAX_PERIOD_TICKS        (65535U)
#define STEP_MOTOR_MAX_MOVE_STEPS          ((uint32_t) INT32_MAX)

typedef struct {
    /* 硬件极性集中放在配置表，业务层不关心具体 GPIO 电平。 */
    GPIO_Regs *dir_port;
    uint32_t dir_pin;
    bool dir_invert;

    GPIO_Regs *decay_port;
    uint32_t decay_pin;
    bool has_decay;
    bool decay_active_high;

    GPIO_Regs *sleep_port;
    uint32_t sleep_pin;
    bool has_sleep;
    bool sleep_awake_high;

    GPIO_Regs *reset_port;
    uint32_t reset_pin;
    bool has_reset;
    bool reset_release_high;

    GPTIMER_Regs *timer;
    IRQn_Type timer_irqn;
    DL_TIMER_CC_INDEX timer_cc_index;
    uint32_t timer_clock_hz;

    uint32_t steps_per_rev;
    uint32_t min_period_ticks;
    uint32_t max_period_ticks;
    uint32_t max_move_steps;
} StepMotorConfig;

typedef struct {
    /* 中断和主循环都会访问这些运行数据，必须保持 volatile。 */
    volatile int32_t remain_steps;
    volatile int32_t position_steps;
    volatile int32_t cw_limit_steps;
    volatile int32_t ccw_limit_steps;
    volatile uint32_t speed_steps_per_sec;
    volatile uint16_t cw_limit_deg;
    volatile uint16_t ccw_limit_deg;
    volatile StepMotorState state;
    volatile StepMotorDir dir;
    bool enabled;
} StepMotorHandle;

/* 每个轴只在这里绑定 SysConfig 生成的硬件资源。 */
static const StepMotorConfig g_stepMotorConfig[STEP_MOTOR_COUNT] = {
    [STEP_MOTOR_ID_YAW] = {
        .dir_port = STEPPER_YAW_GPIO_YAW_DIR_PORT,
        .dir_pin = STEPPER_YAW_GPIO_YAW_DIR_PIN,
        .dir_invert = false,
        .decay_port = STEPPER_YAW_GPIO_YAW_DCY_PORT,
        .decay_pin = STEPPER_YAW_GPIO_YAW_DCY_PIN,
        .has_decay = true,
        .decay_active_high = true,
        .sleep_port = STEPPER_YAW_GPIO_YAW_SLP_PORT,
        .sleep_pin = STEPPER_YAW_GPIO_YAW_SLP_PIN,
        .has_sleep = true,
        .sleep_awake_high = true,
        .reset_port = STEPPER_YAW_GPIO_YAW_RST_PORT,
        .reset_pin = STEPPER_YAW_GPIO_YAW_RST_PIN,
        .has_reset = true,
        .reset_release_high = true,
        .timer = STEPPER_YAW_PWM_INST,
        .timer_irqn = STEPPER_YAW_PWM_INST_INT_IRQN,
        .timer_cc_index = GPIO_STEPPER_YAW_PWM_C0_IDX,
        .timer_clock_hz = STEPPER_YAW_PWM_INST_CLK_FREQ,
        .steps_per_rev = STEP_MOTOR_DEFAULT_STEPS_PER_REV,
        .min_period_ticks = STEP_MOTOR_MIN_PERIOD_TICKS,
        .max_period_ticks = STEP_MOTOR_MAX_PERIOD_TICKS,
        .max_move_steps = STEP_MOTOR_MAX_MOVE_STEPS,
    },
    [STEP_MOTOR_ID_PITCH] = {
        .dir_port = STEPPER_PITCH_GPIO_PITCH_DIR_PORT,
        .dir_pin = STEPPER_PITCH_GPIO_PITCH_DIR_PIN,
        .dir_invert = false,
        .decay_port = STEPPER_PITCH_GPIO_PITCH_DCY_PORT,
        .decay_pin = STEPPER_PITCH_GPIO_PITCH_DCY_PIN,
        .has_decay = true,
        .decay_active_high = true,
        .sleep_port = STEPPER_PITCH_GPIO_PITCH_SLP_PORT,
        .sleep_pin = STEPPER_PITCH_GPIO_PITCH_SLP_PIN,
        .has_sleep = true,
        .sleep_awake_high = true,
        .reset_port = STEPPER_PITCH_GPIO_PITCH_RST_PORT,
        .reset_pin = STEPPER_PITCH_GPIO_PITCH_RST_PIN,
        .has_reset = true,
        .reset_release_high = true,
        .timer = STEPPER_PITCH_PWM_INST,
        .timer_irqn = STEPPER_PITCH_PWM_INST_INT_IRQN,
        .timer_cc_index = GPIO_STEPPER_PITCH_PWM_C0_IDX,
        .timer_clock_hz = STEPPER_PITCH_PWM_INST_CLK_FREQ,
        .steps_per_rev = STEP_MOTOR_DEFAULT_STEPS_PER_REV,
        .min_period_ticks = STEP_MOTOR_MIN_PERIOD_TICKS,
        .max_period_ticks = STEP_MOTOR_MAX_PERIOD_TICKS,
        .max_move_steps = STEP_MOTOR_MAX_MOVE_STEPS,
    },
};

static StepMotorHandle g_stepMotorHandle[STEP_MOTOR_COUNT];

static bool StepMotor_IsValidId(uint8_t id)
{
    return (id < STEP_MOTOR_COUNT);
}

static uint32_t StepMotor_Abs32(int32_t value)
{
    if (value < 0) {
        return ((uint32_t) (-(value + 1))) + 1U;
    }

    return (uint32_t) value;
}

static void StepMotor_WritePin(
    GPIO_Regs *port, uint32_t pin, bool active_high, bool active)
{
    if (active == active_high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static uint32_t StepMotor_ClampPeriod(
    const StepMotorConfig *config, uint32_t period)
{
    if (period < config->min_period_ticks) {
        return config->min_period_ticks;
    }

    if (period > config->max_period_ticks) {
        return config->max_period_ticks;
    }

    return period;
}

static uint32_t StepMotor_SpeedToPeriod(
    const StepMotorConfig *config, uint32_t speed_steps_per_sec)
{
    uint32_t period;

    if (speed_steps_per_sec == 0U) {
        return 0U;
    }

    period = (uint32_t) (((uint64_t) config->timer_clock_hz +
                            (speed_steps_per_sec / 2U)) /
                         speed_steps_per_sec);

    return StepMotor_ClampPeriod(config, period);
}

static uint32_t StepMotor_DegSpeedToSteps(
    const StepMotorConfig *config, uint32_t speed_deg_per_sec)
{
    /* 角速度先转换成脉冲频率，后续 PWM 只处理 steps/s。 */
    return (uint32_t) (((uint64_t) speed_deg_per_sec *
                          config->steps_per_rev +
                      (STEP_MOTOR_DEG_PER_REV / 2U)) /
                     STEP_MOTOR_DEG_PER_REV);
}

static uint32_t StepMotor_DegAngleToSteps(
    const StepMotorConfig *config, uint32_t angle_deg)
{
    /* 当前库以 0 度为原点，角度命令最终都转换成相对步数。 */
    return (uint32_t) (((uint64_t) angle_deg * config->steps_per_rev +
                          (STEP_MOTOR_DEG_PER_REV / 2U)) /
                     STEP_MOTOR_DEG_PER_REV);
}

static int32_t StepMotor_LimitDegToSteps(
    const StepMotorConfig *config, uint16_t limit_deg)
{
    return (int32_t) StepMotor_DegAngleToSteps(config, limit_deg);
}

static int32_t StepMotor_GetRemainingLimitSteps(uint8_t id, StepMotorDir dir)
{
    StepMotorHandle *handle = &g_stepMotorHandle[id];
    int32_t position = handle->position_steps;

    /* 顺时针限位是正位置上限，逆时针限位是负位置下限。 */
    if (dir == STEP_MOTOR_DIR_CW) {
        if (position >= handle->cw_limit_steps) {
            return 0;
        }

        return handle->cw_limit_steps - position;
    }

    if (position <= -handle->ccw_limit_steps) {
        return 0;
    }

    return position + handle->ccw_limit_steps;
}

static bool StepMotor_CanStep(uint8_t id)
{
    StepMotorHandle *handle = &g_stepMotorHandle[id];

    return (StepMotor_GetRemainingLimitSteps(id, handle->dir) > 0);
}

static void StepMotor_SetPwmDuty(const StepMotorConfig *config, uint32_t period)
{
    DL_Timer_setLoadValue(config->timer, period);
    DL_Timer_setCaptureCompareValue(
        config->timer, period / 2U, config->timer_cc_index);
}

static void StepMotor_StopPwm(const StepMotorConfig *config)
{
    DL_Timer_stopCounter(config->timer);
    DL_Timer_setCaptureCompareValue(config->timer, 0U, config->timer_cc_index);
}

static void StepMotor_StartPwm(uint8_t id)
{
    const StepMotorConfig *config = &g_stepMotorConfig[id];
    StepMotorHandle *handle = &g_stepMotorHandle[id];
    uint32_t period = StepMotor_SpeedToPeriod(config, handle->speed_steps_per_sec);

    if (period == 0U) {
        handle->state = STEP_MOTOR_STATE_ERROR;
        return;
    }

    StepMotor_SetPwmDuty(config, period);
    DL_Timer_startCounter(config->timer);
}

static void StepMotor_StopFromIsr(uint8_t id)
{
    StepMotorHandle *handle = &g_stepMotorHandle[id];

    StepMotor_StopPwm(&g_stepMotorConfig[id]);
    handle->remain_steps = 0;
    handle->state = STEP_MOTOR_STATE_IDLE;
}

static void StepMotor_UpdatePositionFromPulse(uint8_t id)
{
    StepMotorHandle *handle = &g_stepMotorHandle[id];

    if (handle->dir == STEP_MOTOR_DIR_CW) {
        handle->position_steps++;
    } else {
        handle->position_steps--;
    }
}

void StepMotor_Init(uint8_t id)
{
    StepMotorHandle *handle;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    handle = &g_stepMotorHandle[id];
    StepMotor_StopPwm(&g_stepMotorConfig[id]);
    handle->remain_steps = 0;
    /* 上电时机械位置作为角度坐标零点。 */
    handle->position_steps = 0;
    handle->cw_limit_deg = STEP_MOTOR_DEFAULT_LIMIT_DEG;
    handle->ccw_limit_deg = STEP_MOTOR_DEFAULT_LIMIT_DEG;
    handle->cw_limit_steps =
        StepMotor_LimitDegToSteps(&g_stepMotorConfig[id], handle->cw_limit_deg);
    handle->ccw_limit_steps =
        StepMotor_LimitDegToSteps(&g_stepMotorConfig[id], handle->ccw_limit_deg);
    handle->speed_steps_per_sec = 0;
    handle->state = STEP_MOTOR_STATE_IDLE;
    handle->dir = STEP_MOTOR_DIR_CW;
    handle->enabled = false;
    StepMotor_SetDir(id, STEP_MOTOR_DIR_CW);
    NVIC_SetPriority(g_stepMotorConfig[id].timer_irqn, 1U);
    NVIC_EnableIRQ(g_stepMotorConfig[id].timer_irqn);
}

void StepMotor_InitAll(void)
{
    uint8_t id;

    for (id = 0U; id < STEP_MOTOR_COUNT; id++) {
        StepMotor_Init(id);
    }
}

void StepMotor_Enable(uint8_t id)
{
    const StepMotorConfig *config;
    StepMotorHandle *handle;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    config = &g_stepMotorConfig[id];
    handle = &g_stepMotorHandle[id];

    if (config->has_reset) {
        StepMotor_WritePin(
            config->reset_port, config->reset_pin, config->reset_release_high, true);
    }

    if (config->has_sleep) {
        StepMotor_WritePin(
            config->sleep_port, config->sleep_pin, config->sleep_awake_high, true);
    }

    if (config->has_decay) {
        StepMotor_WritePin(
            config->decay_port, config->decay_pin, config->decay_active_high, true);
    }

    handle->enabled = true;
    if (handle->state == STEP_MOTOR_STATE_ERROR) {
        handle->state = STEP_MOTOR_STATE_IDLE;
    }
}

void StepMotor_Disable(uint8_t id)
{
    const StepMotorConfig *config;
    StepMotorHandle *handle;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    config = &g_stepMotorConfig[id];
    handle = &g_stepMotorHandle[id];

    StepMotor_Stop(id);

    if (config->has_decay) {
        StepMotor_WritePin(
            config->decay_port, config->decay_pin, config->decay_active_high, false);
    }

    if (config->has_sleep) {
        StepMotor_WritePin(
            config->sleep_port, config->sleep_pin, config->sleep_awake_high, false);
    }

    if (config->has_reset) {
        StepMotor_WritePin(
            config->reset_port, config->reset_pin, config->reset_release_high, false);
    }

    handle->enabled = false;
}

void StepMotor_SetDir(uint8_t id, StepMotorDir dir)
{
    const StepMotorConfig *config;
    StepMotorHandle *handle;
    bool dir_high;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    config = &g_stepMotorConfig[id];
    handle = &g_stepMotorHandle[id];
    if (StepMotor_IsBusy(id)) {
        return;
    }

    dir_high = (dir == STEP_MOTOR_DIR_CCW);
    if (config->dir_invert) {
        dir_high = !dir_high;
    }

    if (dir_high) {
        DL_GPIO_setPins(config->dir_port, config->dir_pin);
    } else {
        DL_GPIO_clearPins(config->dir_port, config->dir_pin);
    }

    handle->dir = dir;
}

void StepMotor_SetSpeed(uint8_t id, uint32_t speed_steps_per_sec)
{
    const StepMotorConfig *config;
    StepMotorHandle *handle;
    uint32_t period;
    uint32_t max_speed;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    config = &g_stepMotorConfig[id];
    handle = &g_stepMotorHandle[id];

    max_speed = config->timer_clock_hz / config->min_period_ticks;
    if (speed_steps_per_sec > max_speed) {
        speed_steps_per_sec = max_speed;
    }

    handle->speed_steps_per_sec = speed_steps_per_sec;

    if ((handle->state == STEP_MOTOR_STATE_RUNNING) ||
        (handle->state == STEP_MOTOR_STATE_MOVING)) {
        period = StepMotor_SpeedToPeriod(config, speed_steps_per_sec);
        if (period == 0U) {
            StepMotor_Stop(id);
        } else {
            StepMotor_SetPwmDuty(config, period);
        }
    }
}

void StepMotor_SetSpeedDeg(uint8_t id, uint32_t speed_deg_per_sec)
{
    if (!StepMotor_IsValidId(id)) {
        return;
    }

    StepMotor_SetSpeed(
        id, StepMotor_DegSpeedToSteps(&g_stepMotorConfig[id], speed_deg_per_sec));
}

void StepMotor_SetLimitDeg(uint8_t id, StepMotorDir dir, uint16_t limit_deg)
{
    StepMotorHandle *handle;
    int32_t limit_steps;
    uint32_t primask;
    bool stop_required = false;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    handle = &g_stepMotorHandle[id];
    limit_steps = StepMotor_LimitDegToSteps(&g_stepMotorConfig[id], limit_deg);

    /* 限位可能在运行中被修改，更新时短暂关中断保护状态一致性。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (dir == STEP_MOTOR_DIR_CW) {
        handle->cw_limit_deg = limit_deg;
        handle->cw_limit_steps = limit_steps;
    } else {
        handle->ccw_limit_deg = limit_deg;
        handle->ccw_limit_steps = limit_steps;
    }

    if (StepMotor_IsBusy(id) && !StepMotor_CanStep(id)) {
        stop_required = true;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    if (stop_required) {
        StepMotor_Stop(id);
    }
}

uint16_t StepMotor_GetLimitDeg(uint8_t id, StepMotorDir dir)
{
    if (!StepMotor_IsValidId(id)) {
        return 0U;
    }

    if (dir == STEP_MOTOR_DIR_CW) {
        return g_stepMotorHandle[id].cw_limit_deg;
    }

    return g_stepMotorHandle[id].ccw_limit_deg;
}

void StepMotor_MoveSteps(uint8_t id, int32_t steps)
{
    StepMotorHandle *handle;
    uint32_t abs_steps;
    int32_t allowed_steps;
    StepMotorDir dir;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    handle = &g_stepMotorHandle[id];
    if (StepMotor_IsBusy(id) || (steps == 0)) {
        return;
    }

    dir = (steps > 0) ? STEP_MOTOR_DIR_CW : STEP_MOTOR_DIR_CCW;
    allowed_steps = StepMotor_GetRemainingLimitSteps(id, dir);
    if (allowed_steps <= 0) {
        return;
    }

    abs_steps = StepMotor_Abs32(steps);
    if (abs_steps > g_stepMotorConfig[id].max_move_steps) {
        handle->state = STEP_MOTOR_STATE_ERROR;
        return;
    }

    if (abs_steps > (uint32_t) allowed_steps) {
        /* 定长运动超过限位时，裁剪到刚好到达限位。 */
        abs_steps = (uint32_t) allowed_steps;
    }

    StepMotor_Enable(id);
    StepMotor_SetDir(id, dir);
    handle->remain_steps = (int32_t) abs_steps;
    handle->state = STEP_MOTOR_STATE_MOVING;
    StepMotor_StartPwm(id);
}

void StepMotor_MoveAngle(uint8_t id, int32_t angle_deg)
{
    uint32_t abs_angle;
    uint32_t steps;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    abs_angle = StepMotor_Abs32(angle_deg);
    steps = StepMotor_DegAngleToSteps(&g_stepMotorConfig[id], abs_angle);
    if ((steps == 0U) || (steps > (uint32_t) INT32_MAX)) {
        if (steps > (uint32_t) INT32_MAX) {
            g_stepMotorHandle[id].state = STEP_MOTOR_STATE_ERROR;
        }
        return;
    }

    StepMotor_MoveSteps(
        id, (angle_deg >= 0) ? (int32_t) steps : -((int32_t) steps));
}

void StepMotor_RunContinuous(uint8_t id, StepMotorDir dir)
{
    StepMotorHandle *handle;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    handle = &g_stepMotorHandle[id];
    if (handle->speed_steps_per_sec == 0U) {
        StepMotor_Stop(id);
        return;
    }

    if (StepMotor_GetRemainingLimitSteps(id, dir) <= 0) {
        StepMotor_Stop(id);
        return;
    }

    StepMotor_Enable(id);
    StepMotor_SetDir(id, dir);
    handle->remain_steps = 0;
    handle->state = STEP_MOTOR_STATE_RUNNING;
    StepMotor_StartPwm(id);
}

void StepMotor_Stop(uint8_t id)
{
    StepMotorHandle *handle;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    handle = &g_stepMotorHandle[id];
    StepMotor_StopPwm(&g_stepMotorConfig[id]);
    handle->remain_steps = 0;
    handle->state = STEP_MOTOR_STATE_IDLE;
}

void StepMotor_EStop(uint8_t id)
{
    if (!StepMotor_IsValidId(id)) {
        return;
    }

    StepMotor_Disable(id);
    g_stepMotorHandle[id].state = STEP_MOTOR_STATE_ERROR;
}

bool StepMotor_IsBusy(uint8_t id)
{
    StepMotorState state;

    if (!StepMotor_IsValidId(id)) {
        return false;
    }

    state = g_stepMotorHandle[id].state;
    return (state == STEP_MOTOR_STATE_RUNNING) ||
           (state == STEP_MOTOR_STATE_MOVING) ||
           (state == STEP_MOTOR_STATE_ACCEL) ||
           (state == STEP_MOTOR_STATE_DECEL) ||
           (state == STEP_MOTOR_STATE_STOPPING);
}

int32_t StepMotor_GetPositionSteps(uint8_t id)
{
    if (!StepMotor_IsValidId(id)) {
        return 0;
    }

    return g_stepMotorHandle[id].position_steps;
}

void StepMotor_ResetInitialPosition(uint8_t id)
{
    uint32_t primask;

    if (!StepMotor_IsValidId(id)) {
        return;
    }

    /* 只重置坐标系，不改变当前速度、限位和电机运行状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    g_stepMotorHandle[id].position_steps = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}

void StepMotor_ResetInitialPositionAll(void)
{
    uint8_t id;

    for (id = 0U; id < STEP_MOTOR_COUNT; id++) {
        StepMotor_ResetInitialPosition(id);
    }
}

void StepMotor_SetPositionZero(uint8_t id)
{
    StepMotor_ResetInitialPosition(id);
}

StepMotorState StepMotor_GetState(uint8_t id)
{
    if (!StepMotor_IsValidId(id)) {
        return STEP_MOTOR_STATE_ERROR;
    }

    return g_stepMotorHandle[id].state;
}

static void StepMotor_HandleTimerIrq(uint8_t id)
{
    StepMotorHandle *handle = &g_stepMotorHandle[id];

    switch (DL_Timer_getPendingInterrupt(g_stepMotorConfig[id].timer)) {
    case DL_TIMER_IIDX_LOAD:
        /* ISR 只做计步和停机判断，避免阻塞操作影响 PWM 节拍。 */
        if (handle->state == STEP_MOTOR_STATE_MOVING) {
            if ((handle->remain_steps <= 0) || !StepMotor_CanStep(id)) {
                StepMotor_StopFromIsr(id);
            } else {
                handle->remain_steps--;
                StepMotor_UpdatePositionFromPulse(id);
                if ((handle->remain_steps == 0) || !StepMotor_CanStep(id)) {
                    StepMotor_StopFromIsr(id);
                }
            }
        } else if (handle->state == STEP_MOTOR_STATE_RUNNING) {
            if (!StepMotor_CanStep(id)) {
                StepMotor_StopFromIsr(id);
            } else {
                StepMotor_UpdatePositionFromPulse(id);
                if (!StepMotor_CanStep(id)) {
                    StepMotor_StopFromIsr(id);
                }
            }
        } else {
            StepMotor_StopFromIsr(id);
        }
        break;

    default:
        break;
    }
}

void STEPPER_YAW_PWM_INST_IRQHandler(void)
{
    StepMotor_HandleTimerIrq(STEP_MOTOR_ID_YAW);
}

void STEPPER_PITCH_PWM_INST_IRQHandler(void)
{
    StepMotor_HandleTimerIrq(STEP_MOTOR_ID_PITCH);
}
