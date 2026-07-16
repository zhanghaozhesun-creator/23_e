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

#define GIMBAL_TASK_STACK_WORDS       (512U)
#define GIMBAL_TASK_PRIORITY          (tskIDLE_PRIORITY + 3U)
#define GIMBAL_TASK_WAIT_MS           (100U)

#define YAW_CW_LIMIT_DEG              (3600U)
#define YAW_CCW_LIMIT_DEG             (3600U)
#define PITCH_CW_LIMIT_DEG            (360U)
#define PITCH_CCW_LIMIT_DEG           (360U)

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
    while (1) {
    }
}
#endif

/*-----------------------------------------------------------*/
