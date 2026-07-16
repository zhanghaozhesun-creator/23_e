# MSPM0G3507 FreeRTOS 工程

本工程面向 MSPM0G3507，使用 TI MSPM0 SDK、SysConfig 和 FreeRTOS。

## 当前硬件配置

硬件配置位于 `blink_led.syscfg`，当前包括：

- `STEPPER_YAW_PWM`：TIMG0，输出引脚 PA12、PA13。
- `STEPPER_PITCH_PWM`：TIMG6，输出引脚 PB2、PB3。
- Yaw 控制：DIR PB13、DCY PB15、SLP PB16、RST PA14。
- Pitch 控制：DIR PB14、DCY PA15、SLP PA16、RST PB17。
- `UART_VISION`：UART0，TX PA0、RX PA1，115200 baud。
- `UART_PC`：UART1，TX PA17、RX PA18，115200 baud。
- 系统时钟及 SWD 调试接口配置。

## FreeRTOS

应用工程引用 CCS 工作区中的 FreeRTOS 构建工程：

`freertos_builds_LP_MSPM0G3507_release_ticlang_2_11_00_07`

应用任务应在 `main.c` 中创建，并在 `vTaskStartScheduler()` 之前完成初始化。

当前云台任务和 Idle Task 使用静态内存。外部 FreeRTOS 构建工程仍启用了
动态分配并关闭了 HighWaterMark；Release 前应在该构建工程中设置
`configSUPPORT_DYNAMIC_ALLOCATION = 0`（确认没有其他动态对象后）和
`INCLUDE_uxTaskGetStackHighWaterMark = 1`，随后重新构建 FreeRTOS 库。

## 构建

在 CCS 中导入应用工程及上述 FreeRTOS 工程，然后执行完整构建。SysConfig 会在构建过程中生成 `ti_msp_dl_config.c` 和 `ti_msp_dl_config.h`。

`blink_led.syscfg` 已关闭项目配置文件生成，因此工程使用仓库中的
`mspm0g3507.cmd` 和 `freertos/ticlang/startup_mspm0g350x_ticlang.c`。
若旧 `Debug` 目录仍包含 `device_linker.cmd` 或根级启动目标文件，应先执行
CCS Clean Project，让托管构建文件重新生成。
