# MSPM0G3507 二维云台驱动

本目录只保存云台业务代码。主工程使用 `../blink_led.syscfg` 作为唯一
SysConfig 配置源。

## 源码组成

- `step_motor.c/.h`：双轴步进电机驱动、软件位置和角度限位。
- `pid_controller.c/.h`：双轴共用 PID 控制器。
- `gimbal_control.c/.h`：角度、视觉、瞄准、IMU 稳定和速度控制接口。

## SysConfig 接口

以实际 PCB 接线为准，确保最终生成下列资源宏：

- `STEPPER_YAW_GPIO`：Yaw 的 DIR、DCY、SLP、RST；
- `STEPPER_PITCH_GPIO`：Pitch 的 DIR、DCY、SLP、RST；
- `STEPPER_YAW_PWM`：TIMG0 单路 PWM，并开启 LOAD_EVENT 中断；
- `STEPPER_PITCH_PWM`：TIMG6 单路 PWM，并开启 LOAD_EVENT 中断。

当前工程使用以下引脚：

| 轴 | PWM | DIR | DCY | SLP | RST |
|---|---|---|---|---|---|
| Yaw | PA12 | PB13 | PB15 | PB16 | PA14 |
| Pitch | PB2 | PB14 | PA15 | PA16 | PB17 |

驱动只使用每个 PWM 实例的 C0 通道。当前 SysConfig 仍生成 C1（Yaw PA13、
Pitch PB3）；如果硬件没有使用这两个输出，应在 SysConfig 中移除 C1，避免
电机运行时在无关引脚输出波形。电机 PWM 的上电默认状态应为停止，SLP/RST
等功率控制脚应保持安全无效状态，完成系统安全检查后才能使能电机。

## 当前任务入口

主工程 `main.c` 使用静态 `GimbalTask` 初始化驱动和软件限位。任务不会在
上电后自动转动电机，后续通信命令和安全状态机应通过任务通知或固定大小
消息接入。
