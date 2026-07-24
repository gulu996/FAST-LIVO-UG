# STM32F103C8T6 阶段 B 资源要求

目标器件为 `STM32F103C8T6`，属于 STM32F103x8/B medium-density。Keil
工程必须显式定义 `STM32F10X_MD`。本轮只准备正确的 medium-density
startup/vector，不启用新定时器、USART 或 GPIO，也不声称已经过 Keil
clean build 或实物验证。

## 当前由源码确认的资源

| 功能 | 外设 | GPIO | 证据 | 状态 |
|---|---|---|---|---|
| 相机触发 | TIM2_CH2 | PA1 | `HARDWARE/TIMER/timer.c` 的 `TIM2_PWM_Init` | 当前使用 |
| MID-360 本地 PPS 候选输出 | TIM3_CH2 partial remap | PB5 | `TIM3_PWM_Init` | 当前使用；实物终点/极性待测 |
| MID-360 合成 RMC | USART1_TX | PA9 | `SYSTEM/usart/usart.c` | 当前使用，9600 8N1 |
| USART1_RX | USART1_RX | PA10 | 同上 | 已初始化；实际是否接线待确认 |

## 阶段 B 待分配资源

以下资源一律保持“待定”，不得仅凭复用表写死：

- MCU 二进制事件串口：USART 实例、TX/RX GPIO、USB-UART、电平待实物确认；
- GNSS PPS 输入捕获：timer/channel、GPIO、边沿极性、电平待实物确认；
- 是否需要 GNSS 数据进入 MCU：默认不需要，GNSS 数据串口直接进入 Jetson；
- TIM4、USART3：虽然 medium-density 芯片具备，但本轮不启用其中断；
- USB CDC：只有确认 PA11/PA12、板级接口和 USB 时钟资源后才评估。

## 编译期门禁要求

未来固件实现必须把所有待定资源写成构建配置，而不是散落在驱动中：

```c
#if !defined(STM32F10X_MD)
#error "STM32F103C8T6 requires STM32F10X_MD"
#endif

#if !defined(SENSOR_EVENT_USART_INSTANCE)
#error "Confirm the physical MCU event UART before enabling Stage B hardware"
#endif

#if !defined(GNSS_PPS_CAPTURE_TIMER) || !defined(GNSS_PPS_CAPTURE_CHANNEL)
#error "Confirm GNSS PPS capture timer/channel/GPIO before enabling PPS capture"
#endif
```

这些示例是未来硬件功能的门禁约定；本轮未把宏加入运行固件，避免在未知
引脚上误启用外设。

## 用户必须提供或实测的信息

1. MCU 封装、板级原理图、连接器引脚表；
2. PA1、PB5、PA9 的实际电缆终点、电平、是否反相；
3. 可用 USART 的 TX/RX 是否引出，是否已有板载器件占用；
4. 可用输入捕获 GPIO、GNSS PPS 电平和边沿极性；
5. MCU、MID-360、GNSS 是否共地；
6. HSE 晶体型号、负载电容与频率计实测 ppm；
7. 用示波器同时测相机触发、本地 PPS、RMC TX、GNSS PPS。

## startup 状态

- 原工程引用 `CORE/startup_stm32f10x_hd.s`，但其文件头和向量内容实际是
  low-density 变体，TIM4/I2C2/SPI2/USART3 位置为 reserved。
- 新增 `CORE/startup_stm32f10x_md.s`，补齐 medium-density 向量。
- 因当前 Linux 环境没有 Keil/armcc，本轮不删除旧文件，也不把工程切到
  新 startup。切换必须在 Keil clean build、map/vector 核对通过后进行。
