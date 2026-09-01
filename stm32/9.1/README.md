# STM32F103：电机、编码器与通信执行端

## 功能

- USART1 接收 Hi3861 的 11 字节控制帧。
- TIM2/TIM3 读取左右编码器。
- TIM4 输出双路 PWM 驱动电机。
- 使用 PID 根据目标速度调节左右电机。
- 命令超过 200 ms 未刷新时自动停车。

## 板间通信

连接方式：

| Hi3861 | STM32F103 |
| --- | --- |
| GPIO11 / UART2_TX | PA10 / USART1_RX |
| GPIO12 / UART2_RX | PA9 / USART1_TX |
| GND | GND |

通信参数为 `115200, 8N1`。控制帧格式：

```text
AA 55 CMD SEQ L_LO L_HI R_LO R_HI XOR 0D 0A
```

## Keil 编译与烧录

1. 使用 Keil uVision5 打开 `USER/Template.uvprojx`。
2. 按 `F7` 编译，确认输出为 `0 Error(s)`。
3. 使用 ST-LINK 连接板载 SWD 接口。
4. 执行 **Flash → Download**。

生成的 HEX 文件位于 `OBJ/Template.hex`；该目录被 Git 忽略，不上传编译产物。
