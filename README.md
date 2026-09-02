# 9.2 双核蓝牙小车——核心改动提交版

这是 9 月 2 日联调的精简 GitHub 提交，只保留本次真正修改并能体现主要工作量的业务代码，不包含 STM32 标准库、Keil 缓存、厂商 SDK 和编译产物。

实测结果：手机通过 JDY-16 发送 `W`，车辆成功前进；发送 `0`，车辆停车。防跌落和避障代码已经实现，但还需要继续进行传感器极性与实车安全验证。

当天完整排错过程和弯路复盘见 [9.2.md](9.2.md)。

## 核心改动

### Hi3861

- `hi3861/vehicle_controller.c`：本次最主要的代码，包含 UART1 蓝牙接收、命令解析、定时控制、UART2 发帧、防跌落状态机、超声波避障和调试日志。
- `hi3861/vehicle_protocol.c/.h`：将左右轮有符号目标速度编码为老师参考代码使用的 6 字节协议。
- `hi3861/BUILD.gn`：OpenHarmony Lite 应用构建目标。

### STM32

- `stm32/usart_vehicle.c/.h`：USART1 115200 接收与逐字节解帧。
- `stm32/vehicle_protocol.c/.h`：校验 `FC ... FD` 帧并还原左右轮有符号速度。
- `stm32/vehicle_motor_app.c/.h`：通信失效保护、编码器闭环 PID、PWM 输出以及解决“电机只响不转”的启动补偿。
- `stm32/vehicle_ws2812.c/.h`：根据左右轮目标渲染转向灯，并执行开机灯光自检。
- `stm32/main.c`：初始化入口和 20 ms 控制循环。

## 最终架构

```text
手机 ──BLE──> JDY-16
              │ UART1，GPIO0/GPIO1，9600 8N1
              ▼
            Hi3861
              │ UART2，GPIO11/GPIO12，115200 8N1
              │ FC L_DIR L_SPEED R_DIR R_SPEED FD
              ▼
          STM32F103 ──PID/PWM──> 左右电机
```

## 指令

| 功能 | ASCII | Hex |
|---|---|---|
| 前进 | `W` | `57` |
| 后退 | `S` | `53` |
| 左转 | `A` | `41` |
| 右转 | `D` | `44` |
| 停车 | `0` | `30` |
| 慢速 100 | `I` | `49` |
| 快速 150 | `K` | `4B` |
| 防跌落 | `G` | `47` |
| 打印底部探头电平 | `P` | `50` |
| 超声波避障 | `V` | `56` |

方向字母后可以附加以 0.1 秒为单位的时长。例如 `W50` 表示前进 5 秒后自动停车；LightBlue 的 Hex 模式下一次发送 `573530`。

## 集成方法

### Hi3861

1. 将 `hi3861` 中的文件放入 OpenHarmony SDK 的应用目录。
2. 在上层 `applications/sample/wifi-iot/app/BUILD.gn` 的 `features` 中加入该目录的目标：

```gn
"你的目录名:vehicle_comm_experiment"
```

3. 关闭 `CONFIG_AT_COMMAND` 或禁止 SDK 调用 `hi_at_init()`，否则 AT 任务会占用 UART1，导致应用收不到 JDY-16 数据。
4. 执行 `python3 build.py wifiiot` 并烧录生成的 all-in-one BIN。

启动成功必须出现：

```text
vehicle app init: uart0=0 uart1=0@9600 uart2=0@115200 frame=FC..FD
vehicle main task started: BLE UART1 9600 + STM32 UART2 115200
```

### STM32

将 `stm32` 中的源文件加入老师提供的 STM32F103 Keil 模板，并保留模板中的标准库、`motor`、`encoder`、`delay` 和 `sys` 驱动。不要同时启用另一套占用 USART1 的接收代码。

最终关键参数：

```text
USART1 = 115200 8N1
PWM_Init(7199, 9)
非零目标启动补偿 = ±3600
控制周期 = 20 ms
通信超时停车 = 200 ms
```

本次完整工程使用 Keil ARMCC V5.06 验证结果为 `0 Error(s), 0 Warning(s)`。

## 测试注意事项

- 首次测试将轮子悬空，并使用小车电池供电。
- 电机持续嗡鸣但不转时立即发送 `0`，避免堵转发热。
- 启动防跌落前，先发送 `P` 验证两只底部探头在桌面和悬空时会正确翻转。
- 当前代码假定“桌面 = 0，悬空 = 1”；未验证前不要直接让小车冲向桌沿。
