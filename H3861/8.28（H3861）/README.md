# 8.28 Hi3861 OpenHarmony 外设与线程同步实验

## 项目简介

本目录保存了基于 Hi3861 和 OpenHarmony LiteOS-M 完成的外设驱动实验，内容包括 HC-SR04 超声波测距、UART 蓝牙通信、SSD1306 OLED 显示，以及 SHT20 温湿度采集与信号量同步。

这些实验用于熟悉 Hi3861 的 GPIO、UART、I2C 外设，以及 OpenHarmony 中线程、消息队列和信号量等基础机制，为后续智能小车的环境感知、数据显示和通信功能打基础。

## 已完成内容

- HC-SR04 超声波测距
- UART1 蓝牙数据收发
- 使用消息队列在线程之间传递串口数据
- SSD1306 OLED 初始化及字符串显示
- SHT20 温湿度数据采集
- 使用信号量同步多个线程
- 使用 `APP_FEATURE_INIT` 注册应用入口
- 为每个实验编写独立的 `BUILD.gn`

## 目录结构

```text
8.28（H3861）/
├── BUILD.gn
├── 4.0_Hcsr04_Tick/
│   ├── BUILD.gn
│   └── Hcsr04.c
├── 5.0_Uart_BLE/
│   ├── BUILD.gn
│   └── Uart.c
├── 7.0_I2c_Ssd1306/
│   ├── BUILD.gn
│   ├── I2c_Ssd1306.c
│   ├── include/
│   │   ├── hal_bsp_ssd1306.h
│   │   ├── hal_bsp_ssd1306_bmps.h
│   │   └── hal_bsp_ssd1306_fonts.h
│   └── src/
│       └── hal_bsp_ssd1306.c
└── 8.0_Sht20/
    ├── BUILD.gn
    ├── Sht20.c
    ├── include/
    │   └── hal_bsp_sht20.h
    └── src/
        └── hal_bsp_sht20.c
```

## 实验一：HC-SR04超声波测距

### 功能

`4.0_Hcsr04_Tick` 使用 Hi3861 的 GPIO7 和 GPIO8 驱动 HC-SR04 超声波模块：

- GPIO7输出触发脉冲（TRIG）
- GPIO8读取回响脉冲（ECHO）
- 使用 `hi_get_us()` 测量高电平持续时间
- 根据声速计算障碍物距离
- 通过串口输出厘米单位的测量结果

距离计算公式：

```text
距离(cm) = 回响高电平时间(us) × 0.034 ÷ 2
```

### 主要接口

```c
GpioSetDir();
GpioSetOutputVal();
GpioGetInputVal();
hi_udelay();
hi_get_us();
```

该模块可以继续用于小车的前方障碍物检测和自动避障。

## 实验二：UART蓝牙通信与消息队列

### 功能

`5.0_Uart_BLE` 使用 Hi3861 UART1 进行蓝牙串口通信，并创建多个OpenHarmony线程：

- GPIO0复用为UART1 TX
- GPIO1复用为UART1 RX
- UART1发送 `Hello, QST!`
- 接收蓝牙串口数据
- 使用消息队列将接收数据传递给另一个线程
- 使用多个线程观察任务调度过程

当前源码中的实际串口配置为：

```text
波特率：9600
数据位：8
停止位：1
校验位：无
```

### 主要接口

```c
UartInit();
UartWrite();
UartRead();
osThreadNew();
osMessageQueueNew();
osMessageQueuePut();
osMessageQueueGet();
```

该模块后续可用于接收手机、蓝牙模块或其他控制端发送的小车控制指令。

## 实验三：SSD1306 OLED显示

### 功能

`7.0_I2c_Ssd1306` 使用I2C驱动SSD1306 OLED显示屏：

- GPIO9复用为I2C0 SCL
- GPIO10复用为I2C0 SDA
- 初始化SSD1306显示屏
- 清除屏幕内容
- 显示固定字符串
- 显示并更新时、分、秒
- 提供字符、点阵和BMP显示接口

### 主要接口

```c
I2cInit();
I2cSetBaudrate();
I2cWrite();
SSD1306_Init();
SSD1306_CLS();
SSD1306_ShowStr();
```

该模块后续可以显示小车速度、测距结果、温湿度和网络连接状态。

## 实验四：SHT20温湿度采集与信号量

### 功能

`8.0_Sht20` 使用I2C读取SHT20温湿度传感器，并使用OpenHarmony信号量协调三个线程：

- GPIO9复用为I2C0 SCL
- GPIO10复用为I2C0 SDA
- 初始化并软复位SHT20
- 读取原始温度和湿度数据
- 将原始数据转换为温度和相对湿度
- 线程1释放信号量
- 线程2获取信号量并读取温湿度
- 线程3获取信号量并打印同步状态

### 数据换算

温度换算：

```text
Temperature = 175.72 × raw / 65536 - 46.85
```

湿度换算：

```text
Humidity = 125 × raw / 65536 - 6
```

### 主要接口

```c
I2cInit();
I2cWrite();
I2cRead();
osSemaphoreNew();
osSemaphoreRelease();
osSemaphoreAcquire();
```

该模块展示了传感器采集任务与其他任务之间的同步方法。

## 编译配置

每个实验目录都有自己的 `BUILD.gn`，顶层 `BUILD.gn` 通过 `features` 选择需要参与编译的实验。

当前顶层配置启用的是SHT20实验：

```gn
lite_component("app") {
  features = [
    "8.0_Sht20:Sht20",
  ]
}
```

运行其他实验时，应注释当前模块，并取消目标模块前面的注释。例如运行超声波实验：

```gn
lite_component("app") {
  features = [
    "4.0_Hcsr04_Tick:Hcsr04",
  ]
}
```

建议一次只启用一个实验，避免多个模块同时占用相同GPIO、I2C总线或系统资源。

## 实验与智能小车的关系

| 实验 | 在智能小车中的用途 |
|---|---|
| HC-SR04 | 检测前方障碍物、实现自动避障 |
| UART/BLE | 接收控制指令、传输传感器状态 |
| SSD1306 OLED | 显示速度、距离、温湿度和运行状态 |
| SHT20 | 采集环境温度和湿度 |
| 消息队列 | 在线程之间安全传递数据 |
| 信号量 | 协调传感器任务和其他任务的执行 |

## 当前注意事项

- 顶层 `BUILD.gn` 当前仅启用了 `8.0_Sht20`。
- UART源码注释中写有115200，但代码实际配置为9600，调试工具应以9600连接。
- HC-SR04等待ECHO信号的循环目前没有超时保护；未连接模块时可能一直等待。
- SHT20示例当前先创建线程、后创建信号量，正式使用时建议先创建信号量再启动线程。
- OLED和SHT20使用相同的I2C0引脚，可以共用总线，但需要统一初始化和任务调度。
- 本目录保存的是分模块实验，还没有将四个模块整合为一个完整应用。

## 后续计划

1. 为超声波测距增加等待超时保护和数据滤波。
2. 统一UART通信协议，用于接收小车方向和速度指令。
3. 在OLED上显示距离、温湿度和小车运行状态。
4. 调整信号量创建顺序，完善线程同步。
5. 将多个传感器数据通过消息队列发送给决策线程。
6. 将Hi3861的决策结果通过串口发送给STM32底盘控制器。
7. 结合STM32编码器PI闭环，实现完整的智能小车系统。

## 开发环境

- 主控：Hi3861
- 操作系统：OpenHarmony LiteOS-M
- 构建系统：GN
- 线程接口：CMSIS-RTOS2
- 主要外设：GPIO、UART、I2C

## 学习总结

通过本次实验，掌握了Hi3861常用外设的基本使用方法，理解了OpenHarmony应用入口、线程创建、消息队列和信号量的基本作用。这些功能分别对应智能小车的感知、通信、显示和任务协同，是后续进行多模块整合和自动控制的重要基础。

