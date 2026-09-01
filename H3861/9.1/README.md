# Hi3861：自动防掉桌控制

## 功能

- 上电默认进入自动巡航模式，不依赖电脑或串口持续发送命令。
- 使用车底左右红外探头检测桌面是否存在。
- 检测到桌边后先后退 400 ms，再原地转向 500 ms，然后继续前进。
- 保留串口调试命令：`W` 或 `F` 前进、`S` 后退、`A` 左转、`D` 右转、`X` 停止、`O` 前方超声波避障、`E` 防掉桌、`P` 输出红外电平。

## 硬件接口

| 功能 | Hi3861 引脚 |
| --- | --- |
| HC-SR04 Trig | GPIO7 |
| HC-SR04 Echo | GPIO8 |
| 左车底红外 | GPIO13 |
| 右车底红外 | GPIO14 |
| 发送至 STM32 USART1_RX | GPIO11 / UART2_TX |
| 接收 STM32 USART1_TX | GPIO12 / UART2_RX |

## 构建

将本目录放入 OpenHarmony 源码树的 `applications/sample/wifi-iot/app/`，并在上层应用配置中加入本库。进入 OpenHarmony 根目录执行：

```bash
python3 build.py wifiiot
```

烧录输出文件为：

```text
out/wifiiot/Hi3861_wifiiot_app_flash_boot_ota.bin
```

## 红外电平校准

默认桌面存在时的输入电平为低电平。若小车在桌面中央上电后立刻进入后退/转向，请将 `vehicle_controller.c` 中：

```c
#define TABLE_PRESENT_LEVEL WIFI_IOT_GPIO_VALUE0
```

改为 `WIFI_IOT_GPIO_VALUE1`，然后重新编译烧录。
