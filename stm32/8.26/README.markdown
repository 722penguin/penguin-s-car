# STM32 PWM 电机控制

## 8月26日完成

- 使用 STM32F103 控制小车左右电机
- 使用 TIM4_CH1 和 TIM4_CH2 输出 PWM
- PB6、PB7 输出 PWM
- PB13、PB14控制电机方向
- 实现左右轮单独转动
- 完成两个方向的直走、转弯和原地转圈的测试
- 完成程序编译与 ST-Link 烧录

## 主要函数

- `PWM_Init()`：初始化 TIM4 PWM
- `Set_Pwm()`：分别设置两个电机的方向和速度
- `Car_Forward()`：前进
- `Car_Backward()`：后退
- `Car_Left()`：左转
- `Car_Right()`：右转
- `Car_Stop()`：停止

## 实验结果

程序成功编译并烧录，左右轮能够按照设定动作运行。