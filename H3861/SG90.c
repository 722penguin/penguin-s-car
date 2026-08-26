#include <stdio.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"

/* 互斥锁ID */
static osMutexId_t mutex_id;

/* SG90信号线连接到Hi3861的GPIO2 */
#define GPIO2 2

/* 舵机当前角度标志 */
static uint8_t flag = 0;

/* 先声明三个线程函数 */
static void thread1(void);
static void thread2(void);
static void thread3(void);

/*
 * 产生一个周期约为20ms的舵机控制脉冲
 *
 * duty表示高电平持续时间，单位为微秒：
 * 500us  约对应0度
 * 1000us 约对应45度
 * 1500us 约对应90度
 * 2000us 约对应135度
 * 2500us 约对应180度
 */
static void set_angle(unsigned int duty)
{
    /* GPIO2设置为输出模式 */
    GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT);

    /* 输出高电平 */
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);

    /* 输出低电平，补足20ms周期 */
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/* 舵机转到0度 */
void engine_run_0(void)
{
    int i;

    for (i = 0; i < 10; i++)
    {
        set_angle(500);
    }
}

/* 舵机转到45度 */
void engine_run_45(void)
{
    int i;

    for (i = 0; i < 10; i++)
    {
        set_angle(1000);
    }
}

/* 舵机转到90度 */
void engine_run_90(void)
{
    int i;

    for (i = 0; i < 10; i++)
    {
        set_angle(1500);
    }
}

/* 舵机转到135度 */
void engine_run_135(void)
{
    int i;

    for (i = 0; i < 10; i++)
    {
        set_angle(2000);
    }
}

/* 舵机转到180度 */
void engine_run_180(void)
{
    int i;

    for (i = 0; i < 10; i++)
    {
        set_angle(2500);
    }
}

/*
 * 线程1：
 * 获取互斥锁后，让舵机转到90度；
 * 持有互斥锁5秒，再释放。
 */
static void thread1(void)
{
    osDelay(100U);

    while (1)
    {
        osMutexAcquire(mutex_id, osWaitForever);

        printf("thread1 is running.\r\n");

        flag = 90;
        engine_run_90();

        /* 持有互斥锁5秒 */
        osDelay(5000U);

        osMutexRelease(mutex_id);
    }
}

/*
 * 线程2：
 * 不获取互斥锁，只读取flag并打印舵机角度。
 */
static void thread2(void)
{
    osDelay(100U);

    while (1)
    {
        printf("thread2 is running.\r\n");

        switch (flag)
        {
            case 90:
                printf("SG90 turn 90 du.\r\n");
                break;

            case 180:
                printf("SG90 turn 180 du.\r\n");
                break;

            default:
                break;
        }

        /* 清除标志位 */
        flag = 0;

        osDelay(100U);
    }
}

/*
 * 线程3：
 * 获取互斥锁后，让舵机转到180度；
 * 持有互斥锁3秒，再释放。
 */
static void thread3(void)
{
    while (1)
    {
        osMutexAcquire(mutex_id, osWaitForever);

        printf("thread3 is running.\r\n");

        flag = 180;
        engine_run_180();

        /* 持有互斥锁3秒 */
        osDelay(3000U);

        osMutexRelease(mutex_id);
    }
}

/* 程序启动函数 */
static void SG90(void)
{
    osThreadAttr_t attr;

    /* 初始化GPIO */
    GpioInit();

    /* 将GPIO2设置为普通GPIO功能 */
    IoSetFunc(
        WIFI_IOT_IO_NAME_GPIO_2,
        WIFI_IOT_IO_FUNC_GPIO_2_GPIO
    );

    GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT);

    /* 先创建互斥锁 */
    mutex_id = osMutexNew(NULL);

    if (mutex_id == NULL)
    {
        printf("Failed to create Mutex!\r\n");
        return;
    }

    /*
     * 公共线程属性
     */
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;

    /*
     * 创建线程1
     * 优先级最高
     */
    attr.name = "thread1";
    attr.priority = 26;

    if (osThreadNew(
            (osThreadFunc_t)thread1,
            NULL,
            &attr
        ) == NULL)
    {
        printf("Failed to create thread1!\r\n");
    }

    /*
     * 创建线程2
     * 优先级居中
     */
    attr.name = "thread2";
    attr.priority = 25;

    if (osThreadNew(
            (osThreadFunc_t)thread2,
            NULL,
            &attr
        ) == NULL)
    {
        printf("Failed to create thread2!\r\n");
    }

    /*
     * 创建线程3
     * 优先级最低
     */
    attr.name = "thread3";
    attr.priority = 24;

    if (osThreadNew(
            (osThreadFunc_t)thread3,
            NULL,
            &attr
        ) == NULL)
    {
        printf("Failed to create thread3!\r\n");
    }
}

/* OpenHarmony启动时运行SG90函数 */
APP_FEATURE_INIT(SG90);