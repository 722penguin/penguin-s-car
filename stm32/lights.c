#include "stm32f10x.h"
#include "sys.h"

int main(void)
{
    u8 i;              // 用于依次设置每颗灯
    u8 pos = 1;        // 前灯当前跑马位置
    u8 rear_pos;       // 后灯当前跑马位置

    /* 系统初始化 */
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);
    uart_init(115200);

    /* 保留SWD下载和调试功能 */
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    /* 初始化前后两组彩灯 */
    colorful_led_Init();

    /* 小车启动时向串口发送一次 */
    printf("hello world\r\n");

    while (1)
    {
        /*
         * 收到电脑发送的大写HELLO后，
         * USART_RX_STA变为1，开始灯光效果。
         */
        if (USART_RX_STA == 1)
        {
            /*
             * 前面6颗灯进行白色跑马。
             */
            for (i = 1; i <= 6; i++)
            {
                if (i == pos)
                {
                    L_ws2812_rgb(i, WS_WHITE);
                }
                else
                {
                    L_ws2812_rgb(i, WS_DARK);
                }
            }

            /*
             * 将1～6的前灯位置转换为后灯中间
             * 第2～5颗灯的位置。
             *
             * rear_pos变化：
             * 2 → 3 → 4 → 5 → 2……
             */
            rear_pos = ((pos - 1) % 4) + 2;

            /*
             * 后灯效果：
             * 第1、6颗灯始终亮红色；
             * 第2～5颗灯进行白色跑马。
             */
            for (i = 1; i <= 6; i++)
            {
                if (i == 1 || i == 6)
                {
                    R_ws2812_rgb(i, WS_RED);
                }
                else if (i == rear_pos)
                {
                    R_ws2812_rgb(i, WS_WHITE);
                }
                else
                {
                    R_ws2812_rgb(i, WS_DARK);
                }
            }

            /*
             * 刷新前后灯带，让设置的颜色真正显示。
             */
            L_ws2812_refresh(led_num);
            R_ws2812_refresh(led_num);

            /*
             * 前灯跑到下一颗。
             */
            pos++;

            /*
             * 第6颗结束后回到第1颗。
             */
            if (pos > 6)
            {
                pos = 1;
            }
        }

        /*
         * 每100毫秒移动一次。
         */
        delay_ms(100);
    }
}