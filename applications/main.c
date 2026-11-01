/*
 * main.c —— 手环端入口
 *
 * 上电流程：
 *   1. 打印接线提示
 *   2. health_init()：初始化 MAX30102 / MPU6050 / DS18B20 / 蓝牙串口，并建采集线程
 *   3. 主线程进入低功耗空转（真正干活的是 4 个采集线程）
 *
 * 调试：
 *   msh 里输入 health 看状态，steps_reset 清步数
 */
#include <rtthread.h>
#include <rtdevice.h>

#include "app_pins.h"
#include "app_health.h"

int main(void)
{
    rt_kprintf("\n");
    rt_kprintf("==========================================\n");
    rt_kprintf(" 星守手环 · ART-Pi II  (STM32H7R7 @480MHz)\n");
    rt_kprintf("------------------------------------------\n");
    rt_kprintf(" I2C   SDA=P1-3(PB9)  SCL=P1-5(PB8)  0x57/0x68\n");
    rt_kprintf(" 1Wire DS18B20 DQ=P1-11(PC3)  4.7k 上拉\n");
    rt_kprintf(" DHT11 DATA=P1-13(PC2)  4.7k~10k 上拉\n");
    rt_kprintf(" UART1 TX =P1-8(PF13)  RX =P1-10(PF12) 9600\n");
    rt_kprintf("==========================================\n");
    rt_kprintf(" 命令: health / steps_reset\n\n");

    if (health_init() != RT_EOK)
    {
        rt_kprintf("[main] 有模块初始化失败，用 health 命令看细节\n");
    }

    /* 主线程没事干，让出去 */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}
