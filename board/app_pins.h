/*
 * app_pins.h —— 本工程全部引脚 / 设备参数的唯一出处
 *
 * 对应 docs/01_接线表.md。改接线只改这里（board/board.h 里的 BSP_I2C1_*
 * 也要跟着改，两处保持一致）。
 *
 * 平台：RT-Thread ART-Pi II（STM32H7R7L8H6H）
 */
#ifndef __APP_PINS_H__
#define __APP_PINS_H__

#include <rtthread.h>
#include <board.h>

/* ========================== I2C（心率血氧 + 加速度计） ==========================
 * P1-3 = PB9 = SDA
 * P1-5 = PB8 = SCL
 * 底板上 R23/R24 已经焊了 4.7k 上拉到 3V3，走的是 BSP 的软件 I2C（drv_soft_i2c.c）
 * 一条总线挂两颗芯片：0x57(MAX30102) / 0x68(MPU6050)
 */
#define APP_I2C_BUS_NAME            "i2c1"
#define APP_I2C1_SCL_PIN            GET_PIN(B, 8)       /* P1-5 */
#define APP_I2C1_SDA_PIN            GET_PIN(B, 9)       /* P1-3 */

/* ========================== DS18B20 单总线（体表温度） ==========================
 * P1-11 = PC3，DQ 需要 4.7k 上拉到 3V3
 */
#define APP_DS18B20_PIN             GET_PIN(C, 3)

/* ========================== DHT11 温湿度（环境温度 + 湿度） ==========================
 * P1-13 = PC2，DATA 需要 4.7k~10k 上拉到 3V3
 * DHT11 两次采样间隔要求 ≥1s（手册），这里给 2s；上电后先等 1s 再读。
 */
#define APP_DHT11_PIN               GET_PIN(C, 2)
#define APP_DHT11_PERIOD_MS         2000

/* ========================== 蓝牙串口 ==========================
 * P1-8  = PF13 = USART1_TX  -> 模块 RXD
 * P1-10 = PF12 = USART1_RX  <- 模块 TXD
 */
#define APP_BT_UART_NAME            "uart1"
#define APP_BT_TX_PIN               GET_PIN(F, 13)
#define APP_BT_RX_PIN               GET_PIN(F, 12)
#define APP_BT_BAUD_RATE            BAUD_RATE_9600

/* ========================== I2C 从机地址 ========================== */
#define MAX30102_I2C_ADDR           0x57
#define MPU6050_I2C_ADDR            0x68    /* AD0 悬空/接地 */
#define ADXL345_I2C_ADDR            0x53    /* CS 接 3V3 时 */

/* ========================== MAX30102 采样参数 ========================== */
#define MAX30102_SAMPLE_RATE_HZ     100     /* SPO2_SR = 100Hz */
#define MAX30102_FIFO_DEPTH         32
#define MAX30102_WINDOW_SAMPLES     400     /* 4 秒窗口做心率/血氧计算 */
#define MAX30102_LED_CURRENT        0x24    /* 7.2mA，手指贴着还不够亮就改 0x32/0x3F */

/* ========================== 计步参数 ========================== */
#define MPU6050_SAMPLE_RATE_HZ      50
#define MPU6050_STEP_THRESHOLD_MIN  0.10f   /* g，动态阈值下限 */
#define MPU6050_STEP_THRESHOLD_MAX  0.60f   /* g，动态阈值上限 */
#define MPU6050_STEP_MIN_INTERVAL   250     /* ms，两步之间最短间隔 */

/* ========================== 湿度参考区间（只用于界面提示，不参与控制） ==========================
 * 老人房间：太干（<30%RH）易起静电、呼吸道不适；太湿（>70%RH）易发霉、地面湿滑增加跌倒风险
 */
#define RH_LEVEL_DRY                30
#define RH_LEVEL_WET                70

/* ========================== 线程参数 ========================== */
#define THREAD_STACK_SIZE           2048
#define THREAD_PRIO_MAX30102        8
#define THREAD_PRIO_MPU6050         9
#define THREAD_PRIO_DS18B20         11
#define THREAD_PRIO_DHT11           12
#define THREAD_PRIO_BT              14

#endif /* __APP_PINS_H__ */
