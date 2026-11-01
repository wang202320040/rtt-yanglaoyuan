/*
 * drv_dht11.c —— DHT11 温湿度驱动（GPIO 位操作，单总线私有协议）
 *
 * 时序（DHT11 数据手册）：
 *   主机起始：拉低 ≥18ms（这里给 20ms）→ 释放
 *   从机响应：约 80us 低 + 80us 高
 *   40 个数据位：每位 = 50us 低 + 高电平（26~28us 表示 0，70us 表示 1）
 *   5 个字节：湿度整数、湿度小数、温度整数、温度小数、校验和
 *
 * 实现要点：
 *   1. 引脚用"开漏输出"，写 1 就是释放总线，靠外部上拉拉高；
 *      这样不用来回切输入/输出模式（rt_pin_mode 走 HAL，切一次要几十微秒，会破坏时序），
 *      开漏模式下 rt_pin_read 仍然能读到真实电平。
 *   2. 40 位数据大约 4~5ms，这一段关中断采样，保证时序不被调度打断。
 */
#include <rtthread.h>
#include <rtdevice.h>

#include "app_pins.h"
#include "drv_dht11.h"

#define DBG_TAG "dht11"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

extern void rt_hw_us_delay(rt_uint32_t us);

#define DHT11_START_LOW_MS      20      /* 起始低电平 ≥18ms */
#define DHT11_WAIT_TIMEOUT_US   200     /* 单个电平跳变的超时（100 次 × 2us） */

static rt_base_t s_pin    = -1;
static rt_bool_t s_ready  = RT_FALSE;

static void dht11_low(void)
{
    rt_pin_write(s_pin, PIN_LOW);
}

static void dht11_release(void)
{
    rt_pin_write(s_pin, PIN_HIGH);      /* 开漏写 1 = 释放，靠外部上拉 */
}

/* 等总线变成指定电平，超时返回错误（不要死等，传感器没接时会一直卡住） */
static rt_err_t dht11_wait_level(rt_int32_t level, rt_uint16_t timeout_us)
{
    while (rt_pin_read(s_pin) != level)
    {
        if (timeout_us < 2)
        {
            return -RT_ETIMEOUT;
        }
        timeout_us -= 2;
        rt_hw_us_delay(2);
    }
    return RT_EOK;
}

rt_err_t dht11_init(void)
{
    s_pin = APP_DHT11_PIN;

    rt_pin_mode(s_pin, PIN_MODE_OUTPUT_OD);
    dht11_release();
    rt_thread_mdelay(1000);             /* 上电后给器件 1 秒稳定 */

    s_ready = RT_TRUE;
    LOG_I("init ok (DATA=PC2/P1-13，记得 4.7k 上拉)");
    return RT_EOK;
}

rt_bool_t dht11_is_ready(void)
{
    return s_ready;
}

rt_err_t dht11_read(rt_int8_t *temp_c, rt_uint8_t *rh_pct)
{
    rt_uint8_t  data[5] = {0};
    rt_uint8_t  i, j;
    rt_base_t   level;
    rt_err_t    ret = RT_EOK;

    if (s_pin < 0)
    {
        return -RT_EINVAL;
    }

    /* ---------- 1) 起始信号 ---------- */
    dht11_low();
    rt_thread_mdelay(DHT11_START_LOW_MS);
    dht11_release();
    rt_hw_us_delay(30);

    /* ---------- 2) 从机响应：80us 低 + 80us 高 ---------- */
    if (dht11_wait_level(PIN_LOW, DHT11_WAIT_TIMEOUT_US) != RT_EOK ||
        dht11_wait_level(PIN_HIGH, DHT11_WAIT_TIMEOUT_US) != RT_EOK ||
        dht11_wait_level(PIN_LOW, DHT11_WAIT_TIMEOUT_US) != RT_EOK)
    {
        LOG_D("no response（DATA 没接 / 没上拉 / 采样太快）");
        s_ready = RT_FALSE;
        return -RT_ETIMEOUT;
    }

    /* ---------- 3) 40 位数据，关中断保证时序 ---------- */
    level = rt_hw_interrupt_disable();

    for (i = 0; i < 5 && ret == RT_EOK; i++)
    {
        for (j = 0; j < 8; j++)
        {
            /* 每位以 50us 低电平开始，先等它变高 */
            if (dht11_wait_level(PIN_HIGH, DHT11_WAIT_TIMEOUT_US) != RT_EOK)
            {
                ret = -RT_ETIMEOUT;
                break;
            }

            /* 40us 后采样：还是高就是 1（高 70us），已经变低就是 0（高 26us） */
            rt_hw_us_delay(40);
            data[i] <<= 1;
            if (rt_pin_read(s_pin) == PIN_HIGH)
            {
                data[i] |= 0x01;
            }

            /* 等这一位结束 */
            if (dht11_wait_level(PIN_LOW, DHT11_WAIT_TIMEOUT_US) != RT_EOK)
            {
                ret = -RT_ETIMEOUT;
                break;
            }
        }
    }

    rt_hw_interrupt_enable(level);

    if (ret != RT_EOK)
    {
        LOG_D("bit stream timeout");
        s_ready = RT_FALSE;
        return ret;
    }

    /* ---------- 4) 校验 ---------- */
    if ((rt_uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
    {
        LOG_D("checksum error: %02X %02X %02X %02X | %02X",
              data[0], data[1], data[2], data[3], data[4]);
        s_ready = RT_FALSE;
        return -RT_EIO;
    }

    if (rh_pct) *rh_pct = data[0];
    if (temp_c) *temp_c = (rt_int8_t)data[2];

    s_ready = RT_TRUE;
    LOG_D("T=%d C, RH=%d %%", data[2], data[0]);
    return RT_EOK;
}
