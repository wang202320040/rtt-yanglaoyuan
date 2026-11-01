/*
 * drv_ds18b20.c —— DS18B20 单总线驱动（GPIO 位操作）
 *
 * 时序全部按 DS18B20 数据手册（±15us 容差内留了余量）：
 *   复位：拉低 480us -> 释放 -> 60~70us 后采样存在脉冲 -> 再等 410us
 *   写1 ：拉低 2us -> 释放 -> 等 60us
 *   写0 ：拉低 60us -> 释放 -> 等 2us
 *   读  ：拉低 2us -> 释放 -> 10us 后采样 -> 再等 50us
 *
 * 注意：DQ 是"开漏 + 外部 4.7k 上拉"，所以写 1 就是释放总线。
 */
#include <rtthread.h>
#include <rtdevice.h>

#include "drv_ds18b20.h"

#define DBG_TAG "ds18b20"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* 需要 BSP 提供微秒延时（ART-Pi2 BSP 的 drv_common.c 里已有 rt_hw_us_delay） */
extern void rt_hw_us_delay(rt_uint32_t us);

#define OW_CMD_SKIP_ROM     0xCC
#define OW_CMD_CONVERT_T    0x44
#define OW_CMD_READ_SCRATCH 0xBE

static rt_base_t s_pin = -1;
static rt_bool_t s_present = RT_FALSE;

/* ------------------------------ 位操作 ------------------------------ */
static void ow_low(void)
{
    rt_pin_write(s_pin, PIN_LOW);
}

static void ow_release(void)
{
    rt_pin_write(s_pin, PIN_HIGH);      /* 开漏输出写 1 = 释放，靠外部上拉拉高 */
}

static rt_int32_t ow_read_level(void)
{
    return rt_pin_read(s_pin);
}

/* 复位并检测存在脉冲，返回 RT_TRUE 表示器件在位 */
static rt_bool_t ow_reset(void)
{
    rt_bool_t present;

    ow_low();
    rt_hw_us_delay(480);
    ow_release();
    rt_hw_us_delay(70);

    present = (ow_read_level() == PIN_LOW) ? RT_TRUE : RT_FALSE;   /* 器件拉低 = 在位 */

    rt_hw_us_delay(410);
    return present;
}

static void ow_write_bit(rt_uint8_t bit)
{
    if (bit)
    {
        ow_low();
        rt_hw_us_delay(2);
        ow_release();
        rt_hw_us_delay(60);
    }
    else
    {
        ow_low();
        rt_hw_us_delay(60);
        ow_release();
        rt_hw_us_delay(2);
    }
}

static rt_uint8_t ow_read_bit(void)
{
    rt_uint8_t v;

    ow_low();
    rt_hw_us_delay(2);
    ow_release();
    rt_hw_us_delay(10);

    v = (ow_read_level() == PIN_HIGH) ? 1 : 0;

    rt_hw_us_delay(50);
    return v;
}

static void ow_write_byte(rt_uint8_t byte)
{
    rt_uint8_t i;

    for (i = 0; i < 8; i++)
    {
        ow_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

static rt_uint8_t ow_read_byte(void)
{
    rt_uint8_t i, byte = 0;

    for (i = 0; i < 8; i++)
    {
        byte >>= 1;
        if (ow_read_bit())
        {
            byte |= 0x80;
        }
    }
    return byte;
}

/* Dallas/Maxim CRC8（多项式 x^8 + x^5 + x^4 + 1） */
static rt_uint8_t ow_crc8(const rt_uint8_t *data, rt_uint8_t len)
{
    rt_uint8_t crc = 0, i, j;

    for (i = 0; i < len; i++)
    {
        rt_uint8_t b = data[i];

        for (j = 0; j < 8; j++)
        {
            rt_uint8_t mix = (crc ^ b) & 0x01;

            crc >>= 1;
            if (mix)
            {
                crc ^= 0x8C;
            }
            b >>= 1;
        }
    }
    return crc;
}

/* ------------------------------ 对外接口 ------------------------------ */
rt_err_t ds18b20_init(rt_base_t pin)
{
    s_pin = pin;

    rt_pin_mode(s_pin, PIN_MODE_OUTPUT_OD);
    ow_release();
    rt_thread_mdelay(2);

    s_present = ow_reset();
    if (!s_present)
    {
        LOG_W("no device on 1-wire bus (检查 DQ 线 / 4.7k 上拉 / VDD-GND)");
        return -RT_EIO;
    }

    LOG_I("init ok");
    return RT_EOK;
}

rt_bool_t ds18b20_present(void)
{
    return s_present;
}

rt_err_t ds18b20_read(float *temp_c)
{
    rt_uint8_t scratch[9];
    rt_int16_t raw;
    float temp;
    rt_uint8_t i;

    if (s_pin < 0 || temp_c == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /* 1) 启动一次转换 */
    if (!ow_reset())
    {
        s_present = RT_FALSE;
        return -RT_EIO;
    }
    ow_write_byte(OW_CMD_SKIP_ROM);
    ow_write_byte(OW_CMD_CONVERT_T);

    /* 12 位分辨率最长 750ms */
    rt_thread_mdelay(800);

    /* 2) 读暂存器 */
    if (!ow_reset())
    {
        s_present = RT_FALSE;
        return -RT_EIO;
    }
    ow_write_byte(OW_CMD_SKIP_ROM);
    ow_write_byte(OW_CMD_READ_SCRATCH);

    for (i = 0; i < 9; i++)
    {
        scratch[i] = ow_read_byte();
    }

    if (ow_crc8(scratch, 8) != scratch[8])
    {
        LOG_W("crc error (读到的数据不可信)");
        return -RT_EIO;
    }

    raw = (rt_int16_t)((scratch[1] << 8) | scratch[0]);
    temp = (float)raw * 0.0625f;

    /* 85.0 是上电默认值：多半是没转换成功 */
    if (temp > 84.9f && temp < 85.1f)
    {
        LOG_D("got 85.0 (power-on default)");
    }
    /* -127 表示总线被拉低/器件没响应 */
    if (temp < -100.0f)
    {
        s_present = RT_FALSE;
        return -RT_EIO;
    }

    s_present = RT_TRUE;
    *temp_c = temp;
    return RT_EOK;
}
