/*
 * drv_max30102.c —— MAX30102 心率 / 血氧驱动 + 演示级算法
 *
 * 硬件：I2C（BSP 软件 I2C，总线名 "i2c1"），地址 0x57
 * 采样：100Hz，FIFO 32 点，每 50ms 读一次 FIFO
 * 算法：4 秒滑窗（400 点）
 *      - 心率：IR 通道去直流后做峰值检测（0.3s 不应期），用峰间隔算 BPM
 *      - 血氧：R = (AC_red/DC_red) / (AC_ir/DC_ir)，SpO2 ≈ 110 - 25R
 * 说明：这是"能演示、能出数"的工程实现，不是医疗级算法；正式产品请用
 *      厂商的 HR/SpO2 库并做标定。
 *
 * 参考：MAX30102 数据手册 Rev.1，寄存器定义见下
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <math.h>

#include "app_pins.h"
#include "drv_max30102.h"

#define DBG_TAG "max30102"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* ------------------------------ 寄存器 ------------------------------ */
#define REG_INTR_STATUS_1       0x00
#define REG_INTR_STATUS_2       0x01
#define REG_INTR_ENABLE_1       0x02
#define REG_INTR_ENABLE_2       0x03
#define REG_FIFO_WR_PTR         0x04
#define REG_OVF_COUNTER         0x05
#define REG_FIFO_RD_PTR         0x06
#define REG_FIFO_DATA           0x07
#define REG_FIFO_CONFIG         0x08
#define REG_MODE_CONFIG         0x09
#define REG_SPO2_CONFIG         0x0A
#define REG_LED1_PA             0x0C    /* RED */
#define REG_LED2_PA             0x0D    /* IR  */
#define REG_PILOT_PA            0x10
#define REG_TEMP_INTR           0x1F
#define REG_TEMP_FRAC           0x20
#define REG_TEMP_CONFIG         0x21
#define REG_PART_ID             0xFF

#define MAX30102_PART_ID_VAL    0x15

/* MODE_CONFIG */
#define MODE_RESET              0x40
#define MODE_SPO2               0x03    /* RED + IR */
/* SPO2_CONFIG: ADC 4096nA | 100Hz | 411us(18bit) */
#define SPO2_CONFIG_VAL         0x27

#define SAMPLE_BYTES            6       /* 3 字节 RED + 3 字节 IR */

/* ------------------------------ 内部状态 ------------------------------ */
static struct rt_i2c_bus_device *s_i2c = RT_NULL;
static rt_bool_t   s_ready   = RT_FALSE;
static rt_bool_t   s_inited  = RT_FALSE;

/* 滑动窗口：环形缓冲，s_head 指向下一个写入位置 */
static rt_uint32_t s_ir_buf[MAX30102_WINDOW_SAMPLES];
static rt_uint32_t s_red_buf[MAX30102_WINDOW_SAMPLES];
static rt_uint32_t s_head    = 0;
static rt_uint32_t s_total   = 0;               /* 累计采样点数 */

/* 算法输出 */
static rt_uint16_t s_hr      = 0;
static rt_uint8_t  s_spo2    = 0;
static rt_uint8_t  s_quality = 0;
static rt_bool_t   s_valid   = RT_FALSE;

/* ------------------------------ I2C 读写 ------------------------------ */
static rt_err_t max_write_reg(rt_uint8_t reg, rt_uint8_t val)
{
    rt_int32_t ret = (rt_int32_t)rt_i2c_mem_write(s_i2c, MAX30102_I2C_ADDR, reg, 1, &val, 1);

    return (ret == 1) ? RT_EOK : -RT_EIO;
}

static rt_err_t max_read_reg(rt_uint8_t reg, rt_uint8_t *buf, rt_uint16_t len)
{
    rt_int32_t ret = (rt_int32_t)rt_i2c_mem_read(s_i2c, MAX30102_I2C_ADDR, reg, 1, buf, len);

    return (ret == (rt_int32_t)len) ? RT_EOK : -RT_EIO;
}

/* ------------------------------ 算法 ------------------------------ */
static void push_sample(rt_uint32_t red, rt_uint32_t ir)
{
    s_ir_buf[s_head]  = ir;
    s_red_buf[s_head] = red;
    s_head = (s_head + 1) % MAX30102_WINDOW_SAMPLES;

    if (s_total < 0xFFFFFF)
    {
        s_total++;
    }
}

/* 逻辑下标 -> 环形缓冲物理下标：k=0 表示窗口里最老的一个样本 */
static rt_uint32_t win_index(rt_uint32_t k, rt_uint32_t n)
{
    return (s_head + MAX30102_WINDOW_SAMPLES - n + k) % MAX30102_WINDOW_SAMPLES;
}

static rt_uint8_t clamp_u8(rt_int32_t v, rt_uint8_t lo, rt_uint8_t hi)
{
    return (rt_uint8_t)(v < lo ? lo : (v > hi ? hi : v));
}

/*
 * 4 秒窗口算一次心率和血氧
 * 没手指 / 信号太弱时把 s_valid 置 RT_FALSE，上层负责显示 "--"
 */
static void algo_compute(void)
{
    rt_uint32_t n = s_total < MAX30102_WINDOW_SAMPLES ? s_total : MAX30102_WINDOW_SAMPLES;
    rt_uint32_t i;
    rt_uint64_t sum_ir = 0, sum_red = 0;
    rt_uint32_t min_ir = 0xFFFFFFFF, max_ir = 0;
    rt_uint32_t min_red = 0xFFFFFFFF, max_red = 0;
    float dc_ir, dc_red, ac_ir, ac_red;
    float ratio, spo2, period, hr;
    rt_uint32_t thr;
    rt_uint16_t peaks = 0;
    rt_uint32_t first = 0, last = 0;
    rt_uint16_t expected;

    if (n < MAX30102_WINDOW_SAMPLES / 2)
    {
        return;                     /* 数据还不够，先等等 */
    }

    for (i = 0; i < n; i++)
    {
        rt_uint32_t idx = win_index(i, n);
        rt_uint32_t v   = s_ir_buf[idx];
        rt_uint32_t r   = s_red_buf[idx];

        sum_ir  += v;
        sum_red += r;
        if (v < min_ir)  min_ir  = v;
        if (v > max_ir)  max_ir  = v;
        if (r < min_red) min_red = r;
        if (r > max_red) max_red = r;
    }

    dc_ir  = (float)sum_ir  / (float)n;
    dc_red = (float)sum_red / (float)n;
    ac_ir  = (float)(max_ir  - min_ir);
    ac_red = (float)(max_red - min_red);

    /* 手指检测：直流分量太小或几乎没有搏动 -> 认为没手指 */
    if (dc_ir < 20000.0f || ac_ir < 400.0f || dc_red < 2000.0f)
    {
        s_valid   = RT_FALSE;
        s_quality = 0;
        return;
    }

    /* 峰值检测：高于 (最小值 + 55% 峰峰值) 的局部极大值才算一次搏动 */
    thr = min_ir + (rt_uint32_t)(ac_ir * 0.55f);
    for (i = 1; i + 1 < n; i++)
    {
        rt_uint32_t v  = s_ir_buf[win_index(i, n)];
        rt_uint32_t vp = s_ir_buf[win_index(i - 1, n)];
        rt_uint32_t vn = s_ir_buf[win_index(i + 1, n)];

        if (v <= thr || v < vp || v <= vn)
        {
            continue;
        }

        /* 0.3s 不应期，防止一个脉搏被数成两次 */
        if (peaks == 0 || (i - last) >= (MAX30102_SAMPLE_RATE_HZ * 3 / 10))
        {
            if (peaks == 0)
            {
                first = i;
            }
            last = i;
            peaks++;
        }
    }

    if (peaks < 3)
    {
        s_valid   = RT_FALSE;
        s_quality = 0;
        return;
    }

    /* 平均峰间隔（样本数）-> BPM */
    period = (float)(last - first) / (float)(peaks - 1);
    if (period < 1.0f)
    {
        s_valid = RT_FALSE;
        return;
    }
    hr = 60.0f * (float)MAX30102_SAMPLE_RATE_HZ / period;

    /* R 比值 -> 血氧（经验公式，演示用） */
    ratio = (ac_red / dc_red) / (ac_ir / dc_ir);
    spo2  = 110.0f - 25.0f * ratio;

    if (hr < 30.0f || hr > 200.0f)
    {
        s_valid = RT_FALSE;
        return;
    }
    s_hr = (rt_uint16_t)(hr + 0.5f);

    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 70.0f)  spo2 = 70.0f;
    s_spo2 = (rt_uint8_t)(spo2 + 0.5f);

    /* 质量：窗口里应有的搏动数 vs 实际数到的心率 */
    expected = (rt_uint16_t)((float)n * hr / 60.0f / (float)MAX30102_SAMPLE_RATE_HZ + 0.5f);
    if (expected == 0)
    {
        expected = 1;
    }
    if (peaks > expected)
    {
        peaks = expected;
    }
    s_quality = clamp_u8((rt_int32_t)peaks * 100 / expected, 0, 100);
    s_valid   = (s_quality >= 30) ? RT_TRUE : RT_FALSE;
}

/* ------------------------------ 对外接口 ------------------------------ */
rt_err_t max30102_init(void)
{
    rt_uint8_t id  = 0;
    rt_uint8_t clr = 0;

    s_i2c = rt_i2c_bus_device_find(APP_I2C_BUS_NAME);
    if (s_i2c == RT_NULL)
    {
        LOG_E("I2C bus \"%s\" not found! 检查 board.h 里的 BSP_USING_I2C1", APP_I2C_BUS_NAME);
        return -RT_ERROR;
    }

    if (max_read_reg(REG_PART_ID, &id, 1) != RT_EOK)
    {
        LOG_E("no ack at 0x%02X (SDA/SCL 接反或模块没供电)", MAX30102_I2C_ADDR);
        return -RT_EIO;
    }
    if (id != MAX30102_PART_ID_VAL)
    {
        LOG_W("PART_ID=0x%02X, expect 0x15 (可能是 MAX30105/兼容片)", id);
    }

    /* 软复位 */
    max_write_reg(REG_MODE_CONFIG, MODE_RESET);
    rt_thread_mdelay(50);

    /* FIFO：4 点平均 + 允许覆盖 + 剩 15 个点中断（我们不用中断，只是别溢出） */
    max_write_reg(REG_FIFO_CONFIG, (0x2 << 5) | (1 << 4) | 0x0F);
    /* 清 FIFO 指针 */
    max_write_reg(REG_FIFO_WR_PTR, 0x00);
    max_write_reg(REG_OVF_COUNTER, 0x00);
    max_write_reg(REG_FIFO_RD_PTR, 0x00);
    /* SpO2 模式：RED + IR */
    max_write_reg(REG_MODE_CONFIG, MODE_SPO2);
    max_write_reg(REG_SPO2_CONFIG, SPO2_CONFIG_VAL);
    /* LED 电流 */
    max_write_reg(REG_LED1_PA, MAX30102_LED_CURRENT);
    max_write_reg(REG_LED2_PA, MAX30102_LED_CURRENT);
    max_write_reg(REG_PILOT_PA, 0x25);
    /* 清中断标志 */
    max_read_reg(REG_INTR_STATUS_1, &clr, 1);
    max_read_reg(REG_INTR_STATUS_2, &clr, 1);

    max30102_reset_algo();
    s_inited = RT_TRUE;
    s_ready  = RT_TRUE;

    LOG_I("init ok (PART_ID=0x%02X)", id);
    return RT_EOK;
}

rt_bool_t max30102_is_ready(void)
{
    return s_ready;
}

void max30102_reset_algo(void)
{
    rt_memset(s_ir_buf,  0, sizeof(s_ir_buf));
    rt_memset(s_red_buf, 0, sizeof(s_red_buf));
    s_head    = 0;
    s_total   = 0;
    s_hr      = 0;
    s_spo2    = 0;
    s_quality = 0;
    s_valid   = RT_FALSE;
}

void max30102_poll(void)
{
    static rt_uint16_t since_calc = 0;
    rt_uint8_t ptr[3];                              /* WR_PTR / OVF / RD_PTR */
    rt_uint8_t raw[MAX30102_FIFO_DEPTH * SAMPLE_BYTES];
    rt_uint16_t num, i;

    if (!s_inited)
    {
        return;
    }

    if (max_read_reg(REG_FIFO_WR_PTR, ptr, 3) != RT_EOK)
    {
        s_ready = RT_FALSE;
        return;
    }

    num = (rt_uint16_t)((ptr[0] + MAX30102_FIFO_DEPTH - ptr[2]) % MAX30102_FIFO_DEPTH);
    if (num == 0)
    {
        return;
    }

    if (ptr[1] != 0)
    {
        LOG_D("FIFO overflow (%d)", ptr[1]);
    }

    if (max_read_reg(REG_FIFO_DATA, raw, (rt_uint16_t)(num * SAMPLE_BYTES)) != RT_EOK)
    {
        s_ready = RT_FALSE;
        return;
    }

    for (i = 0; i < num; i++)
    {
        const rt_uint8_t *p = &raw[i * SAMPLE_BYTES];
        rt_uint32_t red = ((rt_uint32_t)p[0] << 16) | ((rt_uint32_t)p[1] << 8) | p[2];
        rt_uint32_t ir  = ((rt_uint32_t)p[3] << 16) | ((rt_uint32_t)p[4] << 8) | p[5];

        push_sample(red & 0x3FFFF, ir & 0x3FFFF);
    }
    s_ready = RT_TRUE;

    /* 累计到 25 个点（0.25s）算一次，够实时又不至于太占 CPU */
    since_calc += num;
    if (since_calc >= 25)
    {
        since_calc = 0;
        algo_compute();
    }
}

rt_bool_t max30102_get_result(rt_uint16_t *hr_bpm, rt_uint8_t *spo2_pct, rt_uint8_t *quality)
{
    if (hr_bpm)   *hr_bpm   = s_hr;
    if (spo2_pct) *spo2_pct = s_spo2;
    if (quality)  *quality  = s_quality;

    return s_valid;
}
