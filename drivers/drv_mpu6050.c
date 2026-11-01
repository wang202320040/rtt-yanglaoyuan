/*
 * drv_mpu6050.c —— MPU6050 驱动 + 计步算法
 *
 * 硬件：I2C（"i2c1"），地址 0x68（AD0 悬空或接地）
 * 采样：50Hz 轮询读取加速度
 *
 * 计步思路（简单但可用）：
 *   1. 算合加速度模长 |a|，用一阶低通得到"重力基线" lp
 *   2. 动态分量 dyn = |a| - lp，走路时是一个个 0.2~0.8g 的波峰
 *   3. 自适应阈值 thr = 0.55 * 最近峰值，限制在 0.10~0.60g
 *   4. dyn 上穿 thr 记一步，跌回 0.4*thr 才允许下一步（滞回），
 *      再加 250ms 不应期，防止一步数成两三步
 *
 * 说明：这是"演示能走数"的算法；要比赛级精度得用频域/多轴融合，
 *      但配合 MPU6050 的 50Hz 采样，走路计步已经够用。
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <math.h>

#include "app_pins.h"
#include "drv_mpu6050.h"

#define DBG_TAG "mpu6050"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* ------------------------------ 寄存器 ------------------------------ */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

#define MPU6050_ID          0x68
#define ACCEL_LSB_PER_G     16384.0f    /* ±2g 量程 */

#define CADENCE_WINDOW_MS   10000       /* 步频统计窗口 10s */

/* ------------------------------ 内部状态 ------------------------------ */
static struct rt_i2c_bus_device *s_i2c   = RT_NULL;
static rt_bool_t  s_ready   = RT_FALSE;
static rt_bool_t  s_inited  = RT_FALSE;

static float      s_ax, s_ay, s_az;     /* 最近一次加速度（g） */
static float      s_lp      = 1.0f;     /* 重力基线 */
static float      s_peak    = 0.0f;     /* 最近动态峰值 */
static rt_bool_t  s_in_peak = RT_FALSE;
static rt_uint32_t s_steps  = 0;
static rt_uint32_t s_last_tick = 0;

/* 步频统计：环形记录最近若干步的时间戳 */
#define CADENCE_SLOTS   32
static rt_uint32_t s_step_tick[CADENCE_SLOTS];
static rt_uint8_t  s_step_idx = 0;

/* ------------------------------ I2C ------------------------------ */
static rt_err_t mpu_write_reg(rt_uint8_t reg, rt_uint8_t val)
{
    rt_int32_t ret = (rt_int32_t)rt_i2c_mem_write(s_i2c, MPU6050_I2C_ADDR, reg, 1, &val, 1);

    return (ret == 1) ? RT_EOK : -RT_EIO;
}

static rt_err_t mpu_read_reg(rt_uint8_t reg, rt_uint8_t *buf, rt_uint16_t len)
{
    rt_int32_t ret = (rt_int32_t)rt_i2c_mem_read(s_i2c, MPU6050_I2C_ADDR, reg, 1, buf, len);

    return (ret == (rt_int32_t)len) ? RT_EOK : -RT_EIO;
}

/* ------------------------------ 计步 ------------------------------ */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void step_update(float mag)
{
    float dyn, thr;
    rt_uint32_t now = rt_tick_get();
    rt_uint32_t interval_ms = (now - s_last_tick) * 1000u / RT_TICK_PER_SECOND;

    /* 重力基线：时间常数约 0.4s（50Hz 下 alpha=0.05） */
    s_lp += (mag - s_lp) * 0.05f;

    dyn = mag - s_lp;
    if (dyn < 0.0f)
    {
        dyn = -dyn;                 /* 上下振动都算 */
    }

    /* 峰值包络，缓慢衰减 */
    if (dyn > s_peak)
    {
        s_peak = dyn;
    }
    else
    {
        s_peak *= 0.995f;
    }
    if (s_peak < MPU6050_STEP_THRESHOLD_MIN)
    {
        s_peak = MPU6050_STEP_THRESHOLD_MIN;
    }

    thr = clampf(s_peak * 0.55f, MPU6050_STEP_THRESHOLD_MIN, MPU6050_STEP_THRESHOLD_MAX);

    if (!s_in_peak)
    {
        if (dyn > thr && interval_ms >= MPU6050_STEP_MIN_INTERVAL)
        {
            s_in_peak    = RT_TRUE;
            s_last_tick  = now;
            s_steps++;
            s_step_tick[s_step_idx] = now;
            s_step_idx = (rt_uint8_t)((s_step_idx + 1) % CADENCE_SLOTS);
            LOG_D("step #%d (dyn=%.2fg thr=%.2fg)", s_steps, dyn, thr);
        }
    }
    else
    {
        /* 滞回：跌回阈值 40% 以下才允许下一次 */
        if (dyn < thr * 0.4f)
        {
            s_in_peak = RT_FALSE;
        }
    }
}

/* ------------------------------ 对外接口 ------------------------------ */
rt_err_t mpu6050_init(void)
{
    rt_uint8_t id = 0;

    s_i2c = rt_i2c_bus_device_find(APP_I2C_BUS_NAME);
    if (s_i2c == RT_NULL)
    {
        LOG_E("I2C bus \"%s\" not found!", APP_I2C_BUS_NAME);
        return -RT_ERROR;
    }

    if (mpu_read_reg(REG_WHO_AM_I, &id, 1) != RT_EOK)
    {
        LOG_E("no ack at 0x%02X (SDA/SCL 接反或模块没供电)", MPU6050_I2C_ADDR);
        return -RT_EIO;
    }
    if (id != MPU6050_ID && id != 0x70 && id != 0x72 && id != 0x98)
    {
        LOG_W("WHO_AM_I=0x%02X, expect 0x68", id);
    }

    mpu_write_reg(REG_PWR_MGMT_1, 0x80);        /* 软复位 */
    rt_thread_mdelay(100);
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);        /* 唤醒，内部 8M 时钟 */
    rt_thread_mdelay(10);

    /* 陀螺 1kHz 输出，采样分频 9 -> 100Hz；DLPF 44Hz；±2g */
    mpu_write_reg(REG_SMPLRT_DIV, 9);
    mpu_write_reg(REG_CONFIG, 0x03);
    mpu_write_reg(REG_GYRO_CONFIG, 0x00);
    mpu_write_reg(REG_ACCEL_CONFIG, 0x00);

    rt_memset(s_step_tick, 0, sizeof(s_step_tick));
    s_steps     = 0;
    s_step_idx  = 0;
    s_in_peak   = RT_FALSE;
    s_peak      = MPU6050_STEP_THRESHOLD_MIN;
    s_lp        = 1.0f;
    s_last_tick = rt_tick_get();

    s_inited = RT_TRUE;
    s_ready  = RT_TRUE;

    LOG_I("init ok (WHO_AM_I=0x%02X)", id);
    return RT_EOK;
}

rt_bool_t mpu6050_is_ready(void)
{
    return s_ready;
}

void mpu6050_poll(void)
{
    rt_uint8_t buf[6];
    rt_int16_t ax, ay, az;
    float mag;

    if (!s_inited)
    {
        return;
    }

    if (mpu_read_reg(REG_ACCEL_XOUT_H, buf, 6) != RT_EOK)
    {
        s_ready = RT_FALSE;
        return;
    }
    s_ready = RT_TRUE;

    ax = (rt_int16_t)((buf[0] << 8) | buf[1]);
    ay = (rt_int16_t)((buf[2] << 8) | buf[3]);
    az = (rt_int16_t)((buf[4] << 8) | buf[5]);

    s_ax = (float)ax / ACCEL_LSB_PER_G;
    s_ay = (float)ay / ACCEL_LSB_PER_G;
    s_az = (float)az / ACCEL_LSB_PER_G;

    mag = sqrtf(s_ax * s_ax + s_ay * s_ay + s_az * s_az);
    step_update(mag);
}

rt_uint32_t mpu6050_get_steps(void)
{
    return s_steps;
}

void mpu6050_reset_steps(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    s_steps = 0;
    s_step_idx = 0;
    rt_memset(s_step_tick, 0, sizeof(s_step_tick));
    rt_hw_interrupt_enable(level);
}

rt_uint16_t mpu6050_get_cadence(void)
{
    rt_uint32_t now = rt_tick_get();
    rt_uint32_t window_ticks = (rt_uint32_t)CADENCE_WINDOW_MS * RT_TICK_PER_SECOND / 1000u;
    rt_uint8_t i, cnt = 0;

    for (i = 0; i < CADENCE_SLOTS; i++)
    {
        if (s_step_tick[i] != 0 && (now - s_step_tick[i]) < window_ticks)
        {
            cnt++;
        }
    }

    /* cnt 步 / 10s -> 步/分钟 */
    return (rt_uint16_t)((rt_uint32_t)cnt * 6u);
}

rt_bool_t mpu6050_get_accel(float *x, float *y, float *z)
{
    if (!s_ready)
    {
        return RT_FALSE;
    }
    if (x) *x = s_ax;
    if (y) *y = s_ay;
    if (z) *z = s_az;

    return RT_TRUE;
}
