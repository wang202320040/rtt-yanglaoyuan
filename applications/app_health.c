/*
 * app_health.c —— 五个采集线程 + 数据快照 + 蓝牙上报/命令 + msh 调试命令
 *
 * 线程表：
 *   max30102  优先级 8   50ms 轮询一次 FIFO（内部 100Hz 采样，4s 窗口算一次）
 *   mpu6050   优先级 9   20ms 轮询（50Hz），顺带计步
 *   ds18b20   优先级 11  约 1.8s 读一次（转换要 800ms）
 *   dht11     优先级 12  2s 读一次环境温湿度（DHT11 不允许更快）
 *   bt        优先级 14  1s 上报一行 JSON，同时处理收到的命令
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>

#include "app_pins.h"
#include "app_health.h"
#include "drv_max30102.h"
#include "drv_mpu6050.h"
#include "drv_ds18b20.h"
#include "drv_dht11.h"
#include "drv_bt.h"

#define DBG_TAG "health"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct health_snapshot s_snap;
static struct rt_mutex       s_lock;

/* ------------------------------ 数据更新 ------------------------------ */
void health_get(struct health_snapshot *out)
{
    if (out == RT_NULL)
    {
        return;
    }
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    *out = s_snap;
    rt_mutex_release(&s_lock);
}

void health_reset_steps(void)
{
    mpu6050_reset_steps();

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_snap.steps = 0;
    rt_mutex_release(&s_lock);
}

/* ------------------------------ 采集线程 ------------------------------ */
static void max30102_thread_entry(void *parameter)
{
    (void)parameter;

    while (1)
    {
        max30102_poll();
        rt_thread_mdelay(50);
    }
}

static void mpu6050_thread_entry(void *parameter)
{

    (void)parameter;

    while (1)
    {
        mpu6050_poll();
        rt_thread_mdelay(20);
    }
}

static void dht11_thread_entry(void *parameter)
{
    (void)parameter;

    rt_int8_t  t  = 0;
    rt_uint8_t rh = 0;

    while (1)
    {
        rt_err_t ret = dht11_read(&t, &rh);

        rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
        if (ret == RT_EOK)
        {
            s_snap.room_temp = t;
            s_snap.rh        = rh;
            s_snap.env_valid = 1;
            s_snap.dht_ok    = 1;
        }
        else
        {
            s_snap.env_valid = 0;
            s_snap.dht_ok    = 0;
        }
        rt_mutex_release(&s_lock);

        rt_thread_mdelay(APP_DHT11_PERIOD_MS);
    }
}

static void ds18b20_thread_entry(void *parameter)
{


    (void)parameter;

    float t;

    while (1)
    {
        rt_err_t ret = ds18b20_read(&t);

        rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
        if (ret == RT_EOK)
        {
            s_snap.temp       = t;
            s_snap.temp_valid = 1;
            s_snap.ds18b20_ok = 1;
        }
        else
        {
            s_snap.temp_valid = 0;
            s_snap.ds18b20_ok = 0;
        }
        rt_mutex_release(&s_lock);

        rt_thread_mdelay(1000);
    }
}

/* 把 -0.1 这种数拼成字符串，避免依赖浮点 printf */
static void temp_to_str(float t, char *buf, rt_size_t size)
{
    rt_int32_t t10 = (rt_int32_t)(t * 10.0f + (t >= 0.0f ? 0.5f : -0.5f));
    const char *sign = "";

    if (t10 < 0)
    {
        sign = "-";
        t10  = -t10;
    }
    rt_snprintf(buf, size, "%s%d.%d", sign, (int)(t10 / 10), (int)(t10 % 10));
}

static void bt_report(const struct health_snapshot *s)
{
    char line[176];
    char tmp[16];
    char rtmp[8];
    char rhtmp[8];

    if (s->temp_valid)
    {
        temp_to_str(s->temp, tmp, sizeof(tmp));
    }
    else
    {
        rt_snprintf(tmp, sizeof(tmp), "null");
    }

    if (s->env_valid)
    {
        rt_snprintf(rtmp, sizeof(rtmp), "%d", s->room_temp);
        rt_snprintf(rhtmp, sizeof(rhtmp), "%d", s->rh);
    }
    else
    {
        rt_snprintf(rtmp, sizeof(rtmp), "null");
        rt_snprintf(rhtmp, sizeof(rhtmp), "null");
    }

    if (s->hr_valid)
    {
        rt_snprintf(line, sizeof(line),
                    "{\"hr\":%d,\"spo2\":%d,\"q\":%d,\"steps\":%d,\"cad\":%d,\"temp\":%s,\"rt\":%s,\"rh\":%s,\"up\":%d}",
                    s->hr, s->spo2, s->hr_quality, s->steps, s->cadence, tmp, rtmp, rhtmp, s->uptime_s);
    }
    else
    {
        rt_snprintf(line, sizeof(line),
                    "{\"hr\":null,\"spo2\":null,\"q\":0,\"steps\":%d,\"cad\":%d,\"temp\":%s,\"rt\":%s,\"rh\":%s,\"up\":%d}",
                    s->steps, s->cadence, tmp, rtmp, rhtmp, s->uptime_s);
    }

    drv_bt_send_line(line);
}

static void bt_thread_entry(void *parameter)
{

    (void)parameter;

    char line[64];
    struct health_snapshot s;

    while (1)
    {
        /* 0) 刷新一次快照（步数、心率结果、运行时长） */
        health_tick();

        /* 1) 收命令 */
        while (drv_bt_read_line(line, sizeof(line)))
        {
            LOG_I("cmd: %s", line);
            if (rt_strcmp(line, "GET") == 0)
            {
                health_get(&s);
                bt_report(&s);
            }
            else if (rt_strcmp(line, "RESET") == 0)
            {
                health_reset_steps();
                drv_bt_send_line("{\"ack\":\"reset\"}");
            }
            else if (rt_strcmp(line, "PING") == 0)
            {
                drv_bt_send_line("{\"ack\":\"pong\"}");
            }
            else if (line[0] != '\0')
            {
                drv_bt_send_line("{\"ack\":\"unknown\"}");
            }
        }

        /* 2) 每秒上报 */
        health_get(&s);
        bt_report(&s);

        rt_thread_mdelay(1000);
    }
}

/* ------------------------------ msh 调试命令 ------------------------------ */
static void health_dump(void)
{
    struct health_snapshot s;

    health_get(&s);

    if (s.hr_valid)
    {
        rt_kprintf("[health] hr=%d bpm  spo2=%d%%  q=%d\n", s.hr, s.spo2, s.hr_quality);
    }
    else
    {
        rt_kprintf("[health] hr=--  spo2=--  (没手指 / 信号弱)\n");
    }

    if (s.temp_valid)
    {
        char tmp[16];

        temp_to_str(s.temp, tmp, sizeof(tmp));
        rt_kprintf("[health] temp=%s C\n", tmp);
    }
    else
    {
        rt_kprintf("[health] temp=--.- C  (DS18B20 没读上)\n");
    }

    if (s.env_valid)
    {
        const char *hint = "";

        if (s.rh < RH_LEVEL_DRY)
        {
            hint = "  <-- 太干，注意呼吸道/静电";
        }
        else if (s.rh > RH_LEVEL_WET)
        {
            hint = "  <-- 太湿，注意地面滑/发霉";
        }
        rt_kprintf("[health] room=%d C  RH=%d%%%s\n", s.room_temp, s.rh, hint);
    }
    else
    {
        rt_kprintf("[health] room=-- C  RH=--%%  (DHT11 没读上)\n");
    }

    rt_kprintf("[health] steps=%d  cadence=%d/min  uptime=%d s\n", s.steps, s.cadence, s.uptime_s);
    rt_kprintf("[health] max30102=%s  mpu6050=%s  ds18b20=%s  dht11=%s  bt=%s\n",
               s.max_ok ? "OK" : "FAIL",
               s.mpu_ok ? "OK" : "FAIL",
               s.ds18b20_ok ? "OK" : "FAIL",
               s.dht_ok ? "OK" : "FAIL",
               drv_bt_is_ready() ? "OK" : "FAIL");
}
MSH_CMD_EXPORT_ALIAS(health_dump, health, show health band status);

static void steps_reset(void)
{
    health_reset_steps();
    rt_kprintf("steps cleared\n");
}
MSH_CMD_EXPORT(steps_reset, reset step counter);

/* ------------------------------ 初始化 ------------------------------ */
rt_err_t health_init(void)
{
    rt_thread_t tid;
    rt_err_t ret = RT_EOK;

    rt_memset(&s_snap, 0, sizeof(s_snap));
    rt_mutex_init(&s_lock, "hlth", RT_IPC_FLAG_PRIO);

    /* 传感器：一个失败不影响其它的，方便单点排障 */
    if (max30102_init() == RT_EOK)
    {
        s_snap.max_ok = 1;
    }
    else
    {
        LOG_W("MAX30102 未就绪（先查 P1-3/P1-5 和 3.3V）");
    }

    if (mpu6050_init() == RT_EOK)
    {
        s_snap.mpu_ok = 1;
    }
    else
    {
        LOG_W("MPU6050 未就绪");
    }

    if (ds18b20_init(APP_DS18B20_PIN) == RT_EOK)
    {
        s_snap.ds18b20_ok = 1;
    }
    else
    {
        LOG_W("DS18B20 未就绪（查 P1-11、4.7k 上拉）");
    }

    if (dht11_init() == RT_EOK)
    {
        s_snap.dht_ok = 1;
    }
    else
    {
        LOG_W("DHT11 未就绪（查 P1-13、4.7k 上拉）");
    }

    if (drv_bt_init() == RT_EOK)
    {
        drv_bt_send_line("{\"boot\":\"health-band\",\"ver\":\"0.1\"}");
    }
    else
    {
        LOG_W("蓝牙串口未就绪");
    }

    /* 采集线程 */
    if (s_snap.max_ok)
    {
        tid = rt_thread_create("max30102", max30102_thread_entry, RT_NULL,
                               THREAD_STACK_SIZE, THREAD_PRIO_MAX30102, 10);
        if (tid) rt_thread_startup(tid);
        else     ret = -RT_ERROR;
    }

    if (s_snap.mpu_ok)
    {
        tid = rt_thread_create("mpu6050", mpu6050_thread_entry, RT_NULL,
                               THREAD_STACK_SIZE, THREAD_PRIO_MPU6050, 10);
        if (tid) rt_thread_startup(tid);
        else     ret = -RT_ERROR;
    }

    if (s_snap.ds18b20_ok)
    {
        tid = rt_thread_create("ds18b20", ds18b20_thread_entry, RT_NULL,
                               THREAD_STACK_SIZE, THREAD_PRIO_DS18B20, 10);
        if (tid) rt_thread_startup(tid);
        else     ret = -RT_ERROR;
    }

    if (s_snap.dht_ok)
    {
        tid = rt_thread_create("dht11", dht11_thread_entry, RT_NULL,
                               THREAD_STACK_SIZE, THREAD_PRIO_DHT11, 10);
        if (tid) rt_thread_startup(tid);
        else     ret = -RT_ERROR;
    }

    /* 蓝牙线程：只要串口通了就跑（没有传感器也能上报空数据，方便先测通路） */
    if (drv_bt_is_ready())
    {
        tid = rt_thread_create("bt", bt_thread_entry, RT_NULL,
                               THREAD_STACK_SIZE, THREAD_PRIO_BT, 10);
        if (tid) rt_thread_startup(tid);
        else     ret = -RT_ERROR;
    }

    /* 1 秒一次的 uptime / 数据同步线程可以并到 bt 线程里，这里只留个轻量统计 */
    LOG_I("health_init done (max=%d mpu=%d ds=%d dht=%d bt=%d)",
          s_snap.max_ok, s_snap.mpu_ok, s_snap.ds18b20_ok, s_snap.dht_ok, drv_bt_is_ready());

    return ret;
}

/* 每 1 秒刷新一次快照里的步数/心率/uptime —— 由 bt 线程顺带做，避免多开线程
 * 体温和温湿度是各自线程直接写进去的，不在这里读（它们本身就是秒级慢变量） */
void health_tick(void)
{
    rt_uint16_t hr = 0;
    rt_uint8_t  spo2 = 0, q = 0;

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_snap.uptime_s = rt_tick_get() / RT_TICK_PER_SECOND;
    s_snap.steps    = mpu6050_get_steps();
    s_snap.cadence  = mpu6050_get_cadence();
    s_snap.hr_valid = max30102_get_result(&hr, &spo2, &q) ? 1 : 0;
    s_snap.hr       = hr;
    s_snap.spo2     = spo2;
    s_snap.hr_quality = q;
    rt_mutex_release(&s_lock);
}

