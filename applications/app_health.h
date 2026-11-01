/*
 * app_health.h —— 健康数据模型 + 采集线程
 *
 * 上层（蓝牙上报 / msh 命令 / 以后加 OLED、上云）都只跟这个快照打交道。
 */
#ifndef __APP_HEALTH_H__
#define __APP_HEALTH_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

struct health_snapshot
{
    rt_uint16_t hr;             /* 心率 bpm，hr_valid 为 0 时无意义 */
    rt_uint8_t  spo2;           /* 血氧 % */
    rt_uint8_t  hr_quality;     /* 信号质量 0~100 */
    rt_uint8_t  hr_valid;       /* 1 = 有手指且在跳 */

    rt_uint32_t steps;          /* 累计步数 */
    rt_uint16_t cadence;        /* 步/分钟 */

    float       temp;           /* 体表温度（DS18B20），摄氏度 */
    rt_uint8_t  temp_valid;     /* 1 = DS18B20 读到了 */

    rt_int8_t   room_temp;      /* 环境温度（DHT11），整数摄氏度 */
    rt_uint8_t  rh;             /* 环境湿度 %RH */
    rt_uint8_t  env_valid;      /* 1 = DHT11 这一帧校验通过 */

    rt_uint8_t  max_ok;         /* MAX30102 通不通 */
    rt_uint8_t  mpu_ok;         /* MPU6050 通不通 */
    rt_uint8_t  ds18b20_ok;     /* DS18B20 通不通 */
    rt_uint8_t  dht_ok;         /* DHT11 通不通 */

    rt_uint32_t uptime_s;
};

/* 初始化传感器 + 建采集线程（main 里调用一次） */
rt_err_t health_init(void);

/* 取一份带锁的快照 */
void health_get(struct health_snapshot *out);

/* 内部用：把心率/步数/uptime 刷新进快照（bt 线程每秒调一次） */
void health_tick(void);

/* 重置步数 */
void health_reset_steps(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_HEALTH_H__ */
