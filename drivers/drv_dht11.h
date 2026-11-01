/*
 * drv_dht11.h —— DHT11 温湿度传感器（单总线私有协议）
 *
 * 接线：VDD->3.3V, GND->GND, DATA->PC2（P1-13），DATA 与 VDD 之间接 4.7k~10k 上拉。
 *
 * 用法（2 秒一次，DHT11 自己不支持更快的采样）：
 *     dht11_init();
 *     while (1) { dht11_read(&t, &rh); rt_thread_mdelay(2000); }
 *
 * 注意：
 *   1. DHT11 精度是 ±2°C / ±5%RH，只能看趋势，不能当计量仪器；
 *   2. 上电后 1 秒内不要读，器件需要稳定；
 *   3. 两次读取间隔必须 ≥ 1 秒（我们给 2 秒）。
 */
#ifndef __DRV_DHT11_H__
#define __DRV_DHT11_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t dht11_init(void);
rt_bool_t dht11_is_ready(void);

/* 读一次：temp_c 整数摄氏度，rh_pct 整数百分比湿度
 * 返回 RT_EOK 表示这一帧校验通过 */
rt_err_t dht11_read(rt_int8_t *temp_c, rt_uint8_t *rh_pct);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_DHT11_H__ */
