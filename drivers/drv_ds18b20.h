/*
 * drv_ds18b20.h —— DS18B20 单总线温度传感器
 *
 * 接线：VDD->3V3, GND->GND, DQ->PC3（P1-11），DQ 与 3V3 之间必须接 4.7k 上拉。
 *
 * 用法：
 *     ds18b20_init(APP_DS18B20_PIN);
 *     float t;
 *     if (ds18b20_read(&t) == RT_EOK) { ... }   // 阻塞约 800ms
 */
#ifndef __DRV_DS18B20_H__
#define __DRV_DS18B20_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t ds18b20_init(rt_base_t pin);

/* 读一次温度（摄氏度），阻塞约 800ms（等一次 12 位转换） */
rt_err_t ds18b20_read(float *temp_c);

/* 总线是否检测到器件（上电自检用） */
rt_bool_t ds18b20_present(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_DS18B20_H__ */
