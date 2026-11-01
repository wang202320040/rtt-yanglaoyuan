/*
 * drv_max30102.h —— MAX30102 心率 / 血氧传感器
 *
 * 用法（单线程轮询即可）：
 *     max30102_init();
 *     while (1) { max30102_poll(); rt_thread_mdelay(50); }
 *     max30102_get_result(&hr, &spo2, &quality);
 */
#ifndef __DRV_MAX30102_H__
#define __DRV_MAX30102_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：找 I2C 总线、读 PART_ID、配置 FIFO 和 LED 电流
 * 返回值：RT_EOK 成功；-RT_ERROR 找不到总线；-RT_EIO 器件无应答 */
rt_err_t max30102_init(void);

/* 器件是否初始化成功（PART_ID 读到了 0x15） */
rt_bool_t max30102_is_ready(void);

/* 读一次 FIFO 并喂给算法，建议 50ms 调用一次（内部按 100Hz 采样） */
void max30102_poll(void);

/* 取算法结果：hr_bpm 心率，spo2_pct 血氧，quality 信号质量 0~100
 * 返回 RT_TRUE 表示本次结果有效（手指在位、检测到了搏动） */
rt_bool_t max30102_get_result(rt_uint16_t *hr_bpm, rt_uint8_t *spo2_pct, rt_uint8_t *quality);

/* 清空算法窗口（摘下手指出错时调用） */
void max30102_reset_algo(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_MAX30102_H__ */
