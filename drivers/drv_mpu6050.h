/*
 * drv_mpu6050.h —— MPU6050 三轴加速度计 + 计步
 *
 * 用法：
 *     mpu6050_init();
 *     while (1) { mpu6050_poll(); rt_thread_mdelay(20); }   // 50Hz
 *     steps = mpu6050_get_steps();
 *
 * 若是 ADXL345：寄存器不同，需要另写 drv_adxl345.c，引脚接法完全一样。
 */
#ifndef __DRV_MPU6050_H__
#define __DRV_MPU6050_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t mpu6050_init(void);
rt_bool_t mpu6050_is_ready(void);

/* 读一次加速度并做计步判断，建议 20ms 调一次 */
void mpu6050_poll(void);

rt_uint32_t mpu6050_get_steps(void);
void mpu6050_reset_steps(void);

/* 最近 10 秒的步频（步/分钟），0 表示没在走 */
rt_uint16_t mpu6050_get_cadence(void);

/* 最近一次的加速度（单位 g），没有数据时返回 RT_FALSE */
rt_bool_t mpu6050_get_accel(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_MPU6050_H__ */
