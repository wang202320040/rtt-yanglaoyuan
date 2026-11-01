/*
 * drv_bt.h —— 蓝牙模块（HC-05 / HC-06 / JDY-31）串口透传驱动
 *
 * 硬件：USART1，TX=PF13(P1-8)，RX=PF12(P1-10)，默认 9600 8N1
 * 用法：
 *     drv_bt_init();
 *     drv_bt_send_line("{\"hr\":75}");
 *     while (drv_bt_read_line(line, sizeof(line))) { 处理命令 }
 */
#ifndef __DRV_BT_H__
#define __DRV_BT_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t drv_bt_init(void);
rt_bool_t drv_bt_is_ready(void);

/* 发一行（自动补 \r\n），返回实际写出的字节数 */
rt_size_t drv_bt_send_line(const char *line);

/* 取一行（非阻塞）。返回 RT_TRUE 表示取到一行（不含 \r\n） */
rt_bool_t drv_bt_read_line(char *line, rt_size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_BT_H__ */
