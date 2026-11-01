/*
 * drv_bt.c —— 蓝牙串口（USART1）透传驱动
 *
 * 说明：
 *   1. 这里自己把 PF12/PF13 配成 USART1 的 AF7，并打开 GPIOF/USART1 时钟，
 *      这样不用去改 BSP 模板的 board.c 也不会出现"串口打不开"。
 *      （打开设备时 BSP 的 HAL_UART_Init 会真正初始化外设，此时时钟已经就绪）
 *   2. 接收用中断 + 256 字节环形缓冲，按行（\n）切分，给上层做命令解析。
 */
#include <rtthread.h>
#include <rtdevice.h>

#include "app_pins.h"
#include "drv_bt.h"

#define DBG_TAG "bt"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define BT_RX_RING_SIZE     256
#define BT_LINE_MAX         128

static rt_device_t s_dev = RT_NULL;
static rt_bool_t   s_ready = RT_FALSE;

/* 接收环形缓冲 */
static rt_uint8_t  s_ring[BT_RX_RING_SIZE];
static volatile rt_uint16_t s_head = 0;      /* 写指针（中断里改） */
static volatile rt_uint16_t s_tail = 0;      /* 读指针（线程里改） */

/* ------------------------------ 环形缓冲 ------------------------------ */
static void ring_put(rt_uint8_t ch)
{
    rt_uint16_t next = (rt_uint16_t)((s_head + 1) % BT_RX_RING_SIZE);

    if (next != s_tail)          /* 满了就丢弃新数据 */
    {
        s_ring[s_head] = ch;
        s_head = next;
    }
}

static rt_bool_t ring_get(rt_uint8_t *ch)
{
    if (s_tail == s_head)
    {
        return RT_FALSE;
    }
    *ch = s_ring[s_tail];
    s_tail = (rt_uint16_t)((s_tail + 1) % BT_RX_RING_SIZE);
    return RT_TRUE;
}

/* ------------------------------ USART1 引脚 ------------------------------ */
static void bt_uart_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* PF13 = USART1_TX, PF12 = USART1_RX, 复用功能 AF7 */
    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

/* ------------------------------ 接收回调 ------------------------------ */
static rt_err_t bt_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_uint8_t ch;

    (void)size;

    /* 中断里把数据搬进环形缓冲，尽量读干净 */
    while (rt_device_read(dev, 0, &ch, 1) == 1)
    {
        ring_put(ch);
    }
    return RT_EOK;
}

/* ------------------------------ 对外接口 ------------------------------ */
rt_err_t drv_bt_init(void)
{
    rt_uint8_t tmp;

    bt_uart_gpio_init();

    s_dev = rt_device_find(APP_BT_UART_NAME);
    if (s_dev == RT_NULL)
    {
        LOG_E("device \"%s\" not found! 检查 board.h 是否打开了 BSP_USING_UART1", APP_BT_UART_NAME);
        return -RT_ERROR;
    }

    if (rt_device_open(s_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("open \"%s\" failed", APP_BT_UART_NAME);
        return -RT_EIO;
    }

    /* 配置波特率等参数（bufsz 用默认值，避免 serial v1/v2 结构体字段差异） */
    {
        struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

        cfg.baud_rate = APP_BT_BAUD_RATE;
        cfg.data_bits = DATA_BITS_8;
        cfg.stop_bits = STOP_BITS_1;
        cfg.parity    = PARITY_NONE;
        rt_device_control(s_dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    }

    /* 清一下可能已经收到的残留 */
    while (rt_device_read(s_dev, 0, &tmp, 1) == 1)
    {
    }

    rt_device_set_rx_indicate(s_dev, bt_rx_ind);

    s_ready = RT_TRUE;
    LOG_I("init ok (%s @ 9600 8N1)", APP_BT_UART_NAME);
    return RT_EOK;
}

rt_bool_t drv_bt_is_ready(void)
{
    return s_ready;
}

rt_size_t drv_bt_send_line(const char *line)
{
    rt_size_t len, written;

    if (!s_ready || line == RT_NULL)
    {
        return 0;
    }

    len = rt_strlen(line);
    if (len > BT_LINE_MAX)
    {
        len = BT_LINE_MAX;
    }

    written = rt_device_write(s_dev, 0, line, len);
    written += rt_device_write(s_dev, 0, "\r\n", 2);

    return written;
}

rt_bool_t drv_bt_read_line(char *line, rt_size_t size)
{
    static char  buf[BT_LINE_MAX];
    static rt_uint16_t idx = 0;
    rt_uint8_t ch;

    if (line == RT_NULL || size == 0)
    {
        return RT_FALSE;
    }

    while (ring_get(&ch))
    {
        if (ch == '\n')
        {
            rt_uint16_t i;

            /* 去掉行尾 \r */
            if (idx > 0 && buf[idx - 1] == '\r')
            {
                idx--;
            }
            if (idx >= size)
            {
                idx = (rt_uint16_t)(size - 1);
            }
            for (i = 0; i < idx; i++)
            {
                line[i] = buf[i];
            }
            line[idx] = '\0';
            idx = 0;
            return RT_TRUE;
        }

        if (idx < BT_LINE_MAX - 1)
        {
            buf[idx++] = (char)ch;
        }
        else
        {
            idx = 0;            /* 超长行直接丢弃 */
        }
    }
    return RT_FALSE;
}
