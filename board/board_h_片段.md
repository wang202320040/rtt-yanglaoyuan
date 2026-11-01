# board.h 需要改的地方（照抄即可）

> 目标：让 BSP 编译出 `i2c1`（软件 I2C，PB8/PB9）和 `uart1`（USART1，PF13/PF12）两个设备；
> 本条只涉及 I2C 和 UART1；DHT11 是纯 GPIO，不需要额外配置。
> 文件位置：你新建的 ART-Pi2 工程里的 `board/board.h`。

## 1. 在 board.h 的"GPIO 引脚定义"区域加上这几行

```c
/* ==================== 本工程新增 ==================== */
/* I2C1（软件 I2C）：P1-5 = PB8 = SCL，P1-3 = PB9 = SDA
 * 一条总线挂两颗芯片：0x57 MAX30102 / 0x68 MPU6050 */
#define BSP_USING_I2C1
#define BSP_I2C1_SCL_PIN                  GET_PIN(B, 8)
#define BSP_I2C1_SDA_PIN                  GET_PIN(B, 9)

/* USART1：P1-8 = PF13 = TX，P1-10 = PF12 = RX（蓝牙模块） */
#define BSP_USING_UART1
#define UART1_CONFIG                                        \
    {                                                       \
        .name = "uart1",                                    \
        .Instance = USART1,                                 \
        .irq_type = USART1_IRQn,                            \
    }
```



> `UARTx_CONFIG` 的字段要和 BSP 里 `drv_usart_v2.c` 的 `struct stm32_uart_config` 对齐；
> 如果工程开了 `RT_SERIAL_USING_DMA`，结构体里还多 `dma_rx`/`dma_tx` 两项，照抄工程里 UART4 的写法最稳。

## 2. board.c 里（可选，但建议加上）

`drv_bt.c` 里已经自己打开了 GPIO 和 USART1 时钟、配好了 AF7，
所以**不改 board.c 也能跑**。如果想按 BSP 的风格来，就在 `rt_hw_board_init()`
调用 `rt_hw_usart_init()` 之前加上：

```c
#ifdef BSP_USING_UART1
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}
#endif
```

## 3. rtconfig.h / menuconfig 需要打开的选项

在 RT-Thread Studio 里点 **RT-Thread Settings**（或命令行 `scons --menuconfig`）：

| 位置 | 选项 | 值 |
|---|---|---|
| RT-Thread Components → Device Drivers | Using serial device drivers | ✅ |
| | Using serial v2 | ✅（BSP 默认） |
| | Using I2C device drivers | ✅ |
| | Using I2C bit-ops | ✅（软件 I2C 必需） |
| | Using pin device drivers | ✅ |
| Hardware Drivers Config → On-chip Peripheral → I2C | Enable I2C1 BUS (soft) | ✅，SCL=PB8，SDA=PB9 |
| Hardware Drivers Config → On-chip Peripheral → UART | Enable UART1 | ✅ |

| RT-Thread Components → Command line | MSH command | ✅（要 `health` 命令就得开） |
| RT-Thread Components → Device Drivers → Using serial | 控制台 UART4 | 保持默认（ST-LINK） |

`rtconfig.h` 里最终应该有这些宏：

```c
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PIN
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V2
#define BSP_USING_I2C1
#define BSP_I2C1_SCL_PIN    GET_PIN(B, 8)
#define BSP_I2C1_SDA_PIN    GET_PIN(B, 9)
#define BSP_USING_UART1

```

## 4. 提醒

- **I2C 必须是"软件 I2C（bit-ops）"**：ART-Pi2 的 BSP 只提供 `drv_soft_i2c.c`，没有硬件 I2C 驱动；引脚可以随便挑，本项目用 PB8/PB9（底板上正好焊了 4.7k 上拉）。
- **两颗 I2C 芯片共总线**：MAX30102(0x57)、MPU6050(0x68) 地址不冲突，不用分线，也不用加多路复用器。DHT11 和 DS18B20 是两根独立的单总线，不在这条 I2C 上。
- **不要多开没配好 GPIO 的串口**：会出现"设备存在但发不出数据"。
- 换 DS18B20 引脚只改 `board/app_pins.h` 的 `APP_DS18B20_PIN`；换 DHT11 引脚改 `APP_DHT11_PIN`。


