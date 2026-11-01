# rtt — ART-Pi II 多生命体征监测终端（手环端）

基于 RT-Thread 5.x 与 ART-Pi II（STM32H7R7L8H6H，Cortex-M7 @480MHz）的硬件端工程：
采集心率、血氧、步数、体表温度、环境温湿度五路数据，通过蓝牙以 JSON 行每秒上报一次。

## 硬件

| 模块 | 型号 | 接口 | 地址 |
|---|---|---|---|
| 心率血氧 | MAX30102 | I2C | 0x57 |
| 加速度（计步） | MPU6050 | I2C | 0x68 |
| 体表温度 | DS18B20 | 1-Wire | — |
| 环境温湿度 | DHT11 | 私有单总线 | — |
| 蓝牙 | HC-05 | USART1 9600 8N1 | — |

全部接在 P1（2×20 树莓派兼容排针）上，不画转接板。占用 13 个针脚、18 根杜邦线、2 个上拉电阻。

| 信号 | 排针 | MCU 引脚 |
|---|---|---|
| I2C SDA | P1-3 | PB9 |
| I2C SCL | P1-5 | PB8 |
| DS18B20 DQ | P1-11 | PC3（需 4.7k 上拉到 3V3） |
| DHT11 DATA | P1-13 | PC2（需 4.7k~10k 上拉到 3V3） |
| HC-05 TXD | P1-10 | PF12（USART1_RX） |
| HC-05 RXD | P1-8 | PF13（USART1_TX） |
| 3V3 / 5V / GND | P1-1、P1-17 / P1-2 / P1-6、P1-9、P1-14、P1-20 | — |

> P1-3 / P1-5 底板上已焊 R23 / R24 两个 4.7kΩ 上拉到 3V3；P1-8 / P1-10 丝印的 `#IO14_UART_TXD` 是树莓派 BCM 编号，实际为 PF13 / PF12 = USART1。

## 目录

```
applications/        main.c、app_health.c/.h（线程、快照、JSON、msh 命令）
drivers/             drv_max30102 / drv_mpu6050 / drv_ds18b20 / drv_dht11 / drv_bt
board/               app_pins.h（全部引脚与参数）；board_h_片段.md（BSP 配置补丁）
```

## 线程

| 线程 | 优先级 | 周期 | 说明 |
|---|---|---|---|
| max30102 | 8 | 50ms | 读 FIFO（100Hz 采样），4 秒滑窗算心率血氧 |
| mpu6050 | 9 | 20ms | 50Hz 读加速度并计步 |
| ds18b20 | 11 | ~1.8s | 单总线读温度，CRC8 校验 |
| dht11 | 12 | 2s | 40bit 单总线帧，校验和通过才更新 |
| bt | 14 | 1s | 拼 JSON 上报，解析 GET / RESET / PING |

数据统一写入 `health_snapshot`（互斥量保护），上报线程只取快照，不直接访问驱动。

## 数据格式

```json
{"hr":76,"spo2":97,"q":82,"steps":1234,"cad":0,"temp":36.4,"rt":26,"rh":58,"up":512}
```

`hr`/`spo2`/`q` 心率血氧与信号质量，`steps`/`cad` 步数与步频，`temp` 体表温度（DS18B20），`rt`/`rh` 环境温湿度（DHT11），`up` 运行秒数。无有效值时字段为 `null`。

## 编译

1. RT-Thread Studio 2.2.9 + ART-Pi2 SDK 包，新建 `art_pi_blink_led` 工程并先跑通点灯；
2. 将本仓库 `applications/`、`drivers/`、`board/` 拷入工程，覆盖 `main.c` 与 `SConscript`；
3. 按 `board/board_h_片段.md` 打开 `BSP_USING_I2C1`、`BSP_USING_UART1`，并在 menuconfig 中启用 I2C bit-ops；
4. 编译下载，串口 115200，输入 `health` 查看五路数据与各模块自检结果。

## 说明

心率血氧为演示级算法，温湿度为趋势级读数，均不作为医疗或计量依据。
