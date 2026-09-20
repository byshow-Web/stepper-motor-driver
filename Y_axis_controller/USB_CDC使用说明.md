# USB 虚拟串口（CDC）使用说明

本工程已经把原来的二进制帧协议接到 STM32 原生 USB CDC 上。

## 接线

- USB-C 数据线：STM32 板的 USB-C 口 <-> Jetson USB 口。
- 这根线同时传数据和给 STM32 提供 5V 供电。
- DM542 的 24V 电源仍然只给 DM542 / 电机使用，不能接到 STM32 的 USB 5V。
- 使用 USB 给 STM32 供电时，不要再从另一路 5V 同时给 STM32 供电。
- 不需要 CH340，也不需要接 PA9 / PA10。

## 烧录

烧录文件：`MDK-ARM/bjdj/bjdj.hex`。

先用 ST-Link 烧录；烧录完成后，拔掉 ST-Link 也可以。再用一根确认能传数据的 USB-C 数据线连接 Jetson 与 STM32。

## Jetson 检查

插线后在 Jetson 终端执行：

```bash
dmesg -w
ls /dev/ttyACM*
```

正常会出现 `/dev/ttyACM0`（编号也可能是 ACM1）。USB CDC 不真正使用波特率，但 Python 可以保持原来的写法：

```python
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
```

原来的 `AA 55 ... CRC` 二进制帧、命令号和 CRC-8 算法完全不变，直接把原来写 CH340 串口的数据改写到 `/dev/ttyACM0` 即可。

## 第一个光电传感器测试（PA4）

- 棕线（或红线）接 STM32 的 `5V`。
- 蓝线（或黑线）接 STM32 的 `GND`。
- 黑线（信号线）接 STM32 的 `PA4`。
- 再用一只 `10 kΩ` 电阻连接 `PA4` 与 STM32 的 `3.3V`（短线试验可先依靠内部上拉，但正式使用必须加这只电阻）。

所有 GND 必须共地。这个 NPN 常开传感器检测到物料时会把 PA4 拉低；程序会把“检测到”回传为 `1`，未检测到为 `0`。

发送查询帧（HEX 发送）即可读状态：

```
AA 55 15 00 26
```

返回帧是 `AA 55 82 02 00 状态 CRC`，其中倒数第二组“状态”是 `01` 代表检测到，`00` 代表未检测到。

## 如果电脑 / Jetson 完全没有识别到 USB 设备

先换一根确定能传数据的 USB-C 线。若仍没有新设备，说明这块板子的 USB-C 口可能只接了供电，或 PA11/PA12 没有接到 USB 数据线；此时软件没有问题，需要确认板子原理图 / 更换带原生 USB 数据线的 STM32 板。
