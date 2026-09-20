# STM32 步进电机控制器

完整工程源码已按轴明确分开，包含头文件、源文件、CubeMX 配置和 Keil 工程。

## 工程

- `X_axis_controller/`：X轴最终控制器，控制两个步进电机和一个光电传感器。
- `Y_axis_controller/`：Y轴最终控制器，控制一个步进电机和两个限位传感器。

打开工程：

- `X_axis_controller/MDK-ARM/bjdj.uvprojx`
- `Y_axis_controller/MDK-ARM/bjdj.uvprojx`

两套程序支持 DM542 的 STEP/DIR 控制、USB CDC 虚拟串口、二进制帧协议和 CRC-8 校验。X轴支持双电机同步运动；Y轴支持两个传感器查询和状态变化自动上报。
