# ATOM-IMU 模块（基于 XRobot）

![imu](./img/imu.jpg)

[English](./README.md) | [简体中文](./README.zh-CN.md)

## **参数**

- **输出频率**：1-1000Hz
- **支持电压**：5V/24V
- **接口**：USB / UART / CANFD
- **支持数据类型**：加速度计 (ACC) / 陀螺仪 (GYRO) / 欧拉角 (EULR) / 四元数 (QUAT)
- **UART 波特率**：460800 (终端) / 1M (数据)
- **CAN 波特率**：1M / 5M
- **陀螺仪**：最大量程 ±2000DPS，分辨率 0.015DPS
- **加速度计**：最大量程 ±24G，分辨率 0.0001G

## **输出测试**

![data](./img/data.png)

## **连接方式**

- **USB-CH342**：`UART_DATA(1M) UART_TERMINAL(460800)`
- **UART 1.25 4P**：
  - `1: RX`
  - `2: TX`
  - `3: GND`
  - `4: +5V_IN`
- **CAN 1.25 2P**：
  - `1: CANL`
  - `2: CANH`
- **XT30 供电**：`24V_IN`

## **示例代码**

```shell
├── linux_uart_example      `Linux UART 解析示例`
├── ros_imu_publisher       `ROS IMU 发布节点`
├── ros_imu_subscriber      `ROS IMU 订阅节点`
├── ros_rviz_example.rviz   `ROS RViz 可视化`
└── stm32_can_example       `STM32 CANFD 解析示例`
```

### **Linux UART 示例**

![linux_uart_example](./img/linux_uart_example.png)

请使用 [wch 官方 USB 驱动](https://github.com/WCHSoftGroup/ch343ser_linux) 以确保稳定性。

```shell
# 安装驱动
sudo apt install mokutil shim-signed
sudo update-secureboot-policy --new-key
openssl req -new -x509 -newkey rsa:2048 -keyout MOK.priv -outform DER -out MOK.der -nodes -days 36500 -subj "/CN=Descriptive name/"
sudo mokutil --import /var/lib/shim-signed/mok/MOK.der

# 重启并注册 MOK
reboot

# 下载并编译驱动
git clone https://github.com/WCHSoftGroup/ch343ser_linux
cd ch343ser_linux/driver
make

# 签名驱动
sudo /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 /var/lib/shim-signed/mok/MOK.priv /var/lib/shim-signed/mok/MOK.der ch343.ko

# 安装驱动
sudo make install
reboot
```

编译并运行示例：

```shell
gcc main.c -o main
./main
```

### **ROS 示例**

```shell
colcon build
ros2 run imu_publisher node_imu
```

```shell
colcon build
ros2 run imu_subscriber node_imu
```

### **RViz 可视化**

使用 `ros_rviz_example.rviz` 在 RViz2 中打开。

### **STM32 CAN 示例**

```shell
make
```

## **终端交互**

```shell
linux@XRobot:~$ ls /dev/ttyCH*
/dev/ttyCH343USB0  /dev/ttyCH343USB1

linux@XRobot:~$ picocom /dev/ttyCH343USB0 -b 460800
```

示例输出：

```shell
 __  __ _      _ ___ _        _ _
|  \/  (_)_ _ (_) __| |_  ___| | |
| |\/| | | ' \| \__ \ ' \/ -_) | |
|_|  |_|_|_||_|_|___/_||_\___|_|_|
Build:Aug 31 2024 23:47:44
version:1.0.6

Welcome to use XRobot!
```

## **IMU 设置**

```shell
atom@XRobot:~$ set_imu
# 设定模式：CAN / CANFD
can mode
# 设定数据类型：加速度、陀螺仪、四元数、欧拉角
data:accl,gyro,quat,eulr,
# UART输出使能
uart output enabled.
# CAN_BRIDGE模式(通过uart透传收到的CAN数据，使能会强制设置数据串口为2Mbps)
can bridge mode enabled.
# 设定反馈延迟（单位：毫秒，范围 1-1000）
feedback delay:1
# 设定 CAN ID
id:48

Usage:
 set_delay  [time]  设置发送延时ms
 set_can_id [id]    设置can id
 enable/disable     [accl/gyro/quat/eulr/canfd/can/uart/can_bridge]
 set_uart_baud      [0:9600 1:100000 2:115200 3:460800 4:921600 5:1000000 6:2000000]
 set_uart_parity    [0:None, 1:Odd, 2:Even]

```

## **校准（Calibration）**

完整校准过程约需 **20 分钟**。

```shell
# 依次调整 X/Y/Z 轴方向对齐重力方向，并执行校准命令
atom@XRobot:~$ bmi088 cali
...

# 完成后执行最终校准
atom@XRobot:~$ bmi088 cal_cali
```

## **测量零漂**

```shell
atom@XRobot:~$ /dev/AHRS test
请保持静止，开始检测零漂
请等待...
零漂: -0.120239 度/分钟
```

## **USB to CAN模式**

```shell
atom@XRobot:~$ can
monitor [number: 1-32] [timeout] 监控指定数量的can包
atom@XRobot:~$ can monitor 5 100
CanId ID:00000030 DATA:fe ff 03 00 f6 03 a5 ef
CanId ID:00000031 DATA:fc ff 0d 00 00 00 a5 ef
CanId ID:00000032 DATA:be 3c b6 20 44 8d a5 ef
CanId ID:00000034 DATA:de 3d 60 00 38 00 a5 ef
CanId ID:00000033 DATA:32 00 32 00 77 75 a5 ef
CanId ID:00000030 DATA:fe ff 04 00 f7 03 a5 ef
CanId ID:00000031 DATA:fd ff 0d 00 00 00 a5 ef
CanId ID:00000032 DATA:be 3c b6 20 44 8d a5 ef
CanId ID:00000034 DATA:de 3d 60 00 37 00 a5 ef
CanId ID:00000033 DATA:32 00 32 00 77 75 a5 ef
```

## **VOFA+ 可视化数据**

请添加以下自定义命令：

```shell
/dev/AHRS print_quat 1000000 1\r\n
```

## **传输协议**

### **UART 协议**

UART总共会收到两种类型的数据，分别是数据帧（Data）和CAN透传帧（DataCanToUart）。

- 数据帧包括：前缀，ID，时间戳，四元数，陀螺仪，加速度，CRC8校验。
- CAN透传帧包括：前缀，ID，数据，CRC8校验。

CAN透传帧既可以由控制器发送给本模块，也可以由本模块发送给控制器，实现can接口的拓展。

```cpp
typedef struct __attribute__((packed)) {
  float q0, q1, q2, q3;
} Quaternion;

typedef struct __attribute__((packed)) {
  float x, y, z;
} Vector3;

typedef struct __attribute__((packed)) {
  uint8_t prefix;
  uint8_t id;
  uint32_t time_ms;
  Quaternion quat;
  Vector3 gyro;
  Vector3 accl;
  uint8_t crc8;
} Data;

typedef struct __attribute__((packed)) {
  uint8_t prefix;
  uint32_t id;
  uint8_t data[8];
  uint8_t crc8;
} DataCanToUart;
```

### **CAN 协议**

```cpp
struct __attribute__((packed)) {
  uint32_t id;
  uint16_t data[4];
} can_pack;
```

### **CANFD 结构**

```cpp
typedef struct __attribute__((packed)) {
  uint32_t time;
  Quaternion quat_;
  Vector3 gyro_;
  Vector3 accl_;
} Data;
```

## **固件更新**

⚠ **注意：请勿擦除整个 Flash！**

1. 获取最新固件：参考 `firmware` 目录。
2. 进入 Bootloader 模式：

   ```sh
   power bl
   ```

3. 使用 `STM32CubeProgrammer` 或 `stm32flash` 进行 UART 刷写。

## **相关资源**

- [Bilibili 视频演示](https://www.bilibili.com/video/BV1iespeLE5S/?share_source=copy_web)
- [购买链接](https://mall.bilibili.com/neul-next/index.html?page=mall-up_itemDetail&noTitleBar=1&itemsId=1106251092&from=items_share&msource=items_share)

![XRobot](./img/XRobot.jpeg)
