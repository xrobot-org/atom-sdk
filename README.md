# ATOM-IMU 模块 V4.5

![imu](./img/imu.jpg)

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
  - `1: SYNC`
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
└── stm32_can_example       `STM32 CAN/CANFD 解析示例`
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

VERSION=`rolling`

#### 发布

```shell
colcon build
source install/setup.zsh
ros2 run imu_publisher node_imu
```

#### 订阅

```shell
colcon build
source install/setup.zsh
ros2 run imu_subscriber node_imu
```

### **RViz 可视化**

使用 `ros_rviz_example.rviz` 在 RViz2 中打开。

### **STM32 CAN 示例**

```shell
make
```

## **终端交互**

使用 `picocom`、`putty`、`MobaXTerm` 等工具进行交互，以460800波特率连接USB枚举出的第一个串口设备即可。例如 `USB-Enhanced-Serial-A CH342` 或 `/dev/ttyCH343USB0`。

示例操作：

```shell
linux@XRobot:~$ ls /dev/ttyCH*
/dev/ttyCH343USB0  /dev/ttyCH343USB1

linux@XRobot:~$ picocom /dev/ttyCH343USB0 -b 460800
```

回车后示例输出：

```shell
XRobot:/$
```

## **IMU 设置**

```shell
# 输入set_imu命令并回车
XRobot:/$ set_imu
# 这一行显示IMU CAN/CANFD输出的状态，CAN/CANFD/UART输出同时只能有一个开启
can/canfd output disabled.
# 这一行显示IMU 串口输出的状态
uart output enabled.
# 这一行表示帧同步信号的检测模式，第一个数字0表示不检测，1表示检测上升沿，2表示检测下降沿，3表示同时检测上升沿和下降沿
# 第二个数字代表最近一次帧同步信号的时间，单位微秒
FSYNC:0 0
# 这一行显示两次反馈数据的时间间隔，单位毫秒
feedback delay:1
# 这一行显示IMU数据帧的ID
id:48

Usage:
        set_delay  [time]  设置发送延时ms
        set_can_id [id]    设置can id
        # accl/gyro/quat/eulr只对can模式发送有效
        enable/disable     [accl/gyro/quat/eulr/canfd/can/uart]
        fsync              [0: disable 1: rise 2: fall 3: both]           设置fsync模式
```

## **校准（Calibration）**

完整校准过程约需 **20 分钟**，过程中需要保证 IMU 稳定，每次更改IMU方向后需要等待1分钟以上来使角度稳定。校准的每一步都无顺序要求，可以多次尝试。

```shell
# 上电后等待十分钟，IMU预热
XRobot:/$ bmi088 show 600000 1000
# IMU平放，LOGO面朝上
XRobot:/$ bmi088 cali
...
# 校准误差绝对值在0.00003以下视为校准成功，理想情况下应当小于0.000015
Calibration error -0.000013
Calibration data saved.
# 更改方向，USB接口面朝上
XRobot:/$ bmi088 cali
...
# 侧放IMU，USB接口与XT30接口靠近桌面
XRobot:/$ bmi088 cali
...
# 再次将IMU平放，LOGO面朝上
XRobot:/$ icm42688 cali
...
# 搜索得到当地的经纬度，可在谷歌地图中直接右键复制。例如格拉斯哥的经纬度为：55.87241068336635 -4.290120205979219
XRobot:/$ ahrs set_location 55.87241068336635 -4.290120205979219
Done.
# 校准完成
```

## **测量零漂**

仅作参考，以实际情况为准

```shell
XRobot:/$ ahrs test
Please keep the device steady, start measurement
Please wait
Zero offset:-0.035189°/min
```

## **VOFA+ 可视化数据**

请添加以下自定义命令，使用VOFA+可视化数据。
数据分别为：`w x y z pitch roll yaw`

```shell
ahrs print_quat 1000000 100\r\n
```

## 3D 模型

[Top Model](./3D/imu_top.step)

[Middle Model](./3D/imu_mid.step)

[Bottom Model](./3D/imu_bottom.step)

![View](./img/xrobot-atom.png)

## **传输协议**

### **UART 协议**

- 数据帧包括：前缀，微秒时间戳，四元数，角速度，加速度，欧拉角，CRC8校验。

```c++
typedef struct __attribute__((packed)) {
  float x;
  float y;
  float z;
} Vector3;

typedef struct __attribute__((packed)) {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef struct __attribute__((packed)) {
  float yaw;
  float pit;
  float rol;
} EulerAngles;

typedef struct __attribute__((packed))
{
  uint8_t prefix;
  uint64_t time : 40;
  uint64_t sync : 40;
  Quaternion quat_;
  Vector3 gyro_;
  Vector3 accl_;
  EulerAngles eulr_;
  uint8_t crc8;
} Data;
```

### **CAN 协议**

```c++
#define ENCODER_21_MAX_INT ((1u << 21) - 1)
#define CAN_PACK_ID_ACCL 0
#define CAN_PACK_ID_GYRO 1
#define CAN_PACK_ID_EULR 3
#define CAN_PACK_ID_QUAT 4

typedef union {
  struct __attribute__((packed)) {
    int32_t data1 : 21;
    int32_t data2 : 21;
    int32_t data3 : 21;
    int32_t res : 1;
  };
  struct __attribute__((packed)) {
    uint32_t data1_unsigned : 21;
    uint32_t data2_unsigned : 21;
    uint32_t data3_unsigned : 21;
    uint32_t res_unsigned : 1;
  };
  uint8_t raw[8];
} CanData3;

typedef struct __attribute__((packed)) {
  union {
    int16_t data[4];
    uint16_t data_unsigned[4];
  };
} CanData4;

typedef struct {
  struct {
    float x, y, z;
  } accl;
  struct {
    float x, y, z;
  } gyro;
  struct {
    float pitch, roll, yaw;
  } eulr;
  struct {
    float w, x, y, z;
  } quat;
  uint64_t timestamp;
  uint64_t sync_time;
} ImuData;

ImuData imu_data;

static float DecodeFloat21(uint32_t encoded, float min, float max) {
  float norm =
      (float)(encoded & ENCODER_21_MAX_INT) / (float)ENCODER_21_MAX_INT;
  return min + norm * (max - min);
}

static float DecodeInt16Normalized(int16_t value) {
  return (float)value / (float)INT16_MAX;
}

static void ProcessClassicCanPacket(uint32_t id, uint8_t *data) {
  uint32_t packet_type = id - IMU_DEVICE_ID;

  switch (packet_type) {
  case CAN_PACK_ID_ACCL: {
    /* Accelerometer data: ±24g range */
    CanData3 *can_data = (CanData3 *)data;
    imu_data.accl.x = DecodeFloat21(can_data->data1_unsigned, -24.0f, 24.0f);
    imu_data.accl.y = DecodeFloat21(can_data->data2_unsigned, -24.0f, 24.0f);
    imu_data.accl.z = DecodeFloat21(can_data->data3_unsigned, -24.0f, 24.0f);
    break;
  }

  case CAN_PACK_ID_GYRO: {
    /* Gyroscope data: ±2000 deg/s converted to rad/s */
    CanData3 *can_data = (CanData3 *)data;
    float min_gyro = -2000.0f * M_PI / 180.0f;
    float max_gyro = 2000.0f * M_PI / 180.0f;
    imu_data.gyro.x =
        DecodeFloat21(can_data->data1_unsigned, min_gyro, max_gyro);
    imu_data.gyro.y =
        DecodeFloat21(can_data->data2_unsigned, min_gyro, max_gyro);
    imu_data.gyro.z =
        DecodeFloat21(can_data->data3_unsigned, min_gyro, max_gyro);
    break;
  }

  case CAN_PACK_ID_EULR: {
    /* Euler angles: ±π rad */
    CanData3 *can_data = (CanData3 *)data;
    imu_data.eulr.pitch = DecodeFloat21(can_data->data1_unsigned, -M_PI, M_PI);
    imu_data.eulr.roll = DecodeFloat21(can_data->data2_unsigned, -M_PI, M_PI);
    imu_data.eulr.yaw = DecodeFloat21(can_data->data3_unsigned, -M_PI, M_PI);
    break;
  }

  case CAN_PACK_ID_QUAT: {
    /* Quaternion data: normalized int16 */
    CanData4 *can_data = (CanData4 *)data;
    imu_data.quat.w = DecodeInt16Normalized(can_data->data[0]);
    imu_data.quat.x = DecodeInt16Normalized(can_data->data[1]);
    imu_data.quat.y = DecodeInt16Normalized(can_data->data[2]);
    imu_data.quat.z = DecodeInt16Normalized(can_data->data[3]);
    break;
  }

  default:
    /* Unknown packet type */
    break;
  }
}
```

### **CANFD 结构**

```c++
typedef struct __attribute__((packed)) {
  uint64_t time : 48;
  uint64_t sync : 48;
  float quat[4]; /* w, x, y, z */
  float gyro[3]; /* x, y, z */
  float accl[3]; /* x, y, z */
  float eulr[3]; /* pitch, roll, yaw */
} CanfdData;
```

## **固件更新**

<font color=red>

⚠ **请勿擦除整个 Flash！**

⚠ **此分支只支持 2025/9/10 以后购买的版本**
</font>

1. 获取最新固件：参考 `firmware` 目录。
2. 进入 Bootloader 模式：

   ```sh
   power bl
   ```
  
3. 使用 `STM32CubeProgrammer` 或 `stm32flash` 进行 UART 刷写。
4. 重启

## **相关资源**

- [Bilibili 视频演示](https://www.bilibili.com/video/BV1iespeLE5S/?share_source=copy_web)

![XRobot](./img/XRobot.jpeg)
