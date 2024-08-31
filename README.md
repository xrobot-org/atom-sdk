# ATOM-IMU Module based on XRobot

```shell
├── linux_uart_example      `UART Prase Example on Linux`
├── ros_imu_publisher       `IMU Publisher on ROS`
├── ros_imu_subscriber      `IMU Subscriber on ROS`
├── ros_rviz_example.rviz   `IMU Data Visualization on ROS`
└── stm32_can_example       `CAN/CANFD Prase Example on STM32`
```

## linux_uart_example

```shell
gcc main.c -o main
./main
```

## ros_imu_publisher

```shell
colcon build
ros2 run imu_publisher node_imu
```

## ros_imu_subscriber

```shell
colcon build
ros2 run imu_subscriber node_imu
```

## ros_rviz_example

Open it in rviz2.

## stm32_can_example

```shell
make
```