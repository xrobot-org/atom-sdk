#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define IMU_ID (0x30)
#define UART_PORT ("/dev/ttyACM0")

/* Structure for 3D vector (e.g., acceleration, gyroscope) */
typedef struct __attribute__((packed))
{
  float x;
  float y;
  float z;
} Vector3;

/* Structure for quaternion representation of rotation */
typedef struct __attribute__((packed))
{
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

/* Structure for Euler angles representation of rotation */
typedef struct __attribute__((packed))
{
  float rol;
  float pit;
  float yaw;
} EulerAngles;

/* Full data structure received from IMU sensor */
typedef struct __attribute__((packed))
{
  uint8_t prefix; /* Packet header (0xA5) */
  uint64_t time : 40;
  uint64_t sync : 40;
  Quaternion quat_;
  Vector3 gyro_;
  Vector3 accl_;
  EulerAngles eulr_;
  uint8_t crc8; /* CRC-8 checksum */
} Data;

/* CAN Data Structure */
typedef struct __attribute__((packed))
{
  uint8_t prefix;
  uint32_t id;
  uint8_t data[8];
  uint8_t crc8;
} DataCanToUart;

#define DATA_IMU_LENGTH sizeof(Data)
#define DATA_CAN_LENGTH sizeof(DataCanToUart)

static const uint8_t CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
    0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
    0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
    0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
    0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
    0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
    0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
    0xd7, 0x89, 0x6b, 0x35};

uint8_t CalculateCRC8(const uint8_t *buf, size_t len, uint8_t crc)
{
  while (len-- > 0)
  {
    crc = CRC8_TAB[(crc ^ *buf++) & 0xff];
  }
  return crc;
}

bool VerifyData(const uint8_t *buf, size_t len)
{
  if (len < 2)
  {
    return false;
  }

  uint8_t expected = CalculateCRC8(buf, len - sizeof(uint8_t), 0xff);
  return expected == buf[len - sizeof(uint8_t)];
}

/* Function to return a standard baud rate value */
speed_t get_standard_baudrate(int baudrate)
{
  switch (baudrate)
  {
  case 9600:
    return B9600;
  case 115200:
    return B115200;
  case 460800:
    return B460800;
  case 1000000:
    return B1000000;
  case 2000000:
    return B2000000;
  default:
    return 0; /* Invalid baud rate */
  }
}

/* Function to open and configure serial port with the specified baud rate */
int open_serial_port(const char *port, int baudrate)
{
  int fd = open(port, O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    perror("open_port: Unable to open");
    return -1;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd, &tty) != 0)
  {
    perror("tcgetattr");
    close(fd);
    return -1;
  }

  /* Set baud rate */
  speed_t baud = get_standard_baudrate(baudrate);
  if (!baud)
  {
    fprintf(stderr, "Unsupported baud rate: %d\n", baudrate);
    close(fd);
    return -1;
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  /* Configure other serial port settings */
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~(OPOST | ONLCR);

  tty.c_cc[VTIME] = 10; /* 1-second timeout */
  tty.c_cc[VMIN] = 64;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    perror("tcsetattr");
    close(fd);
    return -1;
  }
  return fd;
}

/* Main function */
int main(int argc, char *argv[])
{
  int baudrate = 2000000; /* Default baud rate */
  if (argc > 1)
  {
    baudrate = atoi(argv[1]); /* Allow user to specify baud rate */
  }

  const char *serial_port = UART_PORT;
  int serial_fd = open_serial_port(serial_port, baudrate);
  if (serial_fd < 0)
  {
    return EXIT_FAILURE;
  }

  printf("Opened %s at %d baud.\n", serial_port, baudrate);

  Data received_data;
  ssize_t bytes_read;

  while (1)
  {
    uint8_t prefix;
    ssize_t ret = read(serial_fd, &prefix, 1);
    if (ret < 0)
    {
      perror("Read failed");
      break;
    }
    else if (ret == 0)
    {
      printf("Read returned 0 (EOF or no data)\n");
      break;
    }

    if (prefix == 0xA5)
    {
      Data imu_data;
      imu_data.prefix = prefix;
      if (read(serial_fd, ((uint8_t *)&imu_data) + 1, DATA_IMU_LENGTH - 1) !=
          DATA_IMU_LENGTH - 1)
      {
        perror("Read failed");
        continue;
      }

      if (VerifyData((uint8_t *)&imu_data, DATA_IMU_LENGTH))
      {
        memcpy(&received_data, &imu_data, sizeof(Data));
        printf("Time:%ld Sync:%ld Roll: %+6f, Pitch:%+6f, Yaw:%+6f, "
               "Ax:%+6f, Ay:%+6f, Az:%+6f, Gx:%+6f, Gy:%+6f, Gz:%+6f, Q0:%+6f, Q1:%+6f, Q2:%+6f, Q3:%+6f\n",
               received_data.time, received_data.sync, received_data.eulr_.rol,
               received_data.eulr_.pit, received_data.eulr_.yaw,
               received_data.accl_.x, received_data.accl_.y,
               received_data.accl_.z, received_data.gyro_.x,
               received_data.gyro_.y, received_data.gyro_.z,
               received_data.quat_.q0, received_data.quat_.q1,
               received_data.quat_.q2, received_data.quat_.q3);
      }
      else
      {
        printf("IMU CRC check failed.\n");
      }
    }
#if 0
    else if (prefix == 0xA6)
    {
      DataCanToUart can_data;
      can_data.prefix = prefix;
      if (read(serial_fd, ((uint8_t *)&can_data) + 1, DATA_CAN_LENGTH - 1) !=
          DATA_CAN_LENGTH - 1)
      {
        perror("Read failed");
        continue;
      }

      if (VerifyData((uint8_t *)&can_data, DATA_CAN_LENGTH))
      {
        printf("CAN: ID:%d, Data: ", can_data.id);
        for (int i = 0; i < 8; i++)
          printf("%02X ", can_data.data[i]);
        printf("\n");
        write(serial_fd, ((uint8_t *)&can_data), DATA_CAN_LENGTH);
      }
      else
      {
        printf("CAN CRC check failed.\n");
      }
    }
#endif
  }
  close(serial_fd);
  return EXIT_SUCCESS;
}
