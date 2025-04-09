#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# 1. 读取CSV
df = pd.read_csv("imu_data.csv")

# 2. 解析时间戳（字符串转为datetime对象）
df["Time"] = pd.to_datetime(df["Time"], format="%Y-%m-%d %H:%M:%S.%f")

# 3. 设置时间为索引（可选）
df.set_index("Time", inplace=True)

# 4. 开始画图
plt.figure(figsize=(12, 8))

# --- 子图1：加速度 ---
plt.subplot(3, 1, 1)
plt.plot(df.index, df["ax"], label="ax")
plt.plot(df.index, df["ay"], label="ay")
plt.plot(df.index, df["az"], label="az")
plt.ylabel("Acceleration (m/s²)")
plt.title("Linear Acceleration")
plt.legend()
plt.grid(True)

# --- 子图2：角速度 ---
plt.subplot(3, 1, 2)
plt.plot(df.index, df["gx"], label="gx")
plt.plot(df.index, df["gy"], label="gy")
plt.plot(df.index, df["gz"], label="gz")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Angular Velocity")
plt.legend()
plt.grid(True)

# --- 子图3：四元数 ---
plt.subplot(3, 1, 3)
plt.plot(df.index, df["q0"], label="q0")
plt.plot(df.index, df["q1"], label="q1")
plt.plot(df.index, df["q2"], label="q2")
plt.plot(df.index, df["q3"], label="q3")
plt.xlabel("Time")
plt.ylabel("Quaternion")
plt.title("Orientation (Quaternion)")
plt.legend()
plt.grid(True)

# 5. 自动排版并显示
plt.tight_layout()
plt.show()
