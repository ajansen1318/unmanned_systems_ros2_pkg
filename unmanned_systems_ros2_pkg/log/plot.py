#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd


df = pd.read_csv("Oct_06_17_13.csv")

time_data = df.iloc[:, 0].values
x_pos = df.iloc[:, 1].values
y_pos = df.iloc[:, 2].values
yaw = df.iloc[:, 3].values
lin_vel = df.iloc[:, 4].values
ang_vel = df.iloc[:, 5].values


plt.plot(time_data, ang_vel)
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Angular Velocity Command")
plt.grid(True)
plt.show()
