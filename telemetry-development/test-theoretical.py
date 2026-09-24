import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("simulated_sbst_telemetry.csv")
dt = 0.1
track_width = 2.08 # physical track width of the bus chassis (T)

x, y, theta = 0.0, 0.0, 0.0
x_pred, y_pred = [], []

for idx, row in df.iterrows():
    # Convert quantized km/h to m/s
    v = row['pid_114_speed_kmh'] / 3.6
    v_fl = row['pid_463_fl_kmh'] / 3.6
    v_fr = row['pid_464_fr_kmh'] / 3.6
    gear = row['pid_131_gear'] # Strict directional multiplier
    
    # Calculate differential yaw
    omega = (v_fr - v_fl) / track_width
    
    # Dead reckoning integration
    theta += omega * dt
    x += v * gear * np.cos(theta) * dt
    y += v * gear * np.sin(theta) * dt
    
    x_pred.append(x)
    y_pred.append(y)

# Plot the deterministic math against the simulation's ground truth
plt.plot(df['target_x'], df['target_y'], label="Ground Truth (Perfect)")
plt.plot(x_pred, y_pred, label="Kinematic Baseline (Degraded)")
plt.legend()
plt.show()