import numpy as np
import pandas as pd

def simulate_depot_run(duration_sec=40.0, dt=0.1, track_width=2.08):
    """
    Simulates a depot maneuver and generates synthetic Stratio telemetry.
    
    Sensor profiles:
    - ID 114 (Speed): 1/256 km/h resolution
    - ID 463 (FL Wheel Speed): 1/256 km/h resolution
    - ID 464 (FR Wheel Speed): 1/256 km/h resolution
    - ID 267 (High-Res Distance): 5/125 m (0.04 m) resolution
    - ID 131 (Gear Mode): 1 = Forward (D), -1 = Reverse (R), 0 = Neutral (N)
    """
    steps = int(duration_sec / dt)
    time_arr = np.linspace(0, duration_sec, steps)
    
    # Ground truth state arrays
    x_true = np.zeros(steps)
    y_true = np.zeros(steps)
    theta_true = np.zeros(steps)
    dist_accum_true = np.zeros(steps)
    
    # Control input trajectories
    v_cmd = np.zeros(steps)       # Forward linear speed (m/s)
    omega_cmd = np.zeros(steps)   # Yaw rate (rad/s)
    gear_cmd = np.ones(steps)     # 1: Drive, -1: Reverse
    
    # Define a realistic depot path:
    # 0-10s: Drive straight, accelerate to ~15 km/h (4.16 m/s)
    # 10-18s: Slow down to 8 km/h and execute a 90-degree right turn
    # 18-25s: Drive straight to align with parking berth
    # 25-28s: Stop, shift gear to Reverse (Gear 131 = -1)
    # 28-40s: Reverse slowly into berth (-1.5 m/s)
    for i, t in enumerate(time_arr):
        if t < 10.0:
            v_cmd[i] = min(4.16, 0.5 * t)
            omega_cmd[i] = 0.0
            gear_cmd[i] = 1
        elif 10.0 <= t < 18.0:
            v_cmd[i] = 2.2
            omega_cmd[i] = - (np.pi / 2.0) / 8.0  # Turn 90 deg right over 8 seconds
            gear_cmd[i] = 1
        elif 18.0 <= t < 25.0:
            v_cmd[i] = 2.0
            omega_cmd[i] = 0.0
            gear_cmd[i] = 1
        elif 25.0 <= t < 28.0:
            v_cmd[i] = 0.0
            omega_cmd[i] = 0.0
            gear_cmd[i] = 0  # Neutral / stopping
        else:
            v_cmd[i] = -1.5  # Reversing
            omega_cmd[i] = 0.0
            gear_cmd[i] = -1  # Reverse
            
    # Integrate nominal physics to obtain True Trajectory
    for i in range(1, steps):
        theta_true[i] = theta_true[i-1] + omega_cmd[i-1] * dt
        x_true[i] = x_true[i-1] + v_cmd[i-1] * np.cos(theta_true[i]) * dt
        y_true[i] = y_true[i-1] + v_cmd[i-1] * np.sin(theta_true[i]) * dt
        dist_accum_true[i] = dist_accum_true[i-1] + abs(v_cmd[i-1]) * dt

    # -------------------------------------------------------------
    # Synthesize Raw Sensors & Apply Degradations / Quantization
    # -------------------------------------------------------------
    # 1. Differential wheel speeds (m/s)
    v_fl_true = v_cmd - (omega_cmd * track_width / 2.0)
    v_fr_true = v_cmd + (omega_cmd * track_width / 2.0)
    
    # 2. Inject realistic errors (tire wear scale bias = 0.98, Gaussian noise, occasional turn slip)
    tire_scale_factor = 0.98
    noise_fl = np.random.normal(0, 0.05, steps)
    noise_fr = np.random.normal(0, 0.05, steps)
    
    v_fl_sensed = (v_fl_true * tire_scale_factor) + noise_fl
    v_fr_sensed = (v_fr_true * tire_scale_factor) + noise_fr
    v_sensed = (abs(v_cmd) * tire_scale_factor) + np.random.normal(0, 0.04, steps)
    
    # 3. Discretize into Stratio native resolution
    # Speed resolution: 1/256 km/h. Distance resolution: 5/125 m (0.04 m)
    kmh_to_ms = 1.0 / 3.6
    ms_to_kmh = 3.6
    speed_lsb = 1.0 / 256.0      # km/h per bit
    dist_lsb = 5.0 / 125.0       # 0.04 m per bit
    
    pid_114_speed = np.round((v_sensed * ms_to_kmh) / speed_lsb) * speed_lsb
    pid_460_tacho = np.round((v_sensed * ms_to_kmh) / speed_lsb) * speed_lsb
    pid_463_fl = np.round((abs(v_fl_sensed) * ms_to_kmh) / speed_lsb) * speed_lsb
    pid_464_fr = np.round((abs(v_fr_sensed) * ms_to_kmh) / speed_lsb) * speed_lsb
    pid_267_dist = np.round(dist_accum_true / dist_lsb) * dist_lsb
    pid_131_gear = gear_cmd
    
    telemetry_df = pd.DataFrame({
        "timestamp_sec": time_arr,
        "pid_114_speed_kmh": pid_114_speed,
        "pid_460_tacho_kmh": pid_460_tacho,
        "pid_463_fl_kmh": pid_463_fl,
        "pid_464_fr_kmh": pid_464_fr,
        "pid_267_dist_m": pid_267_dist,
        "pid_131_gear": pid_131_gear,
        # Ground Truth Targets for Training / Validation
        "target_x": x_true,
        "target_y": y_true,
        "target_theta": theta_true
    })
    
    return telemetry_df

if __name__ == "__main__":
    df = simulate_depot_run()
    print("Synthesized Telemetry Sample:")
    print(df[["timestamp_sec", "pid_114_speed_kmh", "pid_463_fl_kmh", "pid_464_fr_kmh", "pid_267_dist_m", "pid_131_gear"]].head(10))
    df.to_csv("simulated_sbst_telemetry.csv", index=False)