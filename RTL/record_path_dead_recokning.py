import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

# ========================
# CONFIG
# ========================
GRAVITY = 9.81  # m/s^2

# ------------------------
# Load your data
# Assume CSV or JSON converted to DataFrame with columns:
# time_usec, xacc, yacc, zacc, roll, pitch, yaw
# ------------------------
df = pd.read_csv("imu_attitude_log.csv")

# Convert units
df["time"] = df["time_usec"] * 1e-6              # µs → seconds
df["ax"] = df["xacc"] * 9.81 / 1000.0            # mg → m/s^2
df["ay"] = df["yacc"] * 9.81 / 1000.0
df["az"] = df["zacc"] * 9.81 / 1000.0

# ------------------------
# Initialize states
# ------------------------
pos = np.zeros(3)   # x,y,z
vel = np.zeros(3)   # vx,vy,vz
path = []           # store trajectory

last_t = df["time"].iloc[0]

for i, row in df.iterrows():
    # Time delta
    t = row["time"]
    dt = t - last_t
    if dt <= 0:
        continue
    last_t = t

    # Body-frame acceleration
    acc_body = np.array([row["ax"], row["ay"], row["az"]])

    # Rotation from attitude (roll,pitch,yaw)
    rot = R.from_euler('xyz', [row["roll"], row["pitch"], row["yaw"]])
    acc_world = rot.apply(acc_body)

    # Remove gravity
    acc_world[2] -= GRAVITY

    # Integrate acceleration → velocity
    vel += acc_world * dt

    # Integrate velocity → position
    pos += vel * dt

    # Store (x, y, z, yaw)
    path.append([pos[0], pos[1], pos[2], row["yaw"]])

# Convert to array
path = np.array(path)

# Save path
np.save("path.npy", path)
print(f"Saved {len(path)} waypoints to path.npy")
