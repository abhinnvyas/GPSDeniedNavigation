import numpy as np
from ahrs.filters import Madgwick

madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])

gyro_data = np.array([0.0, np.deg2rad(5.0), 0.0])  # 5°/s
accel_data = np.array([0.0, 0.0, 9.81])

# Normalize accel
accel_data = accel_data / np.linalg.norm(accel_data)

for i in range(10):
    q = madgwick.updateIMU(q, gyr=gyro_data, acc=accel_data)
    print(f"Step {i} Quaternion: {q}")
