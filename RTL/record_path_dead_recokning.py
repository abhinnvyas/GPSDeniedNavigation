from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
import time

# ========================
# CONFIG
# ========================
GRAVITY = 9.81   # m/s^2
SAVE_FILE = "path.npy"

# ========================
# STATE VARIABLES
# ========================
pos = np.zeros(3)   # [x, y, z]
vel = np.zeros(3)   # [vx, vy, vz]
path = []           # trajectory log
last_t = None

# ========================
# Helper conversion functions
# ========================
def parse_raw_imu(msg):
    # accel in m/s^2
    ax = msg.xacc * 9.81 / 1000.0
    ay = msg.yacc * 9.81 / 1000.0
    az = msg.zacc * 9.81 / 1000.0
    return np.array([ax, ay, az])

def parse_attitude(msg):
    # Roll, Pitch, Yaw in radians
    return msg.roll, msg.pitch, msg.yaw

# ========================
# Connect to Pixhawk
# ========================
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("✅ Connected to system:", master.target_system, master.target_component)

# Request data stream at 50Hz
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    50,   # Hz
    1     # start
)

# Buffers for latest data
latest_acc = None
latest_att = None

print("🚁 Starting INS integration (press CTRL+C to stop)...")

try:
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue

        mtype = msg.get_type()

        if mtype == "RAW_IMU":
            latest_acc = parse_raw_imu(msg)

        elif mtype == "ATTITUDE":
            latest_att = parse_attitude(msg)

        # If we have both IMU and attitude, integrate
        if latest_acc is not None and latest_att is not None:
            now = time.time()
            global last_t, pos, vel

            if last_t is None:
                last_t = now
                continue

            dt = now - last_t
            last_t = now

            roll, pitch, yaw = latest_att

            # Rotation: body → world
            rot = R.from_euler('xyz', [roll, pitch, yaw])
            acc_world = rot.apply(latest_acc)

            # Subtract gravity
            acc_world[2] -= GRAVITY

            # Integrate accel → vel → pos
            vel += acc_world * dt
            pos += vel * dt

            # Save trajectory point
            path.append([pos[0], pos[1], pos[2], yaw])

            # Print debug
            print(f"t={now:.2f}  pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})  yaw={math.degrees(yaw):.1f}°")

except KeyboardInterrupt:
    print("⏹ Stopping... saving trajectory.")
    np.save(SAVE_FILE, np.array(path))
    print(f"💾 Saved {len(path)} waypoints to {SAVE_FILE}")
