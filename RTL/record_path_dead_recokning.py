import time
import numpy as np
from pymavlink import mavutil

# =====================
# Global Variables
# =====================
last_t = None
pos = np.array([0.0, 0.0, 0.0])  # x, y, z position in meters
vel = np.array([0.0, 0.0, 0.0])  # velocity in m/s
path = []  # list to store waypoints

# =====================
# MAVLink Connection
# =====================
print("Connecting to drone...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("Connected to system:", master.target_system, " component:", master.target_component)

# =====================
# Functions
# =====================
def update_position(msg):
    """
    Update position using GPS (when available).
    """
    global pos, path
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000.0
    pos = np.array([lat, lon, alt])
    path.append(pos.copy())


def dead_reckoning(msg):
    """
    Update position using IMU dead reckoning when GPS is lost.
    """
    global last_t, pos, vel, path

    # Ensure globals are declared at top
    global last_t, pos, vel

    # Extract IMU data
    ax = msg.xacc / 1000.0  # acceleration (m/s^2)
    ay = msg.yacc / 1000.0
    az = msg.zacc / 1000.0

    t = time.time()
    if last_t is None:
        last_t = t
        return

    dt = t - last_t
    last_t = t

    # Integrate acceleration → velocity
    vel[0] += ax * dt
    vel[1] += ay * dt
    vel[2] += az * dt

    # Integrate velocity → position
    pos[0] += vel[0] * dt
    pos[1] += vel[1] * dt
    pos[2] += vel[2] * dt

    path.append(pos.copy())


def save_path():
    np.save("path.npy", np.array(path))
    print("Saved", len(path), "waypoints to path.npy")


# =====================
# Main Loop
# =====================
try:
    while True:
        msg = master.recv_match(blocking=True)

        if not msg:
            continue

        msg_type = msg.get_type()

        if msg_type == "GLOBAL_POSITION_INT":  # GPS available
            update_position(msg)

        elif msg_type == "RAW_IMU":  # Use IMU for dead reckoning
            dead_reckoning(msg)

except KeyboardInterrupt:
    print("Stopping recording...")
    save_path()
