import math
import numpy as np
from pymavlink import mavutil

# =====================
# EKF + INS Setup
# =====================
dt = 0.1  # update rate (s)

# State vector: [px, py, pz, vx, vy, vz, bax, bay, baz]
x = np.zeros(9)
P = np.eye(9) * 0.1

Q = np.eye(9) * 0.01   # process noise
R_baro = np.array([[0.5]])   # barometer noise
R_gps  = np.eye(3) * 2.0     # GPS noise

gps_ref = None  # will hold reference lat/lon/alt

def predict(u, dt):
    global x, P
    ax, ay, az = u - x[6:9]  # remove bias
    F = np.eye(9)
    F[0,3] = F[1,4] = F[2,5] = dt

    B = np.zeros((9,3))
    B[3:6, :] = np.eye(3) * dt

    x[0:3] += x[3:6] * dt + 0.5 * u * dt*dt
    x[3:6] += u * dt

    P = F @ P @ F.T + Q

def gps_update(z):
    global x, P
    H = np.zeros((3,9))
    H[0,0] = H[1,1] = H[2,2] = 1
    y = z - x[0:3]
    S = H @ P @ H.T + R_gps
    K = P @ H.T @ np.linalg.inv(S)
    x[:] = x + K @ y
    P[:] = (np.eye(9) - K @ H) @ P

# =====================
# MAVLink Connection
# =====================
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print(f"Connected to system {master.target_system} component {master.target_component}")

# Request all data streams
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # Hz
    1
)

# =====================
# Main Loop
# =====================
while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue

    mtype = msg.get_type()

    # --------- IMU Update (Prediction) ---------
    if mtype == "RAW_IMU":
        ax = msg.xacc / 1000.0  # milli-g to g
        ay = msg.yacc / 1000.0
        az = msg.zacc / 1000.0
        imu = np.array([ax, ay, az])
        predict(imu, dt)

    # --------- GPS Update (Correction + Comparison) ---------
    elif mtype == "GPS_RAW_INT":
        gps_lat = msg.lat / 1e7
        gps_lon = msg.lon / 1e7
        gps_alt = msg.alt / 1000.0

        # Store reference for local frame
        if gps_ref is None:
            gps_ref = (gps_lat, gps_lon, gps_alt)

        # Convert to meters relative to gps_ref
        dlat = (gps_lat - gps_ref[0]) * 111320
        dlon = (gps_lon - gps_ref[1]) * 111320 * math.cos(math.radians(gps_ref[0]))
        dz   = gps_alt - gps_ref[2]

        gps_pos = np.array([dlon, dlat, dz])
        gps_update(gps_pos)

        # Compare with INS-estimated pos
        ins_pos = x[0:3]
        error = np.linalg.norm(ins_pos - gps_pos)

        match = "✅ MATCH" if error < 1.0 else "❌ DRIFT"  # tolerance = 5m

        print(f"GPS: {gps_pos}, INS: {ins_pos}, Error={error:.2f}m {match}")
