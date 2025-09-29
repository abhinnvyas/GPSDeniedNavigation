#!/usr/bin/env python3
# replay_path.py
# Load path.npy and replay it backwards with SET_POSITION_TARGET_LOCAL_NED
# WARNING: Test in safe environment. This will switch mode to GUIDED and send setpoints.

import numpy as np
import time
import math
from pymavlink import mavutil
from scipy.interpolate import CubicSpline

TELEM = '/dev/ttyACM0'
BAUD = 57600
PATHFILE = 'path.npy'

# tuning
WAYPOINT_SPACING_M = 0.5     # desired spacing (meters) when resampling path
POS_TOLERANCE = 1.0         # meters tolerance per waypoint
MAX_TIME_PER_WP = 10.0     # seconds
SETPOINT_RATE = 2.0        # Hz to send setpoints
REPLAY_SPEED = 1.0         # multiplier for timing (1.0 = same speed as recording)

master = mavutil.mavlink_connection(TELEM, baud=BAUD)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected", master.target_system, master.target_component)

# load path
arr = np.load(PATHFILE)   # shape: (N,5) columns: t_rel, x(N), y(E), z(D), yaw
t = arr[:,0]
x = arr[:,1]
y = arr[:,2]
z = arr[:,3]
yaw = arr[:,4]

# preprocess: convert NED z (down) to altitude (up) if desired
# We'll keep NED coordinates for sending; ensure we send same frame.

# resample path by arc length to get roughly uniform spacing
pts = np.stack([x, y, z], axis=1)
# compute cumulative distances
d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
arc = np.concatenate([[0.0], np.cumsum(d)])
total_len = arc[-1]
if total_len < 1e-6:
    raise SystemExit("Path is too short")

n_pts = max(2, int(total_len / WAYPOINT_SPACING_M))
new_s = np.linspace(0, total_len, n_pts)

# cubic spline interpolation on each axis versus arc-length
cs_x = CubicSpline(arc, x)
cs_y = CubicSpline(arc, y)
cs_z = CubicSpline(arc, z)
cs_yaw = CubicSpline(arc, yaw)

res_x = cs_x(new_s)
res_y = cs_y(new_s)
res_z = cs_z(new_s)
res_yaw = cs_yaw(new_s)

# reverse for RTL
res_x = res_x[::-1]
res_y = res_y[::-1]
res_z = res_z[::-1]
res_yaw = res_yaw[::-1]

print(f"Prepared {len(res_x)} replay waypoints (reversed).")

# helper: send position target local ned
def send_local_position_ned(x_n, y_e, z_down, yaw_rad, vx=0, vy=0, vz=0, type_mask=0b0000111111111000):
    # type_mask: bitmask — position enabled, velocity disabled, acceleration disabled, yaw enabled
    # using MAVLink helper: set_position_target_local_ned_send(time_boot_ms, target_system, target_component, coordinate_frame, type_mask,
    # px, py, pz, vx, vy, vz, afx, afy, afz, yaw, yaw_rate)
    master.mav.set_position_target_local_ned_send(
        int(time.time()*1000), master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        float(x_n), float(y_e), float(z_down),
        float(vx), float(vy), float(vz),
        0,0,0,   # afx afy afz (accel)
        float(yaw_rad), 0.0
    )

# switch to GUIDED (or GUIDED_NOGPS if available). This depends on your firmware.
# Some firmwares have GUIDED and accept local setpoints even with GPS off.
# You might need to set mode to 'GUIDED' via MAV_CMD_DO_SET_MODE or mav.set_mode_send
mode = 'GUIDED'
mode_id = None
# try to find mode id
if mode in master.mode_mapping():
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Mode set to GUIDED")
else:
    print("GUIDED not in mode mapping; ensure your FC supports guided/offboard local setpoints")

# small wait
time.sleep(1.0)

# Replay loop
for i in range(len(res_x)):
    wp_x = res_x[i]
    wp_y = res_y[i]
    wp_z = res_z[i]
    wp_yaw = res_yaw[i]

    print(f"Sending WP {i+1}/{len(res_x)} -> N={wp_x:.2f}, E={wp_y:.2f}, D={wp_z:.2f}")
    send_local_position_ned(wp_x, wp_y, wp_z, wp_yaw)

    # wait until within tolerance or timeout
    start = time.time()
    reached = False
    while time.time() - start < MAX_TIME_PER_WP:
        # You can read LOCAL_POSITION_NED from FC to check current pos
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg is None:
            continue
        cur_x = float(msg.x)
        cur_y = float(msg.y)
        cur_z = float(msg.z)
        err = math.sqrt((cur_x - wp_x)**2 + (cur_y - wp_y)**2 + (cur_z - wp_z)**2)
        # print progress
        print(f"  cur pos ({cur_x:.2f},{cur_y:.2f},{cur_z:.2f}) err={err:.2f}m")
        if err <= POS_TOLERANCE:
            reached = True
            break
        # keep resending setpoint at a lower rate
        send_local_position_ned(wp_x, wp_y, wp_z, wp_yaw)
        time.sleep(1.0/SETPOINT_RATE)

    if not reached:
        print(f"Waypoint {i+1} not reached within timeout. ABORTING to LAND as safety.")
        master.set_mode(master.mode_mapping()['LAND'])
        raise SystemExit("Aborting replay due to waypoint timeout")

# On completion: final slow descent to land
print("Completed path replay. Performing gentle land sequence.")
# Final: set final position to current XY and gradually decrease altitude
# get current local pos
msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
if msg:
    cx, cy, cz = float(msg.x), float(msg.y), float(msg.z)
    # descend by 0.5 m steps
    for dz in np.linspace(cz, cz+2.0, 8):  # cz is down, so increasing cz goes downwards
        send_local_position_ned(cx, cy, dz, 0.0)
        time.sleep(1.0)
# Switch to LAND for final safety
master.set_mode(master.mode_mapping()['LAND'])
print("Set mode LAND. Replay complete.")
