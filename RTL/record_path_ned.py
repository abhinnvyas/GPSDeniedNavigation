#!/usr/bin/env python3
# record_path.py
# Run while drone is flying in GPS mode. Saves path to path.npy

import numpy as np
import time
from pymavlink import mavutil

TELEM = '/dev/ttyACM0'
BAUD = 57600
OUTFILE = 'path.npy'
SAMPLE_HZ = 5     # sample rate in Hz

master = mavutil.mavlink_connection(TELEM, baud=BAUD)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected", master.target_system, master.target_component)

# Request local position stream if available
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_POSITION, SAMPLE_HZ, 1)

path = []  # list of [t_rel, x_n, y_e, z_d, yaw_rad]

start_t = time.time()
print("Recording path. Fly your mission. Ctrl-C to stop and save.")

try:
    while True:
        msg = master.recv_match(timeout=1)
        if msg is None:
            continue

        mtype = msg.get_type()

        # prefer LOCAL_POSITION_NED (gives x,y,z in meters relative to origin)
        if mtype == 'LOCAL_POSITION_NED':

            print("NED:", msg.msg.get_fieldnames())
            t_rel = time.time() - start_t
            x = float(msg.x)   # north (m)
            y = float(msg.y)   # east (m)
            z = float(msg.z)   # down (m)
            # get yaw from ATTITUDE message: store latest
            # We'll track latest attitude separately
            # fallback yaw=None for now (we fill it later if we have it)
            yaw = None
            path.append([t_rel, x, y, z, yaw])

        elif mtype == 'ATTITUDE':
            # if path has entries and the last entry has no yaw, set it
            if path:
                yaw = float(msg.yaw)  # radians
                # assign yaw to the most recent path entry that lacks yaw
                if path[-1][4] is None:
                    path[-1][4] = yaw

        # also, as fallback if LOCAL_POSITION_NED not published,
        # you can build local NED from GPS_RAW_INT relative to first fix (not shown)
except KeyboardInterrupt:
    print("Stopping recording...")

# Postprocess: fill missing yaws by forward-filling
for i in range(len(path)):
    if path[i][4] is None:
        path[i][4] = path[i-1][4] if i>0 else 0.0

arr = np.array(path)
np.save(OUTFILE, arr)
print(f"Saved {len(arr)} waypoints to {OUTFILE}")
