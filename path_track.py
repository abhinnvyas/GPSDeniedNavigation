from pymavlink import mavutil
import numpy as np
import math
import csv
from scipy.spatial.transform import Rotation as R
import time

TELEM = '/dev/ttyACM0'
BAUD = 57600
G = 9.80665

# -------------------------
# Params
# -------------------------
CALIB_SECONDS = 2.0      # collect bias for first N seconds while drone is stationary
LOG_CSV = 'path_log.csv'
MIN_DT = 1e-4
MAX_DT = 0.1             # ignore unrealistic big gaps

# -------------------------
# Helpers
# -------------------------
def rawimu_to_mps2(xacc_mg, yacc_mg, zacc_mg):
    # RAW_IMU units: mg (1 -> 0.001 g)
    return np.array([xacc_mg, yacc_mg, zacc_mg], dtype=float) / 1000.0 * G

def rot_from_att(att_msg):
    # ATTITUDE message gives roll,pitch,yaw in radians
    # Use 'xyz' (roll, pitch, yaw) to produce body->world rotation
    return R.from_euler('xyz', [att_msg.roll, att_msg.pitch, att_msg.yaw], degrees=False)

# -------------------------
# MAVLink connection
# -------------------------
master = mavutil.mavlink_connection(TELEM, baud=BAUD)
print("waiting for heartbeat...")
master.wait_heartbeat()
print("connected", master.target_system, master.target_component)
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)

# -------------------------
# State variables
# -------------------------
pos = np.zeros(3)
vel = np.zeros(3)

last_time = None
last_acc_world = None   # for trapezoidal integration

# keep latest attitude
latest_att = None

# bias estimation
calib_samples = []
calib_end_time = None
bias_acc = np.zeros(3)
calibrated = False
start_wall = time.time()

# csv log
csvf = open(LOG_CSV, 'w', newline='')
cw = csv.writer(csvf)
cw.writerow(['t_usec', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'ax_w', 'ay_w', 'az_w'])

print("Starting loop... (perform initial stationary calibration)")

try:
    while True:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue

        mtype = msg.get_type()

        # update latest attitude as it arrives
        if mtype == 'ATTITUDE':
            latest_att = msg
            continue

        # Use RAW_IMU (time_usec is available)
        if mtype != 'RAW_IMU':
            continue

        # timestamp from IMU
        if hasattr(msg, 'time_usec') and msg.time_usec is not None:
            t_usec = int(msg.time_usec)
            t = t_usec / 1e6
        elif hasattr(msg, 'time_boot_ms'):
            t = msg.time_boot_ms / 1000.0
            t_usec = int(t * 1e6)
        else:
            # fallback
            t = time.time()
            t_usec = int(t * 1e6)

        # if still calibrating (first CALIB_SECONDS wall seconds)
        if (time.time() - start_wall) < CALIB_SECONDS:
            # collect raw accel in m/s^2 (body frame)
            acc_body = rawimu_to_mps2(msg.xacc, msg.yacc, msg.zacc)
            calib_samples.append(acc_body)
            if (time.time() - start_wall) >= CALIB_SECONDS:
                bias_acc = np.mean(calib_samples, axis=0)
                calibrated = True
                print("Calibration done. accel bias (m/s^2):", bias_acc)
            continue

        if not calibrated:
            continue

        # require attitude to transform into earth frame
        if latest_att is None:
            # wait until first attitude arrives
            continue

        # compute dt using IMU timestamps
        if last_time is None:
            last_time = t
            # compute acc_world for first time then continue (no integration)
            acc_body = rawimu_to_mps2(msg.xacc, msg.yacc, msg.zacc) - bias_acc
            Rwb = rot_from_att(latest_att)
            acc_world = Rwb.apply(acc_body)
            last_acc_world = acc_world
            continue

        dt = t - last_time
        if dt < MIN_DT or dt > MAX_DT:
            # skip invalid dt, reset last_time but keep last_acc_world
            last_time = t
            continue

        # convert raw accel -> m/s^2 and remove bias
        acc_body = rawimu_to_mps2(msg.xacc, msg.yacc, msg.zacc) - bias_acc

        # rotate to earth frame using latest attitude
        Rwb = rot_from_att(latest_att)
        acc_world = Rwb.apply(acc_body)

        # gravity vector in earth frame (Rwb.apply([0,0,1]) gives direction of body z in earth)
        gravity_vec = Rwb.apply([0.0, 0.0, 1.0]) * G

        # linear acceleration (remove gravity)
        lin_acc = acc_world - gravity_vec

        # trapezoidal integration for velocity and position
        # v_new = v + 0.5*(a_last + a_now)*dt
        vel += 0.5 * (last_acc_world + lin_acc) * dt
        pos += vel * dt + 0.5 * lin_acc * dt * dt  # semi-implicit

        # update trackers
        last_time = t
        last_acc_world = lin_acc

        # Log & print (t_usec is IMU timestamp)
        cw.writerow([t_usec, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2],
                     lin_acc[0], lin_acc[1], lin_acc[2]])
        csvf.flush()

        # quick debug print (every Nth sample or only if big change)
        print(f"t={t:.3f} pos={pos} vel={vel} lin_acc={lin_acc}")

except KeyboardInterrupt:
    print("stopping...")

finally:
    csvf.close()
    print("log saved to", LOG_CSV)
