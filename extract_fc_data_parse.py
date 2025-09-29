from pymavlink import mavutil
import math

# ----------------------------
# Helper conversion functions
# ----------------------------
def parse_raw_imu(msg):
    return {
        "accel_g": (
            msg.xacc / 1000.0,  # g
            msg.yacc / 1000.0,
            msg.zacc / 1000.0
        ),
        "gyro_rad_s": (
            msg.xgyro / 1000.0,  # rad/s
            msg.ygyro / 1000.0,
            msg.zgyro / 1000.0
        ),
        "mag_gauss": (
            msg.xmag / 1000.0,   # Gauss
            msg.ymag / 1000.0,
            msg.zmag / 1000.0
        ),
        "temperature_c": msg.temperature / 100.0  # °C
    }

def parse_attitude(msg):
    return {
        "roll_deg": math.degrees(msg.roll),     # degrees
        "pitch_deg": math.degrees(msg.pitch),   # degrees
        "yaw_deg": math.degrees(msg.yaw),       # degrees
        "rollspeed_rad_s": msg.rollspeed,       # rad/s
        "pitchspeed_rad_s": msg.pitchspeed,     # rad/s
        "yawspeed_rad_s": msg.yawspeed          # rad/s
    }

# ----------------------------
# Connect to Pixhawk
# ----------------------------
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Wait for heartbeat
master.wait_heartbeat()
print("Connected to system:", master.target_system, master.target_component)

# Request data stream
master.mav.request_data_stream_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # Hz
    1    # start/stop
)

# ----------------------------
# Main loop
# ----------------------------
while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue

    mtype = msg.get_type()

    if mtype == "RAW_IMU":
        print("-" * 20)
        print("Message type:", mtype)
        print("Fields:", msg.get_fieldnames())
        for field in msg.get_fieldnames():
            print(f"{field} : {getattr(msg, field)}")

        # Parsed version with units
        parsed = parse_raw_imu(msg)
        print("Converted values:")
        print(f"  Accel (g): {parsed['accel_g']}")
        print(f"  Gyro (rad/s): {parsed['gyro_rad_s']}")
        print(f"  Mag (Gauss): {parsed['mag_gauss']}")
        print(f"  Temperature (°C): {parsed['temperature_c']}")

    elif mtype == "ATTITUDE":
        print("-" * 20)
        print("Message type:", mtype)
        print("Fields:", msg.get_fieldnames())
        for field in msg.get_fieldnames():
            print(f"{field} : {getattr(msg, field)}")

        # Parsed version with units
        parsed = parse_attitude(msg)
        print("Converted values:")
        print(f"  Roll (deg): {parsed['roll_deg']:.2f}")
        print(f"  Pitch (deg): {parsed['pitch_deg']:.2f}")
        print(f"  Yaw (deg): {parsed['yaw_deg']:.2f}")
        print(f"  Rollspeed (rad/s): {parsed['rollspeed_rad_s']:.4f}")
        print(f"  Pitchspeed (rad/s): {parsed['pitchspeed_rad_s']:.4f}")
        print(f"  Yawspeed (rad/s): {parsed['yawspeed_rad_s']:.4f}")
