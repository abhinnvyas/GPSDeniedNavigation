from pymavlink import mavutil

# ----------------------------
# Connect to Cube Orange
# ----------------------------
print("Connecting to Cube Orange...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)  # adjust if needed
master.wait_heartbeat()
print("Connected to system:", master.target_system, " component:", master.target_component)

# ----------------------------
# Request LOCAL_POSITION_NED stream
# ----------------------------
# MAV_DATA_STREAM_POSITION = 6
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    10,  # Hz
    1    # start
)

print("Requested LOCAL_POSITION_NED stream at 10Hz...")

waypoints = []

# ----------------------------
# Main loop
# ----------------------------
try:
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue

        if msg.get_type() == "LOCAL_POSITION_NED":
            x = msg.x  # meters, NED frame
            y = msg.y
            z = msg.z  # Down is positive, so altitude is -z
            vx = msg.vx
            vy = msg.vy
            vz = msg.vz

            waypoints.append((x, y, z))

            print(f"[Waypoint {len(waypoints)}] X={x:.2f} m, Y={y:.2f} m, Z={z:.2f} m "
                  f"| Vel=({vx:.2f},{vy:.2f},{vz:.2f})")

except KeyboardInterrupt:
    print("Stopping logging... Saved", len(waypoints), "waypoints.")
