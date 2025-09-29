from pymavlink import mavutil

# Connect to Pixhawk via UART or USB
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Wait for heartbeat
master.wait_heartbeat()
print("Connected to system:", master.target_system, master.target_component)

# Request data stream
master.mav.request_data_stream_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # frequency (Hz)
    1    # start/stop
)

while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue

    mtype = msg.get_type()

    if mtype == "LOCAL_POSITION_NED":
        print("-"*20)
        if msg:
            print("Message type:", msg.get_type())     # e.g. RAW_IMU, ATTITUDE
            print("Fields:", msg.get_fieldnames())     # list of field names
            for field in msg.get_fieldnames():
                print(field, ":", getattr(msg, field))
        
    # elif mtype == "ATTITUDE":
    #     print("-"*20)
    #     if msg:
    #         print("Message type:", msg.get_type())     # e.g. RAW_IMU, ATTITUDE
    #         print("Fields:", msg.get_fieldnames())     # list of field names
    #         for field in msg.get_fieldnames():
    #             print(field, ":", getattr(msg, field))
