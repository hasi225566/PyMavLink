from pymavlink import mavutil
import time

# Establish connection to the drone
connection_string = '/dev/ttyACM0'  # Update this as needed
master = mavutil.mavlink_connection(connection_string, baud=57600)

# Wait for the heartbeat from the drone
master.wait_heartbeat()
print("Heartbeat received")

# Set the mode to GUIDED
mode = 'GUIDED'
master.set_mode_apm(mode)
print(f"Mode set to {mode}")

# Arm the drone
master.arducopter_arm()
print("Arming the drone...")

# Wait until the drone is armed
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg is not None and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("Drone is armed")
        break
    time.sleep(1)

print("Ready for takeoff!")

# Perform any desired operations here
# For example, you might add takeoff or movement commands

# Disarm the drone
master.arducopter_disarm()
print("Disarming the drone...")

# Wait until the drone is disarmed
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg is not None and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("Drone is disarmed")
        break
    time.sleep(1)

print("Drone is safely disarmed.")
