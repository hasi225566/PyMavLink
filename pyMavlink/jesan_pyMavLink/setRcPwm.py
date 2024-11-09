from pymavlink import mavutil
import time

# Connect to the Pixhawk via serial
# Adjust the port and baud rate as per your setup
connection = mavutil.mavlink_connection('/dev/ttyACM2', baud=115200)

# Wait for a heartbeat from the Pixhawk
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received")

def set_pwm(channel, pwm):
    """
    Set PWM value on a specific Pixhawk channel.
    
    Parameters:
        channel (int): The output channel number (typically 1-8 for a Pixhawk).
        pwm (int): PWM value in microseconds (typically between 1000 and 2000).
    """
    # MAV_CMD_DO_SET_SERVO command id is 183
    connection.mav.command_long_send(
        connection.target_system,         # Target system
        connection.target_component,      # Target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # Command id
        0,                                # Confirmation
        channel,                          # Servo output channel
        pwm,                              # PWM value in microseconds
        0, 0, 0, 0, 0                     # Unused parameters
    )

# Set up parameters for movement
forward_pwm = 1600  # Adjust as needed
stop_pwm = 1500     # Stop
reverse_pwm = 1400  # Adjust as needed
pwm_channel = 1     # Change to the correct channel number for your setup

try:
    # Move forward
    print("Moving forward")
    set_pwm(pwm_channel, forward_pwm)
    time.sleep(2)

    # Stop
    print("Stopping")
    set_pwm(pwm_channel, stop_pwm)
    time.sleep(1)

    # Move backward
    print("Moving backward")
    set_pwm(pwm_channel, reverse_pwm)
    time.sleep(2)

    # Stop
    print("Stopping")
    set_pwm(pwm_channel, stop_pwm)

finally:
    # Clean up and reset to stop position
    print("Resetting to stop position")
    set_pwm(pwm_channel, stop_pwm)