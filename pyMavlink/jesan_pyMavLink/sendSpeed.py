
from pymavlink import mavutil
import time

# Connect to the Pixhawk via serial
connection = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)

# Wait for a heartbeat from the Pixhawk
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received")

def set_speed(channel, speed_percentage):
    """
    Set the speed on a specific Pixhawk channel by adjusting PWM value.
    
    Parameters:
        channel (int): The output channel number (typically 1-8 for a Pixhawk).
        speed_percentage (float): Speed as a percentage (0 to 100), where 50% is stop.
    """
    # Base PWM values for full reverse, stop, and full forward
    min_pwm = 1400  # Reverse
    neutral_pwm = 1500  # Stop
    max_pwm = 1600  # Forward

    # Calculate the PWM value based on speed percentage
    if speed_percentage < 0:
        # Reverse
        pwm_value = neutral_pwm - int((neutral_pwm - min_pwm) * (-speed_percentage) / 100)
    else:
        # Forward
        pwm_value = neutral_pwm + int((max_pwm - neutral_pwm) * speed_percentage / 100)

    # Send the command to set PWM
    connection.mav.command_long_send(
        connection.target_system,         # Target system
        connection.target_component,      # Target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # Command id
        0,                                # Confirmation
        channel,                          # Servo output channel
        pwm_value,                        # PWM value in microseconds
        0, 0, 0, 0, 0                     # Unused parameters
    )
    print(f"Set speed to {speed_percentage}% on channel {channel} with PWM {pwm_value}")

# Set up parameters for speed control
pwm_channel = 1  # Change to the correct channel for your setup

try:
    # Move forward at 75% speed
    print("Moving forward at 75% speed")
    set_speed(pwm_channel, 75)
    time.sleep(2)

    # Stop
    print("Stopping")
    set_speed(pwm_channel, 0)
    time.sleep(1)

    # Move backward at 50% speed
    print("Moving backward at 50% speed")
    set_speed(pwm_channel, -50)
    time.sleep(2)

    # Stop
    print("Stopping")
    set_speed(pwm_channel, 0)

finally:
    # Clean up and reset to stop position
    print("Resetting to stop position")
    set_speed(pwm_channel, 0)
