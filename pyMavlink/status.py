from pymavlink import mavutil
import time

def connect_to_pixhawk(connection_string='/dev/ttyACM0', baudrate=57600):
    """
    Establish a connection to the Pixhawk flight controller.

    Parameters:
        connection_string (str): The connection string or port.
        baudrate (int): The baud rate for the connection.

    Returns:
        mavutil.mavlink_connection: The MAVLink connection object.
    """
    print(f"Connecting to Pixhawk on {connection_string} at {baudrate} baud.")
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    master.wait_heartbeat()
    print("Heartbeat received from Pixhawk.")
    return master

def display_data(master):
    """
    Receive and display data from the Pixhawk.

    Parameters:
        master (mavutil.mavlink_connection): The MAVLink connection object.
    """
    print("Receiving data...")
    try:
        while True:
            # Receive the next MAVLink message
            msg = master.recv_match(blocking=True)
            if not msg:
                continue

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                flight_mode = mavutil.mode_string_v10(msg)
                print(f"\n--- HEARTBEAT ---")
                print(f"System Status: {msg.system_status}")
                print(f"Flight Mode: {flight_mode}")
                print(f"Armed: {'Yes' if armed else 'No'}")

            elif msg_type == "SYS_STATUS":
                voltage_battery = msg.voltage_battery / 1000  # in Volts
                current_battery = msg.current_battery / 100   # in Amperes
                battery_remaining = msg.battery_remaining     # in Percent
                print(f"\n--- SYSTEM STATUS ---")
                print(f"Battery Voltage: {voltage_battery:.2f} V")
                print(f"Battery Current: {current_battery:.2f} A")
                print(f"Battery Remaining: {battery_remaining}%")

            elif msg_type == "GPS_RAW_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0  # in meters
                eph = msg.eph / 100.0
                satellites_visible = msg.satellites_visible
                print(f"\n--- GPS RAW DATA ---")
                print(f"Latitude: {lat:.7f}")
                print(f"Longitude: {lon:.7f}")
                print(f"Altitude: {alt:.2f} m")
                print(f"HDOP: {eph}")
                print(f"Satellites Visible: {satellites_visible}")

            elif msg_type == "ATTITUDE":
                roll = msg.roll * (180.0 / 3.14159265)
                pitch = msg.pitch * (180.0 / 3.14159265)
                yaw = msg.yaw * (180.0 / 3.14159265)
                print(f"\n--- ATTITUDE ---")
                print(f"Roll: {roll:.2f}°")
                print(f"Pitch: {pitch:.2f}°")
                print(f"Yaw: {yaw:.2f}°")

            elif msg_type == "VFR_HUD":
                airspeed = msg.airspeed
                groundspeed = msg.groundspeed
                alt = msg.alt
                climb = msg.climb
                print(f"\n--- VFR HUD ---")
                print(f"Airspeed: {airspeed:.2f} m/s")
                print(f"Groundspeed: {groundspeed:.2f} m/s")
                print(f"Altitude: {alt:.2f} m")
                print(f"Climb Rate: {climb:.2f} m/s")

            elif msg_type == "RAW_IMU":
                print(f"\n--- RAW IMU ---")
                print(f"Accelerometer: X={msg.xacc}, Y={msg.yacc}, Z={msg.zacc}")
                print(f"Gyroscope: X={msg.xgyro}, Y={msg.ygyro}, Z={msg.zgyro}")
                print(f"Magnetometer: X={msg.xmag}, Y={msg.ymag}, Z={msg.zmag}")

            elif msg_type == "BATTERY_STATUS":
                voltages = [v / 1000.0 for v in msg.voltages if v != 0xFFFF]
                current_battery = msg.current_battery / 100.0
                battery_remaining = msg.battery_remaining
                print(f"\n--- BATTERY STATUS ---")
                print(f"Voltages: {voltages} V")
                print(f"Battery Current: {current_battery:.2f} A")
                print(f"Battery Remaining: {battery_remaining}%")

            elif msg_type == "RC_CHANNELS":
                channels = msg.channels[:msg.chancount]
                print(f"\n--- RC CHANNELS ---")
                for i, ch in enumerate(channels, 1):
                    print(f"Channel {i}: {ch}")

            elif msg_type == "SERVO_OUTPUT_RAW":
                print(f"\n--- SERVO OUTPUT ---")
                for i in range(1, 9):
                    servo_value = getattr(msg, f'servo{i}_raw', None)
                    if servo_value is not None:
                        print(f"Servo {i}: {servo_value}")

            else:
                # Uncomment the following line to see all message types
                # print(f"Received message of type: {msg_type}")
                pass

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    """
    Main function to run the script.
    """
    # Update the connection string and baudrate as needed
    connection_string = '/dev/ttyACM0'
    baudrate = 57600

    master = connect_to_pixhawk(connection_string, baudrate)
    display_data(master)

if __name__ == "__main__":
    main()
