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

def display_imu_data(master):
    """
    Receive and display IMU data from the Pixhawk.

    Parameters:
        master (mavutil.mavlink_connection): The MAVLink connection object.
    """
    print("Receiving IMU data...")
    try:
        while True:
            # Receive the next MAVLink message
            msg = master.recv_match(type=['RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3'], blocking=True)
            if not msg:
                continue

            msg_type = msg.get_type()

            if msg_type == "RAW_IMU":
                timestamp = msg.time_usec / 1e6  # Convert microseconds to seconds
                accelerometer = {
                    'x': msg.xacc,
                    'y': msg.yacc,
                    'z': msg.zacc
                }
                gyroscope = {
                    'x': msg.xgyro,
                    'y': msg.ygyro,
                    'z': msg.zgyro
                }
                magnetometer = {
                    'x': msg.xmag,
                    'y': msg.ymag,
                    'z': msg.zmag
                }

                print(f"\n--- RAW IMU Data ---")
                print(f"Timestamp: {timestamp:.2f} s")
                print(f"Accelerometer (raw): X={accelerometer['x']} mg, Y={accelerometer['y']} mg, Z={accelerometer['z']} mg")
                print(f"Gyroscope (raw): X={gyroscope['x']} mrad/s, Y={gyroscope['y']} mrad/s, Z={gyroscope['z']} mrad/s")
                print(f"Magnetometer (raw): X={magnetometer['x']} mGauss, Y={magnetometer['y']} mGauss, Z={magnetometer['z']} mGauss")

            elif msg_type == "SCALED_IMU2" or msg_type == "SCALED_IMU3":
                timestamp = msg.time_boot_ms / 1000.0  # Convert milliseconds to seconds
                accelerometer = {
                    'x': msg.xacc / 1000.0,  # mg to g
                    'y': msg.yacc / 1000.0,
                    'z': msg.zacc / 1000.0
                }
                gyroscope = {
                    'x': msg.xgyro / 1000.0,  # mrad/s to rad/s
                    'y': msg.ygyro / 1000.0,
                    'z': msg.zgyro / 1000.0
                }
                magnetometer = {
                    'x': msg.xmag / 1000.0,  # mGauss to Gauss
                    'y': msg.ymag / 1000.0,
                    'z': msg.zmag / 1000.0
                }

                print(f"\n--- {msg_type} Data ---")
                print(f"Timestamp: {timestamp:.2f} s")
                print(f"Accelerometer: X={accelerometer['x']:.3f} g, Y={accelerometer['y']:.3f} g, Z={accelerometer['z']:.3f} g")
                print(f"Gyroscope: X={gyroscope['x']:.3f} rad/s, Y={gyroscope['y']:.3f} rad/s, Z={gyroscope['z']:.3f} rad/s")
                print(f"Magnetometer: X={magnetometer['x']:.3f} Gauss, Y={magnetometer['y']:.3f} Gauss, Z={magnetometer['z']:.3f} Gauss")

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
    display_imu_data(master)

if __name__ == "__main__":
    main()
