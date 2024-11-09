from pymavlink import mavutil
import time

def connect_to_pixhawk(connection_string='/dev/ttyACM0', baudrate=57600):
    """
    Establish a connection to the Pixhawk flight controller.
    """
    print(f"Connecting to Pixhawk on {connection_string} at {baudrate} baud.")
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    master.wait_heartbeat()
    print("Heartbeat received from Pixhawk.")
    return master

def format_imu_data(data_type, timestamp, accelerometer, gyroscope, magnetometer):
    """
    Formats and prints IMU data in a human-readable way.
    """
    print(f"\n--- {data_type} Data ---")
    print(f"Timestamp: {timestamp:.2f} s")
    print("Accelerometer:  X = {:.3f} g, Y = {:.3f} g, Z = {:.3f} g".format(
        accelerometer['x'], accelerometer['y'], accelerometer['z']))
    print("Gyroscope:      X = {:.3f} rad/s, Y = {:.3f} rad/s, Z = {:.3f} rad/s".format(
        gyroscope['x'], gyroscope['y'], gyroscope['z']))
    print("Magnetometer:   X = {:.3f} G, Y = {:.3f} G, Z = {:.3f} G".format(
        magnetometer['x'], magnetometer['y'], magnetometer['z']))

def display_imu_data(master):
    """
    Receive and display IMU data from the Pixhawk in a human-readable format.
    """
    print("Receiving IMU data... Press Ctrl+C to exit.")
    try:
        while True:
            msg = master.recv_match(type=['RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3'], blocking=True)
            if not msg:
                continue

            msg_type = msg.get_type()
            timestamp, accelerometer, gyroscope, magnetometer = None, {}, {}, {}

            if msg_type == "RAW_IMU":
                timestamp = msg.time_usec / 1e6  # Convert microseconds to seconds
                accelerometer = {'x': msg.xacc / 1000.0, 'y': msg.yacc / 1000.0, 'z': msg.zacc / 1000.0}
                gyroscope = {'x': msg.xgyro / 1000.0, 'y': msg.ygyro / 1000.0, 'z': msg.zgyro / 1000.0}
                magnetometer = {'x': msg.xmag / 1000.0, 'y': msg.ymag / 1000.0, 'z': msg.zmag / 1000.0}
                format_imu_data("RAW_IMU", timestamp, accelerometer, gyroscope, magnetometer)

            elif msg_type in ["SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"]:
                timestamp = msg.time_boot_ms / 1000.0  # Convert milliseconds to seconds
                accelerometer = {'x': msg.xacc / 1000.0, 'y': msg.yacc / 1000.0, 'z': msg.zacc / 1000.0}
                gyroscope = {'x': msg.xgyro / 1000.0, 'y': msg.ygyro / 1000.0, 'z': msg.zgyro / 1000.0}
                magnetometer = {'x': msg.xmag / 1000.0, 'y': msg.ymag / 1000.0, 'z': msg.zmag / 1000.0}
                format_imu_data(msg_type, timestamp, accelerometer, gyroscope, magnetometer)

            time.sleep(0.5)  # Adjust this if data is coming in too fast or slow

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    """
    Main function to establish connection and display IMU data.
    """
    connection_string = '/dev/ttyACM0'
    baudrate = 57600
    master = connect_to_pixhawk(connection_string, baudrate)
    display_imu_data(master)

if __name__ == "__main__":
    main()
