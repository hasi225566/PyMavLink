import time
import sys
from pymavlink import mavutil

def read_yaw(master):
    
    """Here msg gives roll,pitch and yaw and coresponding rollspeed pitchspeed and yaw speed
    in the from of a dictionary.We just need yaw 
    """
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    # print(msg)
    if msg is None:
        return None
    else:
        # Initially converting into degrees 
        yaw_degree=msg.yaw * 180 / 3.14159

        """ 
        Pixhawk By default give yaw ( 0 to 180) and (0 to -180),
        So we have to tweak it a bit to convert it(0 to 360) which will be easier to play with bearing angle
        """
        if(yaw_degree<0):
            return 360 + yaw_degree
        return yaw_degree


if __name__ == '__main__':
    master = mavutil.mavlink_connection("/dev/ttyACM1",baud=115200)
    master.wait_heartbeat()
    target_yaw=int(input("Give your target yaw :"))
    error_margin=10 
    #We are accepting 5 degrees of error margin,actually 10 degree error span
    while True:
        current_yaw = read_yaw(master)
        if current_yaw is not None:
            print(f"Current Yaw: {current_yaw:.2f} degrees")
            if(abs(target_yaw-current_yaw)<error_margin):
                print("Goal Achived")
            else:
                if(current_yaw-target_yaw<0):
                    '''
                    Here deviation is negative ,So it is indicating right ,Clockwise rotation
                    '''
                    print(f"Current Deviation is: {current_yaw-target_yaw:.2f} degrees")
                    print("Target Yaw is in right,Rotate Right")
                else:
                    '''
                    Here deviation is positive ,So it is indicating Left ,Anticlockwise rotation
                    '''
                    print(f"Current Deviation is: {current_yaw-target_yaw:.2f} degrees")
                    print("Target Yaw is in Left,Rotate Left")

            