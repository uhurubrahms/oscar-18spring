#!/usr/bin/env python

import rospy

from race.msg import drive_param
from race.msg import pid_input
# All needs to be modified later.
kp = 15.0
kd = 0.09
servo_offset = 0.0	# zero correction offset in case servo is misaligned.
prev_error = 0.0

vel_input = 25.0
read_vel = 10.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

#newly added for velocity input test
def callback(data):
    read_vel = data.velocity


def control(data):
    rospy.Subscriber("drive_parameters", drive_param, callback)

    global prev_error
    global vel_input
    global kp
    global kd


    # 1. Scale the error
    # 2. Apply the PID equation on error
    # 3. Make sure the error is within bounds
    pid_error = data.pid_error
    error_p = pid_error * kp
    error_d = kd * (prev_error - pid_error)
    angle = error_p + error_d




    prev_error = pid_error


    msg = drive_param();
    #msg.velocity = vel_input #This vel_input is fixed to 25
    msg.velocity = read_vel #To read keyboard vel input 
    # ==> doesn't work with /rpm and other teensy topics I made.. WHY??????
    # But screen /dev/teensy gives right value in real-time!

    if angle > 100:
        angle = 100
    elif angle < -100:
        angle = -100    

    msg.angle = angle

    print("--- Adjusted angle: "+ str(angle))

    pub.publish(msg)

    rospy.spin()#added
if __name__ == '__main__':
    #global kp
    #global kd
    #global vel_input

    print("Listening to error for PID")
    #kp = input("Enter Kp Value: ")
    #kd = input("Enter Kd Value: ")
    #vel_input = input("Enter Velocity: ")

    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", pid_input, control)
    rospy.spin()
