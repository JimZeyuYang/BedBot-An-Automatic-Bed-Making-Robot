#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int32
import numpy as np
import time

pub = rospy.Publisher('/sonar_state', Int32, queue_size=1)
points = []
status = 0
flag_on = False

Inner_threshold = 1.1
Outer_threshold = 2
start_angle = 100

def calc_angle(coord):
    vec_a = coord
    vec_b = [1, 0]
    dot_p = np.dot(vec_a,vec_b)
    abs_p = np.linalg.norm(vec_a)*np.linalg.norm(vec_b)
    angle = np.arccos(dot_p/abs_p)*180/math.pi

    return angle

def callback(data):   
    global status, flag_on
    dist_list = [] 
    sensor_list = []
    data_points = data.points
    for item in data.points:
        temp = [round(item.x,3), round(item.y,3)]
        distance = np.linalg.norm(temp)
        #print(distance)
        dist_list.append(round(distance, 2))
        if distance <= Inner_threshold:
            angle = calc_angle(temp)
            if angle >= start_angle:
                sensor_list.append(1)

        elif distance >= Outer_threshold:
            sensor_list.append(0)

    if sum(sensor_list) != 0 and flag_on == False:
        status = 1
        pub.publish(Int32(status))
        flag_on = True
    elif sum(sensor_list) == 0 and flag_on == True:
        status = 0
        pub.publish(Int32(status))
        flag_on = False

    #pub.publish(Int32(status))


    # status = 1
    # pub.publish(Int32(status))
    # rospy.sleep(5)
    # status = 0
    # pub.publish(Int32(status))
    # rospy.sleep(15)


def main():
    rospy.init_node("Subscriber_Node", anonymous=True)
    rospy.Subscriber('/robot/sonar/head_sonar/state',PointCloud,callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
