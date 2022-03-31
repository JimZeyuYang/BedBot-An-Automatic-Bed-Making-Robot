#!/usr/bin/env python
import rospy
from math import sqrt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from numpy import arctan2
import time
import copy

import pandas as pd
import numpy as np

from socket import *
import threading

# from baseBaxter_Bridge_Server_Node import sonar_data


PI = 3.1415926535897

global x
x = 0
global y
y = 0
global theta
theta = 0
global speed
speed = Twist()
global laser_array
laser_array = []
global goal
goal = Point()
goal.x = -4
goal.y = 5

global twi_pub 
global rate

global LaserData
LaserData = LaserScan()

global Odom_Data
Odom_Data = Odometry()


global currentDistance_x
currentDistance_x = 0
global targetDistance_x
targetDistance_x = 4     
     
global currentDistance_y
currentDistance_y = 0
global targetFDistance_y
targetDistance_y = 0.5  

global flag_movingInY
flag_movingInY = False

global flag_movingInX
flag_movingInX = False




# movingSpeed is in m/s
# distance is how far away the ridgeback need to travel
def moveForward(distance, movingSpeed=0.1):
    
    
    global x
    global y
    global speed
    global twi_pub
    # init_x = x
    # init_y = y
    
    global laser_array
    global LaserData
    
    global rate
    
    global currentDistance_x
    
    
    print("moving Forward!!!!!!!")
    
    
    # positive direction (+)
    movingSpeed=abs(movingSpeed)
    
    
    speed.linear.y=0
    speed.linear.z=0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    
    
    # move foward 3m
    # while sqrt(abs(init_x - x)**2 + abs(init_y - y)**2) < 3.01:
    #     print("Current Location (x,y) : ({},{})".format(x,y))
    #     speed.angular.z = 0.0
    #     speed.linear.x = 2.0
    #     twi_pub.publish(speed) # publish the move object
    #     rate.sleep()
    
    # Approximately calibrate +0.3m when the moving speed is 0.5m/s
    # calibration_distance = 0.3
    # distance += calibration_distance
    
    # currentDistance = 0
    t0 = time.time()
    while currentDistance_x<distance:
        speed.linear.x = movingSpeed
        twi_pub.publish(speed) # publish the move object
        rate.sleep()
        
        # calculate the traveled distance
        t1 = time.time()
        currentDistance_x = movingSpeed*(t1-t0)
        print("c_x: {}".format(currentDistance_x))
         
    # stop moving forward
    
    speed.linear.x = 0
    twi_pub.publish(speed) # publish the move object
    rate.sleep()
    print('Arriving at the Destination!')
    print('({}, {})'.format(x, y))

    print(currentDistance_x)


def moveBackward(distance, movingSpeed=0.2):
    global x
    global y
    global speed
    global twi_pub
    # init_x = x
    # init_y = y
    
    global rate
    
    print("moving Backward!!!!!!!")
    
    
    # negative direction (-)
    movingSpeed=-abs(movingSpeed)
    
    speed.linear.y=0
    speed.linear.z=0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    
    
    # move foward 3m
    # while sqrt(abs(init_x - x)**2 + abs(init_y - y)**2) < 3.01:
    #     print("Current Location (x,y) : ({},{})".format(x,y))
    #     speed.angular.z = 0.0
    #     speed.linear.x = -2.0
    #     twi_pub.publish(speed) # publish the move object
    #     rate.sleep()
    
    # Approximately calibrate +0.3m when the moving speed is 0.5m/s
    # calibration_distance = 0.3
    # distance += calibration_distance
    
    currentDistance = 0
    t0 = time.time()
    while currentDistance<distance:
        speed.linear.x = movingSpeed
        twi_pub.publish(speed) # publish the move object
        rate.sleep()
        
        # calculate the traveled distance
        t1 = time.time()
        currentDistance = abs(movingSpeed*(t1-t0))
    
    
    # stop moving backward
    speed.linear.x = 0
    twi_pub.publish(speed) # publish the move object
    rate.sleep()
    # print('Arriving at the Destination!')
    # print('({}, {})'.format(x, y))



# angular_vel is angular velocity in degree per second
# target_angle is the target angle in degree
def turnRight(target_angle, angular_vel=15):
    global speed
    global theta
    global twi_pub
    global rate
    
    print("Turn Right!!!!!!!!!!!!!!!1")
    
    
    #We wont use linear components
    speed.linear.x=0
    speed.linear.y=0
    speed.linear.z=0
    speed.angular.x = 0
    speed.angular.y = 0
    
    
    angular_speed = angular_vel*2*PI/360
    
    # Approximately calibrate -9 degree when the ratating angular speed is 15 degree/s
    # calibration_angle = 9
    # relative_angle = (target_angle+calibration_angle)*2*PI/360
    
    relative_angle = target_angle*2*PI/360
    
    
    # clockwise movement is negative direction (-).
    speed.angular.z = -abs(angular_speed)
    
    
    # Setting the current time for rotation calculus
    # t0 = rospy.Time.now().to_sec()
    t0 = time.time()
    current_angle = 0
    # print("Start Rotating!!!")
    # print("Target Angle in Degree: {}".format(target_angle))
    while current_angle<relative_angle:
        twi_pub.publish(speed)
        # r_temp = rospy.Rate(1)
        # r_temp.sleep()
        rate.sleep()
        # print(current_angle, relative_angle)
        # t1 = rospy.Time.now().to_sec()
        t1 = time.time()
        current_angle = angular_speed*(t1-t0)
        
        # print("Rotated Angle in Degree: {}".format(current_angle))
        
    # stop rotating
    speed.angular.z = 0
    twi_pub.publish(speed)
    rate.sleep()
    # print("Reach to the Target Angle!!!")
    # print("Rotation Angle: {} Degree".format(theta*180/PI))

    

def turnLeft(target_angle, angular_vel=15):
    global speed
    global twi_pub
    global rate
    
    
    print("Turn Left!!!!!!!!!!!!!!!1")
    
    #We wont use linear components
    speed.linear.x=0
    speed.linear.y=0
    speed.linear.z=0
    speed.angular.x = 0
    speed.angular.y = 0
    
    
    angular_speed = angular_vel*2*PI/360
    
    # Approximately calibrate +9 degree when the ratating angular speed is 15 degree/s
    # calibration_angle = 9
    # relative_angle = (target_angle+calibration_angle)*2*PI/360
    
    relative_angle = target_angle*2*PI/360
    
    # counter-clockwise movement is positive direction (+).
    speed.angular.z = abs(angular_speed)
    
    
    # Setting the current time for rotation calculus
    # t0 = rospy.Time.now().to_sec()
    t0 = time.time()
    current_angle = 0
    # print("Start Rotating!!!")
    # print("Target Angle in Degree: {}".format(target_angle))
    while current_angle<relative_angle:
        twi_pub.publish(speed)
        rate.sleep()
        # print(current_angle, relative_angle)
        t1 = time.time()
        current_angle = angular_speed*(t1-t0)
        
        
    # stop rotating
    speed.angular.z = 0
    twi_pub.publish(speed)
    rate.sleep()
    # print("Reach to the Target Angle!!!")
    # print("Rotation Angle: {} Degree".format(theta*180/PI))

# moveLeft is to let the robot move in +y direction without any rotation
def moveLeft(distance, movingSpeed=0.2):
    global x
    global y
    global speed
    global twi_pub
    global rate
    
    print("Moving To Left!!!!!!!!!!!!!!!!!!1")
    
    
    # positve y direction (+)
    movingSpeed=abs(movingSpeed)
    
    
    speed.linear.x=0
    speed.linear.z=0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    
    # Approximately calibrate +0.3m when the moving speed is 0.5m/s
    # calibration_distance = 0.3
    # distance += calibration_distance
    
    currentDistance = 0
    t0 = time.time()
    while currentDistance<distance:
        speed.linear.y = movingSpeed
        twi_pub.publish(speed) # publish the move object
        rate.sleep()
        
        # calculate the traveled distance
        t1 = time.time()
        currentDistance = abs(movingSpeed*(t1-t0))
    
    
    # stop moving to the Left
    speed.linear.y = 0
    twi_pub.publish(speed) # publish the move object
    rate.sleep()
    # print('Arriving at the Destination!')
    # print('({}, {})'.format(x, y))


# moveRight is to let the robot move in -y direction without any rotation
def moveRight(distance, movingSpeed=0.2):
    global x
    global y
    global speed
    global twi_pub
    global rate
    
    print("moving to Right!!!!!!!!!!")
    
    # positve y direction (-)
    movingSpeed=-abs(movingSpeed)
    
    
    speed.linear.x=0
    speed.linear.z=0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    
    # Approximately calibrate +0.3m when the moving speed is 0.5m/s
    # calibration_distance = 0.3
    # distance += calibration_distance
    
    currentDistance = 0
    t0 = time.time()
    while currentDistance<distance:
        speed.linear.y = movingSpeed
        twi_pub.publish(speed) # publish the move object
        rate.sleep()
        
        # calculate the traveled distance
        t1 = time.time()
        currentDistance = abs(movingSpeed*(t1-t0))
    
    
    # stop moving to the Left
    speed.linear.y = 0
    twi_pub.publish(speed) # publish the move object
    rate.sleep()
    # print('Arriving at the Destination!')
    # print('({}, {})'.format(x, y))


def stop():
    global speed
    global twi_pub
    global rate
    
    speed.linear.x=0
    speed.linear.z=0
    speed.linear.y=0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    twi_pub.publish(speed)
    rate.sleep()




# Demostration
def walk_left_To_right():
    moveBackward(distance=0.4)
    moveRight(distance=1.7)
    moveForward(distance=2.8) # 2.8
    turnLeft(angular_vel=15, target_angle=180) #190
    moveRight(distance=1.7)
    moveForward(distance=0.4)

def walk_right_To_left():
    moveBackward(distance=0.4)
    moveLeft(distance=1.7)
    moveForward(distance=2.8)
    turnRight(angular_vel=15, target_angle=180) #180
    moveLeft(distance=1.7)
    moveForward(distance=0.4)





def odom_callback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # rospy.loginfo('(X,Y) : ({},{})'.format(x, y))
    
    
    rot_q = msg.pose.pose.orientation
    
    # Theta is the total angle that the robot has rotated. Clockwise is -; CCW is +
    # When facing toward, theta is 0 degree if the robot didnt have any rotation.
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])   
    
    # rospy.loginfo("Rotated Angle: {} Degree".format(theta/PI*180))
    
     
     
    



def laser_callback(msg):
    global x
    global y
    global theta
    global twi_pub
    global speed
    global laser_array
    global goal
    global LaserData
    
    global currentDistance_x
    global targetDistance_x
    
    global currentDistance_y
    global targetFDistance_y
    
    # turnLeft(target_angle=180)
    
    
    
    # the scanning area of laser scan is around [-135, +135] in degree
    
    laser_array = msg.ranges
    
    # try:
    # temp = list(copy.deepcopy(laser_array[270:810]))
    # temp_list = [j for j in temp if j is not None]
    # temp_list = temp_list.sort()
    # print(temp_list)
    # print(temp_list.__class__)
    # except:
        # print("WTF!!!!")
        # print(temp_list)
    
    
    # rospy.loginfo("updating laser data!!")
    # print("+++++++++++++++++++++++++++++++")
    # print(LaserData)
    # print("+++++++++++++++++++++++++++++++")
    # print(min(laser_array))
    # print("--------------------------------------")
    # # temp = []
    # # for i in laser_array: 
    # #     if i<1.5:
    # #         temp.append((laser_array.index(i), i))
        
    # # print(temp) 
    # # print("Length: {}".format(len(temp))) # 88
    
    # # print(laser_array[:239])
    # # print ("min: {}".format(min(laser_array[:239])))
    # # print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    
    # # scanning the right front area, 240-480
    # front_scan = laser_array[240:480]
    # right_scan = laser_array[:239]
    # left_scan = laser_array[481:]

    # # if not flag:    
    # # Check the starting point if the obstacles enclosed the robot from the front, right and the left side
    # if min(front_scan)<1 and min(right_scan)<1 and min(left_scan)<1:
    #     # moveBackward(distance=1)
    #     print("Sorry, I can't move! Please move the obstacles surrounding me!")
    #     stop()
    # elif min(laser_array)<1:
    #     print(min(laser_array), laser_array.index(min(laser_array)))
    #     # flag_movingInY = True
    #     if laser_array.index(min(laser_array))<360:
    #         print("moving left")
    #         # moveLeft(distance=0.5)
            
    #         t0 = time.time()
    #         if currentDistance_y<targetDistance_y:
    #             speed.linear.y = 0.5
    #             twi_pub.publish(speed) # publish the move object
    #             rate.sleep()
                
    #             # calculate the traveled distance
    #             t1 = time.time()
    #             currentDistance_y = 0.5*(t1-t0)
            
    #     else:
    #         print("moving right")
    #         # moveRight(distance=0.5)
            
    #         t0 = time.time()
    #         if currentDistance_y<targetDistance_y:
    #             speed.linear.y = -0.5
    #             twi_pub.publish(speed) # publish the move object
    #             rate.sleep()
                
    #             # calculate the traveled distance
    #             t1 = time.time()
    #             currentDistance_y = 0.5*(t1-t0)
            
            
            
    # elif min(laser_array)>1:
        
    #     # stop all the previous operation
    #     stop()
        
    #     # initialize the global flag
    #     # flag_movingInY = False
    #     currentDistance_y = 0
        
    #     print((min(laser_array), laser_array.index(min(laser_array))))
    #     print("Moving Forward")
        
    #     # currentDistance = 0
    #     # distance = 1
    #     t0 = time.time()
    #     if currentDistance_x<targetDistance_x:
    #         speed.linear.x = 0.5
    #         twi_pub.publish(speed) # publish the move object
    #         rate.sleep()
            
    #         # calculate the traveled distance
    #         t1 = time.time()
    #         currentDistance_x = 0.5*(t1-t0)
    #     else:
    #         print("Reaching to the Target Position!")
    #         stop()
        #     # syn laser_scan.ranges
        #     laser_array = msg.ranges
        
        
        
        
        # moveForward(distance=0.1)
        # flag = True
            
    # print("Arriving at the destination!")
    #Obstacle Avoidance
    # distance = 0.7
    
    # arrived = False
    # rate = rospy.Rate(1)
    # while not arrived:
    #     # print msg.ranges
    #     # print '++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'
    #     inc_x = goal.x -x
    #     inc_y = goal.y -y
    #     angle_to_goal = arctan2(inc_y, inc_x)

    #     if angle_to_goal > 0:
    #         val = 0.25
    #     else:
    #         val = -0.25

    #     if abs(angle_to_goal - theta) > 0.25:
    #         speed.angular.z = val
    #         speed.linear.x = 0.0
    #     else:
    #         speed.linear.x = 1.0
    #         speed.angular.z = 0.0
        
    #     if abs(inc_x) < distance and abs(inc_y) < distance:
    #         arrived = True
    #         speed.linear.x=0
    #         speed.angualr.z=0
    #         print("destination reached!")
    #         rospy.loginfo("detination reached!")
    #     else:
    #         if msg.ranges[320] and msg.ranges[340] and msg.ranges[360] and msg.ranges[380] and msg.ranges[400] > distance: 
    #         #when no any obstacle near detected
    #             # rospy.loginfo("Move Forward!!!")
    #             speed.linear.x = 0.5 # go (linear velocity)
    #             speed.angular.z = 0.0
    #             # speed.angular.z = 0.1 # rotate (angular velocity)
    #             # speed.loginfo("Circling") #state situation constantly

    #         else: #when an obstacle near detected
    #             rospy.loginfo("An Obstacle Near Detected") #state case of detection
    #             speed.linear.x = 0.0 # stop
    #             speed.angular.z = 0.5 # rotate counter-clockwise
    #             # if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
    #             #     #when no any obstacle near detected after rotation
    #             #     circle.linear.x = 0.5 #go
    #             #     circle.angular.z = 0.1 #rotate
    #     twi_pub.publish(speed) # publish the move object
    #     rate.sleep()


# def sonar_callback(str):
#     print(float(str))

conn = 0
def server():
    global conn



    HOST = "0.0.0.0" # Standard loopback interface address (localhost)
    PORT = 6666 # Port to listen on (non-privileged ports are > 1023)



    sockfd=socket()
    sockfd.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
    sockfd.bind((HOST, PORT))



    sockfd.listen(5)
    print("Waiting for the connection......")

    conn, addr=sockfd.accept()
    print("Connect to",addr)

nlp_cmd = []
def nlp_listener(c):
    global nlp_cmd

    while True:
        temp = int(c.recv(1024).decode())
        if temp == 4:
            nlp_cmd.append(4)
        elif temp == 3:
            stop()
        # cmd = c.recv(1024).decode()
        # print(nlp_cmd)
        time.sleep(1)
        # if nlp_cmd != []:
        # if nlp_cmd != [] and int(nlp_cmd[-1]) == 3:
            # nlp_cmd.pop(-1)
            # print(nlp_cmd)
            # stop()
         




if __name__ == '__main__':
    # global twi_pub 
    # global rate 
    # global currentDistance_x
    # global targetDistance_x
    
    # global x
    # global y
    # global speed
    # global laser_array
    # global LaserData 
    
    
    reversing_Flag = False
    obstacleFlag = False

    server()
    
    rospy.init_node("speed_controller")    
    rate = rospy.Rate(10)
    twi_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/front/scan", LaserScan, laser_callback) #subscribe message 
    
    x = threading.Thread(target=nlp_listener, args=(conn, ))
    x.setDaemon(True)
    x.start()

    
    
    
    # rospy.Subscriber("sonar_topic", String, sonar_callback)
    
    
    # moveForward(distance=1)
    # turnRight(angular_vel=5, target_angle=92)
    # moveBackward(distance=1)
    
    # moveRight(distance=1)
    # moveLeft(distance=1)
    
    # hard-code a path from one side to another side
    # moveBackward(distance=0.3)
    # moveRight(distance=1.7)
    # moveForward(distance=3) # 2.8
    # turnLeft(angular_vel=15, target_angle=180) #190
    # moveRight(distance=1.7)
    # moveForward(distance=0.4)
    
    # walk_left_To_right()
    
    
    # another path
    # moveBackward(distance=0.4)
    # moveLeft(distance=1.7)
    # moveForward(distance=3) 2.8
    # turnRight(angular_vel=15, target_angle=190) #180
    # moveLeft(distance=1.7)
    # moveForward(distance=0.4)
    
    # walk_right_To_left()
    
    
    # while True:
    #     if nlp_cmd != [] and int(nlp_cmd[-1]) == 4:
    #         print(nlp_cmd)
    #         nlp_cmd.pop(-1)
    
    
    stop()
    # # t0 = time.time()
    count = 0
    # while not rospy.is_shutdown():
    
    # counting the horizontal movement times
    h_count = 0
    l_flag = False
    r_flag = False
    while True:
        # if nlp_cmd != []:
            # nlp_temp = int(nlp_cmd.pop(0))
        if ((nlp_cmd != []) and (int(nlp_cmd[-1]) == 4)):
            nlp_cmd.pop(-1)
            while count < 270:
                rospy.loginfo(count)
                # if count > 10:
                #     stop()
                
                
                
                # moving forward in +x direction
                # movingSpeed = 0.5
                # if currentDistance_x < targetDistance_x:

                #     speed.linear.x = movingSpeed
                #     twi_pub.publish(speed) # publish the move object
                #     rate.sleep()
                    
                #     # calculate the traveled distance
                #     t1 = time.time()
                #     currentDistance_x = movingSpeed*(t1-t0)
                #     print("c_x: {}".format(currentDistance_x))
                #     print("t_x: {}".format(targetDistance_x))
                
                # else: 
                #     # stop moving forward
                #     stop()
                #     print('Arriving at the Destination!')
                #     print('({}, {})'.format(x, y))

                
                
            
                # print("+++++++++++++++++++++++++++++++")
                # print(LaserData)
                # print("+++++++++++++++++++++++++++++++")
                # print(min(laser_array))
                # print("--------------------------------------")
                # # temp = []
                # # for i in laser_array: 
                # #     if i<1.5:
                # #         temp.append((laser_array.index(i), i))
                    
                # # print(temp) 
                # # print("Length: {}".format(len(temp))) # 88
                
                # # print(laser_array[:239])
                # # print ("min: {}".format(min(laser_array[:239])))
                # # print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                
                
                
                # print(len(laser_array))
                # print(laser_array)
                
                # # scanning the right front area, 240-480
                right_scan = laser_array[:359]
                front_scan = laser_array[360:720]
                left_scan = laser_array[721:]
                # temp_df = pd.DataFrame(laser_array[120:600])

                try:
                    # # if not flag:    
                    # # Check the starting point if the obstacles enclosed the robot from the front, right and the left side
                    if min(front_scan)<0.5 and min(right_scan)<0.5 and min(left_scan)<0.5:
                        # moveBackward(distance=1)
                        print("Sorry, I can't move! Please move the obstacles surrounding me!")
                        stop()
                        
                        for i in range(30):
                            speed.linear.x = -0.07
                            twi_pub.publish(speed) # publish the move object
                            rate.sleep()
                        stop()
                    if min(laser_array[230:520])<0.7: # front scanning to the right 67.5-5 degree; 540 is right front direction  
                    # if sum(laser_array[270:520]) < sum(laser_array[540:810]):
                    
                        # print(min(laser_array), laser_array.index(min(laser_array)))
                        # print("1111111111111111111111111111111111111111111111111111")
                        # obstacleFlag = True
                        
                        # if abs(sum(laser_array[120:359])-sum(laser_array[360:599]))>100:
                        stop()
                        
                        # temp_df = pd.DataFrame(laser_array[120:600]).replace([np.inf, -np.inf], np.nan).dropna(axis=1)
                        # print(temp_df)
                        
                        # temp_r = sorted(list(pd.DataFrame(list(laser_array[240:360])).replace([np.inf, -np.inf], np.nan).dropna()[0]))[:20]
                        # temp_l = sorted(list(pd.DataFrame(list(laser_array[361:481])).replace([np.inf, -np.inf], np.nan).dropna()[0]))[:20]
                        # temp_l = list(laser_array[361:481]).sort()
                        # print(temp_r)
                        
                        # print(list(laser_array[240:360]))
                        # print(laser_array.__class__)
                        
                        
                        
                        
                        # if laser_array.index(min(laser_array[270:810]))<540:
                        # if sum(laser_array[180:540])<sum(laser_array[541:901]):
                        # if laser_array.index(min(laser_array[180:900]))<540:
                        # if sum(temp_r)<sum(temp_l):
                            # print(min(laser_array), laser_array.index(min(laser_array[120:600])))
                            # print("R:{}, L:{}".format(sum(laser_array[300:360]), sum(laser_array[361:421])))
                        print("moving left")
                            
                            # moveLeft(distance=0.5)
                            
                        
                        # if not l_flag and not r_flag:
                        #     l_flag_2 = True
                        # elif l_flag and r_flag:
                            
                        
                        # l_flag = True
                        
                        # speed.linear.y = 0.1
                        # twi_pub.publish(speed) # publish the move object
                        # rate.sleep()
                        
                        for i in range(20):
                            speed.linear.y = 0.07
                            twi_pub.publish(speed) # publish the move object
                            rate.sleep()
                            
                            # if min(laser_array[700:])<0.3:
                            #     stop()
                            #     break
                            
                            # pass
                        
                        
                        
                        # turnLeft(angular_vel=15, target_angle=15) 
                            
                        # for i in range(20):
                        #     speed.linear.y = 0.1
                        #     twi_pub.publish(speed) # publish the move object
                        #     rate.sleep()    
                            
                        # turnRight(angular_vel=15, target_angle=15)
                            
                        speed.linear.y = 0.04
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                        speed.linear.y = 0.03
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                        speed.linear.y = 0.02
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                        speed.linear.y = 0.01
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                            
                                
                        stop()
                            # print("turn left")
                            # speed.angular.z = 15*PI/180
                            # twi_pub.publish(speed)
                            # rate.sleep()
                                
                    elif min(laser_array[560:810])<0.85:
                    # if sum(laser_array[270:520]) > sum(laser_array[540:810]):
                        # print(min(laser_array), laser_array.index(min(laser_array[120:600])))
                        # print("R:{}, L:{}".format(sum(laser_array[300:360]), sum(laser_array[361:421])))
                        print("moving right")
                            
                        # r_flag = True
                        
                        # speed.linear.y = -0.1
                        # twi_pub.publish(speed) # publish the move object
                        # rate.sleep()
                        for i in range(20):
                            # if min(laser_array[:360])<0.3:
                            #     stop()
                            #     break

                            speed.linear.y = -0.07
                            twi_pub.publish(speed) # publish the move object
                            rate.sleep()
                            
                            
                            
                            
                        speed.linear.y = -0.04
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                        speed.linear.y = -0.03
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                        speed.linear.y = -0.02
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                        speed.linear.y = -0.01
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                                
                        stop()
                            # print("turn right")
                            
                            # speed.angular.z = -15*PI/180
                            # twi_pub.publish(speed)
                            # rate.sleep()
                                
                    # elif min(laser_array[270:810])>=1:
                    else:
                        
                        # stop all the previous operation
                        # stop()

                        
                        # initialize the global flag
                        flag_movingInY = False
                        currentDistance_y = 0
                        
                        # print((min(laser_array), laser_array.index(min(laser_array))))
                        
                        
                        if not obstacleFlag:

                            
                            if reversing_Flag :
                                # check how many angles had been rotated 
                                
                                # rotatedAngle = float(theta)/PI*180
                                if int(float(theta)/PI*180) is not 0:
                                    print("reversing!!!!!!!!!!!!!!111")
                                    if float(theta)/PI*180 < 0:
                                        reverse_Angle = abs(float(theta)/PI*180 - int((float(theta)/PI*180)/360)*360)
                                        # t0 = time.time()
                                        # curr_angle = 0
                                        if reverse_Angle>5:
                                            print("turn to left: {}".format(float(theta)/PI*180))
                                            # print("{}, {}".format(curr_angle, reverse_Angle))
                                            
                                            speed.angular.z = 15*PI/180
                                            twi_pub.publish(speed)
                                            rate.sleep()
                                            
                                            # t1 = time.time()
                                            # curr_angle = 15*PI/180*(t1-t0)
                                        else:
                                            stop()
                                            reversing_Flag = False
                                    else:
                                        reverse_Angle = abs(float(theta)/PI*180 - int((float(theta)/PI*180)/360)*360)
                                        # t0 = time.time()
                                        # curr_angle = 0
                                        if reverse_Angle>5:
                                            print("turn to right: {}".format(float(theta)/PI*180))
                                            # print("{}, {}".format(curr_angle, reverse_Angle))
                                        
                                            speed.angular.z = -15*PI/180
                                            twi_pub.publish(speed)
                                            rate.sleep()
                                            
                                            # t1 = time.time()
                                            # curr_angle = 15*PI/180*(t1-t0)
                                        else:
                                            stop()
                                            reversing_Flag = False
                            # else:
                        
                        print("Moving Forward")
                        speed.linear.x = 0.1
                        twi_pub.publish(speed) # publish the move object
                        rate.sleep()
                        count += 1
                        
                
                            
                        # else:
                            # print("Obstacle is nearby!")

                            # for i in range(20):
                            #     speed.linear.x = 0.5
                            #     twi_pub.publish(speed) 
                            #     rate.sleep()
                            
                            # stop()

                            # obstacleFlag = False
                            # reveresing_Flag = True
                            # print("Starting Reversing!!!!!!!!")
                except:
                    pass
            
            moveLeft(distance=1.7)
            turnLeft(angular_vel=15, target_angle=180)   
            # moveForward(distance=0.4)
            
            print("Getting Close to the Bed now!!!")
            temp_list = list(copy.deepcopy(laser_array[270:810]))
            # temp = [j for j in temp_list if j is not None]
            temp_list.sort()
            temp = temp_list[:20]
            while temp > [0.1]:
                moveForward(distance=0.1)
                print("Finetuning@!!!!!!!!!")
                
                temp_list = list(copy.deepcopy(laser_array[270:810]))
                # temp = [j for j in temp_list if j is not None]
                temp_list.sort()
                temp = temp_list[:20]
            
            stop()
            print("Jeffffffffffffffffffffffffffffffffff")
        
        
        
        
    rospy.spin()
    
    
    # while not rospy.is_shutdown():
        
    
    
    
    
    
    # goTo(goal)
    # a = int(input("want to go back ? (0/1)  "))
    # if (a == 1):
    #     goal.x = 0
    #     goal.y = 0
    #     goTo(goal)

    
    