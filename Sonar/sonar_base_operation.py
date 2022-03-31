#!/usr/bin/env python

import time
from turtle import distance
import rospy
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import PointCloud

import math
import sys

from socket import *

s=socket()
s.connect(("10.0.0.149",6667)) #change address and port in accordance with instructions in readme




#from headSonar_Data_topic.msg from Heads

addr = "/home/y/ws_baxter/src/baxter_bedbot/scripts/data.txt"
string = ""
decisionBuffer = []   
channelBuffer = []

pub = rospy.Publisher('headSonar_Data_topic',String,queue_size=10)
#rospy.init_node('publisher_node',anonymous=True)
#rate = rospy.Rate(1)

def calcAng(vector_1,vector_2):
#alternative methods depending on representation
    # unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    # unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    # dot_product = np.dot(unit_vector_1, unit_vector_2)
    # angle = (np.arccos(dot_product))*180/math.pi
    # if angle>0 and angle<=180:
    #     return -angle
    # else:
    #     return angle-180
    x1,x2 = vector_1[0],vector_1[1]
    y1,y2 = vector_2[0],vector_2[1]
    dot = x1*x2 + y1*y2      # dot product
    det = x1*y2 - y1*x2      # determinant
    angle = math.atan2(det, dot)*180/math.pi  # atan2(y, x) or atan2(sin, cos)
    return angle

def callback(data):
   

    
    for i in range(0,len(data.channels),2):
        # print(j.values)
        j2 = data.channels[i]
        j1 = data.channels[i+1]
        
        

        distances = j1.values

        channels = list(j2.values)

        

        if (len(decisionBuffer)>10):
                decisionBuffer.pop(0)

        if (len(channelBuffer)>10):
                channelBuffer.pop(0)

      
        
        
        if(list(distances) != []):
            
            if(min(distances)<1.1):
                decisionBuffer.append(1)
                index_min = min(range(len(distances)), key=distances.__getitem__)
                # print(min(distances))
                if(channels[index_min] not in [0.0,11.0]):
                    channelBuffer.append(channels[index_min])
            else:
                decisionBuffer.append(0)
        else:
            decisionBuffer.append(0)
        
#10.0.0.149
#Port:6666
#debugging comments during inital testing below        
        print("stop: " + str(evaluateBuffer(decisionBuffer)))
        # print(evaluateBuffer(channelBuffer))
        
        # if(j.name =="SensorId"):        #0-11 clockwise
        #     print(j.values)
        angle = ((evaluateBuffer(channelBuffer) * (360/12)))
        if(angle>=0 and angle<=180):
            angle = -angle
        elif(angle >180):
            angle = (180-angle)*(-1)
        print(angle)
        # s.send("stop: " + str(evaluateBuffer(decisionBuffer))+'\n'.encode())
        # s.send(("angle" + str(angle))+'\n'.encode())
        
        s.send(str(evaluateBuffer(decisionBuffer))+"\n".encode())
        pub.publish(str(evaluateBuffer(decisionBuffer))+","+str(evaluateBuffer(channelBuffer)))
        # time.sleep(0.1)
        for i in range(20):
            pass
        
        
def evaluateBuffer(passBuffer):
    decisionBuffer = passBuffer
    
    # print(round(sum(decisionBuffer)/(len(decisionBuffer)+ sys.float_info.epsilon)))
    return round(sum(decisionBuffer)/(len(decisionBuffer)+ sys.float_info.epsilon))

     

def listener():
    rospy.init_node("Subscriber_Node", anonymous=True)
    rospy.Subscriber('/robot/sonar/head_sonar/state',PointCloud,callback)
    rospy.spin()

    print("Out of loop...")
    # fp = open(addr, "w")
    # fp.write(string)
    # fp.close()


def talk_to_me(angl):
    
    #rospy.loginfo("Publisher Node Started, now publishing msgs")
    #while not rospy.is_shutdown():
    msg = str(angl)
    pub.publish(msg)
    #rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("Upon interruption...")
        fp = open(addr, "w")
        fp.write(string)
        fp.close()