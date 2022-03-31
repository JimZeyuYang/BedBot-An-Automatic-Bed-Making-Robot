#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
from std_msgs.msg import Int8
from sensor_msgs.msg import Image

image_folder = "/home/y/ws_baxter/src/baxter_bedbot/gui_images"
state_topic_to_subscribe = "baxter_state_topic"

state = 0

def callback(data):
    global state
    rospy.loginfo(rospy.get_caller_id() + "receive order %s", data.data)
    state = data.data
    
if __name__ == '__main__':
    rospy.init_node('gui_display', anonymous=True)

    # subs
    sub = rospy.Subscriber(state_topic_to_subscribe, Int8, callback, queue_size=10)
    
    # pubs
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    
    # images
    images_groups = []
    
    # images_hellorobot = 
    images_groups.append(["waking0130.png","waking0131.png","waking0132.png","waking0133.png","waking0134.png",
                 "waking0155.png","waking0156.png","waking0157.png","waking0158.png","waking0159.png",
                 "waking0160.png","waking0161.png","waking0162.png","waking0163.png","waking0164.png",
                 "waking0155.png","waking0156.png","waking0157.png","waking0158.png","waking0159.png",
                 "waking0160.png","waking0161.png","waking0162.png","waking0163.png","waking0164.png",
                 "waking0165.png","waking0160.png","waking0160.png","waking0160.png","waking0160.png",
                 "happiness0001.png","happiness0002.png","happiness0003.png","happiness0004.png","happiness0005.png",
                 "happiness0006.png","happiness0007.png","happiness0008.png","happiness0009.png","happiness0010.png",
                 "happiness0011.png","happiness0012.png","happiness0013.png","happiness0014.png","happiness0015.png",
                 "happiness0016.png","happiness0017.png","happiness0018.png","happiness0019.png","happiness0020.png",
                 "happiness0021.png","happiness0022.png","happiness0023.png","happiness0024.png","happiness0025.png",
                 "happiness0026.png","happiness0027.png","happiness0028.png","happiness0029.png","happiness0030.png",
                 "happiness0031.png","happiness0032.png","happiness0033.png","happiness0034.png","happiness0035.png",
                 "happiness0036.png","happiness0037.png","happiness0038.png","happiness0039.png","happiness0040.png",
                 "happiness0041.png","happiness0042.png","happiness0043.png","happiness0044.png","happiness0045.png",
                 "happiness0046.png","happiness0047.png","happiness0048.png","happiness0049.png","happiness0050.png",
                 "happiness0051.png","happiness0052.png","happiness0053.png","happiness0054.png","happiness0055.png"])
    # images_grab = 
    images_groups.append(["grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png",
             "grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png",
             "grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png",
             "grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png",
             "grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png","grab.png",
             "approval0001.png","approval0002.png","approval0003.png","approval0004.png","approval0005.png",
             "approval0006.png","approval0007.png","approval0008.png","approval0009.png","approval0010.png",
             "approval0011.png","approval0012.png","approval0013.png","approval0014.png","approval0015.png",
             "approval0016.png","approval0017.png","approval0018.png","approval0019.png","approval0020.png",
             "approval0021.png","approval0022.png","approval0023.png","approval0024.png","approval0025.png",
             "approval0026.png","approval0027.png","approval0028.png","approval0029.png","approval0030.png",
             "approval0031.png","approval0032.png","approval0033.png","approval0034.png","approval0035.png",
             "approval0036.png","approval0037.png","approval0038.png","approval0039.png","approval0040.png",
             "approval0041.png","approval0042.png","approval0043.png","approval0044.png","approval0045.png",
             "approval0046.png","approval0047.png","approval0048.png","approval0049.png","approval0050.png",
             "approval0051.png","approval0052.png","approval0053.png","approval0054.png","approval0055.png"
             ])
    # images_move = 
    images_groups.append(["move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png",
             "move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png",
             "move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png",
             "move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png",
             "move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png","move.png",
             "approval0001.png","approval0002.png","approval0003.png","approval0004.png","approval0005.png",
             "approval0006.png","approval0007.png","approval0008.png","approval0009.png","approval0010.png",
             "approval0011.png","approval0012.png","approval0013.png","approval0014.png","approval0015.png",
             "approval0016.png","approval0017.png","approval0018.png","approval0019.png","approval0020.png",
             "approval0021.png","approval0022.png","approval0023.png","approval0024.png","approval0025.png",
             "approval0026.png","approval0027.png","approval0028.png","approval0029.png","approval0030.png",
             "approval0031.png","approval0032.png","approval0033.png","approval0034.png","approval0035.png",
             "approval0036.png","approval0037.png","approval0038.png","approval0039.png","approval0040.png",
             "approval0041.png","approval0042.png","approval0043.png","approval0044.png","approval0045.png",
             "approval0046.png","approval0047.png","approval0048.png","approval0049.png","approval0050.png",
             "approval0051.png","approval0052.png","approval0053.png","approval0054.png","approval0055.png"
             ])
    # images_stop = 
    images_groups.append(["stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png",
             "stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png",
             "stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png",
             "stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png",
             "stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png","stop.png"
             "approval0001.png","approval0002.png","approval0003.png","approval0004.png","approval0005.png",
             "approval0006.png","approval0007.png","approval0008.png","approval0009.png","approval0010.png",
             "approval0011.png","approval0012.png","approval0013.png","approval0014.png","approval0015.png",
             "approval0016.png","approval0017.png","approval0018.png","approval0019.png","approval0020.png",
             "approval0021.png","approval0022.png","approval0023.png","approval0024.png","approval0025.png",
             "approval0026.png","approval0027.png","approval0028.png","approval0029.png","approval0030.png",
             "approval0031.png","approval0032.png","approval0033.png","approval0034.png","approval0035.png",
             "approval0036.png","approval0037.png","approval0038.png","approval0039.png","approval0040.png",
             "approval0041.png","approval0042.png","approval0043.png","approval0044.png","approval0045.png",
             "approval0046.png","approval0047.png","approval0048.png","approval0049.png","approval0050.png",
             "approval0051.png","approval0052.png","approval0053.png","approval0054.png","approval0055.png"
             ])
     # images_gotobed = 
    images_groups.append(["gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png",
             "gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png",
             "gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png",
             "gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png", 
             "gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png","gotobed.png",
             "approval0001.png","approval0002.png","approval0003.png","approval0004.png","approval0005.png",
             "approval0006.png","approval0007.png","approval0008.png","approval0009.png","approval0010.png",
             "approval0011.png","approval0012.png","approval0013.png","approval0014.png","approval0015.png",
             "approval0016.png","approval0017.png","approval0018.png","approval0019.png","approval0020.png",
             "approval0021.png","approval0022.png","approval0023.png","approval0024.png","approval0025.png",
             "approval0026.png","approval0027.png","approval0028.png","approval0029.png","approval0030.png",
             "approval0031.png","approval0032.png","approval0033.png","approval0034.png","approval0035.png",
             "approval0036.png","approval0037.png","approval0038.png","approval0039.png","approval0040.png",
             "approval0041.png","approval0042.png","approval0043.png","approval0044.png","approval0045.png",
             "approval0046.png","approval0047.png","approval0048.png","approval0049.png","approval0050.png",
             "approval0051.png","approval0052.png","approval0053.png","approval0054.png","approval0055.png"
             ])
    # images_terminate = 
    images_groups.append(["sleeping0049.png","sleeping0050.png","sleeping0051.png","sleeping0052.png","sleeping0053.png",
                "sleeping0054.png","sleeping0055.png","sleeping0056.png","sleeping0057.png","sleeping0058.png",
                "sleeping0059png","sleeping0060.png","sleeping0061.png","sleeping0062.png","sleeping0063.png",
                "sleeping0064.png","sleeping0065.png","sleeping0066.png","sleeping0067.png","sleeping0068.png",
                "sleeping0069.png","sleeping0070.png","sleeping0071.png","sleeping0072.png","sleeping0073.png",
                "sleeping0074.png","sleeping0075.png","sleeping0076.png","sleeping0077.png","sleeping0078.png",
                "sleeping0079png","sleeping0080.png","sleeping0081.png","sleeping0082.png","sleeping0083.png",
                "sleeping0084.png","sleeping0085.png","sleeping0086.png","sleeping0087.png","sleeping0088.png",
                "sleeping0089png","sleeping0090.png","sleeping0091.png","sleeping0092.png","sleeping0093.png",
                "sleeping0094.png","sleeping0095.png","sleeping0096.png","sleeping0097.png","sleeping0098.png"])
    # images_error = 
    images_groups.append(["confusion0001.png","confusion0002.png","confusion0003.png","confusion0004.png","confusion0005.png",
              "confusion0041.png","confusion0042.png","confusion0043.png","confusion0044.png","confusion0045.png",
              "confusion0046.png","confusion0047.png","confusion0048.png","confusion0049.png","confusion0050.png",
              "confusion0051.png","confusion0052.png","confusion0053.png","confusion0054.png","confusion0055.png",
              "confusion0056.png","confusion0057.png","confusion0058.png","confusion0059.png","confusion0060.png"])
    
    # Interface
    # if state == 0:
    #     images = [image_folder+"/"+image for image in images_hellorobot]
    # if state == 1:
    #     images = [image_folder+"/"+image for image in images_grab]
    # if state == 2:
    #     images = [image_folder+"/"+image for image in images_move]
    # if state == 3:
    #     images = [image_folder+"/"+image for image in images_stop]
    # if state == 4:
    #     images = [imagfolder+"/"+image for image in images_gotobed]
    # if state == 5:
    #     images = [image_folder+"/"+image for image in images_terminate]
    # if state == 6:
    #     images = [image_folder+"/"+image for image in images_error]

    for i in range(len(images_groups)):
        for j in range(len(images_groups[i])):
            images_groups[i][j] = image_folder + "/" + images_groups[i][j]

    print("[GUI module] GUI Launched!")
    while not rospy.is_shutdown():
        images = images_groups[state]
        for path in images:
            img = cv2.imread(path)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
            pub.publish(msg)
            r = rospy.Rate(1500)
            r.sleep()
