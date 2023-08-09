#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Instantiate CvBridge
bridge = CvBridge()



def listener():

    count = 0
    while count < 1:
        count = count +1
        rospy.init_node('listener', anonymous=True)

        #rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
        msg = rospy.wait_for_message("/camera/image", Image, timeout=None)
        if count == 1:
            
            print("Received an image!")
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError:
                print("error")
            else:
                # Save your OpenCV2 image as a jpeg 
                cv2.imwrite('/app/camera_image.jpeg', cv2_img)
                print("Saved image")
                #cv2.imshow('image',cv2_img)
                #cv2.waitKey(0)
 

if __name__ == '__main__':
    listener()
