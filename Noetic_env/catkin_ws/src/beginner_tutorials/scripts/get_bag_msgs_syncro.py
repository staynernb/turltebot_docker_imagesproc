#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage 
import rosbag
from cv_bridge import CvBridge
import cv2 as cv
import os
import numpy as np

    
# spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        bag = rosbag.Bag("/app/bag.bag", "r")
        bridge = CvBridge()
        count = 0
        while 1:
            # In ROS, nodes are uniquely named. If two nodes with the same
            # name are launched, the previous one is kicked off. The
            #   anonymous=True flag means that rospy will choose a unique
            # name for our 'listener' node so that multiple listeners can
            # # run simultaneously.
            rospy.init_node('listener', anonymous=True)

            #rospy.Subscriber("/sensorring_cam3d/rgb/image_raw", Image, callback)
            img = rospy.wait_for_message("/sensorring_cam3d/rgb/image_raw", Image, timeout=None)
            depth = rospy.wait_for_message("/sensorring_cam3d/depth/image_rect", Image, timeout=None)
            
            #scan = rospy.wait_for_message("/scan_unified", LaserScan, timeout=None)

            print("time depth: ",depth.header.stamp.secs )
            print("time rgb: ",img.header.stamp.secs )

            i = 0
            get_tf =0
            
            while get_tf == 0:
                tf_list = rospy.wait_for_message("/tf", TFMessage, timeout=None)
                for tf in tf_list.transforms:
                    if(tf.header.frame_id == "/odom_combined"):
                        get_tf = 1
                        print("time tf: ",tf.header.stamp.secs)
                        

                        cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
                        cv.imwrite(os.path.join("/app/images", "frame%06i.png" % count), cv_img)

                        cv_depth = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
                        image_np = np.asarray(cv_depth)
                        np.save(os.path.join("/app/depth", "frame%06i.npy" % count), image_np)

                        
                        
                        print ("Wrote image %i" % count )
                        print("")
                        count = count + 1
    except KeyboardInterrupt:
        pass