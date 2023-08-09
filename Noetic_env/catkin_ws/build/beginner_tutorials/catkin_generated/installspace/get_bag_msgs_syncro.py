#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage 

def callback(img):
    depth = rospy.wait_for_message("//sensorring_cam3d/depth/image_rect", Image, timeout=None)
    tf = rospy.wait_for_message("/tf", TFMessage, timeout=None)
    scan = rospy.wait_for_message("/scan_unified", LaserScan, timeout=None)

    print("time depth: ",depth.header.stamp )
    print("time rgb: ",img.header.stamp )
    print("time tf: ",tf.header.stamp )
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    #   anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensorring_cam3d/rgb/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()