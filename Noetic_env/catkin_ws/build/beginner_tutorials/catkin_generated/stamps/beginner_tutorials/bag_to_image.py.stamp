#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""
# python bag_to_images.py --bag_file /app/bag.bag --output_dir /app/images --image_topic 

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#import imageio.v3 as iio
import numpy as np
def main():
    """Extract a folder of images from a rosbag.
    """
    #parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    #parser.add_argument("bag_file", help="Input ROS bag.")
    #parser.add_argument("output_dir", help="Output directory.")
    #parser.add_argument("image_topic", help="Image topic.")

    #args = parser.parse_args()

    #print ("Extract images from %s on topic %s into %s" % (args.bag_file,
    #                                                      args.image_topic, args.output_dir))

    #depth_instensity = np.array(256 * depth_image / 0x0fff,
    #                        dtype=np.uint8)
    #iio.imwrite('output/grayscale.png', depth_instensity)

    bag = rosbag.Bag("/app/bag.bag", "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=["/sensorring_cam3d/depth/image_rect"]):
        
        #print(msg)
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        #print(msg)
        cv2.imwrite(os.path.join("/app/depth2", "frame%06i.png" % count), cv_img)
        
        print ("Wrote image %i" % count )
        print("Seq: ", msg.header.seq )
        image_np = np.asarray(cv_img)
        
        np.save(os.path.join("/app/depth_npy", "frame%06i.npy" % count), cv_img)

        #from matplotlib import pyplot as plt
        #plt.imshow(image_np, interpolation='nearest')
        #plt.show()

        count += 1

    bag.close()
    

    return

if __name__ == '__main__':
    main()