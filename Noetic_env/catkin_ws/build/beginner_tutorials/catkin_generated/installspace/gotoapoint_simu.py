#!/usr/bin/env python3

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def gotogoal(odom,theta,scan):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    move = PoseStamped()

    theta = math.ceil(theta)
    distance = scan.ranges[theta]

    X = odom.pose.pose.position.x
    Y = odom.pose.pose.position.y
    [a,b,W] = euler_from_quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)

    print(distance)
    move.header.stamp = rospy.Time.now()
    move.header.frame_id = "map"
    #move.pose.position.x = -1.81
    #move.pose.position.y = -0.349
    #move.pose.position.z = 0
    
    xr = abs((distance-0.15)*math.cos(theta*math.pi/180))
    yr = abs((distance-0.15)*math.sin(theta*math.pi/180))
    if theta < 0:
        yr = -yr

    n_x = xr*math.cos(W) - yr*math.sin(W)
    n_y = xr*math.sin(W) + yr*math.cos(W)
    #print("n_x:",n_x)
    #print("n_y:",n_y)
    print("X:", X + n_x)
    print("Y:", Y + n_y)
    move.pose.position.x = X + n_x
    move.pose.position.y = Y + n_y
    move.pose.position.z = 0

    move.pose.orientation.w = 0.9986

    rospy.sleep(1)

    pub.publish(move)

    rospy.loginfo("Published node move")


if __name__ == '__main__':
    rospy.init_node('move', anonymous=True)
    try:
        scan = rospy.wait_for_message("/scan", LaserScan, timeout=None)
        odom = rospy.wait_for_message("/odom", Odometry, timeout=None)
        arg = float(sys.argv[1])
        if(arg>100):
             arg = arg - 256
        gotogoal(odom,arg,scan)

    except rospy.ROSInterruptException:
        pass
