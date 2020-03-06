#!/usr/bin/env python

# Node do testowania path_extractor
# Przekazuje do tf prosta transformacje map - base_link

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import numpy as np

UPDATE_RATE = 10

# 0 - straight line
# 1 - circle
trajectory = 1

if __name__ == '__main__':
    rospy.init_node('position_publisher')
    rate = rospy.Rate(UPDATE_RATE)

    dx = 0.1
    dy = 0.1
    x = 0
    y = 0
    if trajectory == 0:
        max_x = 10
        max_y = 10
    elif trajectory == 1:
        max_x = 1000
        max_y = 1000
    while not rospy.is_shutdown():
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        if trajectory == 0:
            t.transform.translation.x = x - max_x/2
            t.transform.translation.y = y - max_y/2
        elif trajectory == 1:
            t.transform.translation.x = 4*np.sin(x)
            t.transform.translation.y = 4*np.cos(x)
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

        x = (x+dx) % max_x
        y = (y+dy) % max_y
        rate.sleep()
