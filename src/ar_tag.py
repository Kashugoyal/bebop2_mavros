#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker

rospy.init_node('obstacle_distance')
dist_pub = rospy.Publisher('/obstacle_distance', Float32, queue_size = 5)

def ar_cb(data):
    if data:
        for tag in data.markers:
            distances = []
            distances.append(math.sqrt(tag.pose.pose.position.x**2 + tag.pose.pose.position.y**2 + tag.pose.pose.position.z**2))
            dist_pub.publish(min(distances))

rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_cb)

rospy.spin()