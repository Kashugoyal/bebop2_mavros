#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker


dist_pub = rospy.Publinsher('/obstacle_distance', Float32, queue_size = 5)

def ar_cb(data):
    if data:
        for tag in data.markers:
            distances = []
            distances.append(math.sqrt(tag.pose.position.x**2 + tag.pose.position.y**2 + tag.pose.position.z**2))
            dist_pub.publish(min(distances))

rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_cb)

if __name__ == '__main__':
    rospy.spin()