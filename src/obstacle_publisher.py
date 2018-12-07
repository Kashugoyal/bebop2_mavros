#!/usr/bin/env python

import rospy
import socket
import struct
import sys
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker


def ar_cb(data):
    if data.markers:
        for tag in data.markers:
            distances = []
            distances.append(math.sqrt(tag.pose.pose.position.x**2 + tag.pose.pose.position.y**2 + tag.pose.pose.position.z**2))
            obstacle_distance = min(distances)
            dist_pub.publish(obstacle_distance)
    else:
        dist_pub.publish(100.0)
        obstacle_distance = 100.0


def main():

    rospy.init_node('obstacle_publisher')
    sonar_detection = rospy.get_param('enable_sonar')
    dist_pub = rospy.Publisher('/obstacle_distance', Float32, queue_size = 5)

    if sonar_detection:
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #  replace with ip_adderss of pi
        server_address = ('192.168.1.119', 10000)
        # server_address = ('localhost', 10000)

        sock.connect(server_address)

        unpacker = struct.Struct('f')

        while not rospy.is_shutdown():
            data = sock.recv(unpacker.size)
            obstacle_distance = unpacker.unpack(data)
            dist_pub.publish(obstacle_distance)
        sock.close()

    else:
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_cb)
    rospy.spin()


if __name__ == '__main__':
    main()

