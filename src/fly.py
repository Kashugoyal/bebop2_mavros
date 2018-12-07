#!/usr/bin/env python

import rospy
from ar_tag import drone_adv
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import PoseStamped
import tf


def main():
    rospy.init_node('fly')
    bebop = drone_adv()

    bebop.send_mission_from_file()
    bebop.mode('4')
    bebop.arm(True)
    bebop.take_off(3)
    rospy.loginfo("Going into auto mode")
    bebop.mode('3')
    while not bebop.mission_completed() and not rospy.is_shutdown():
        rospy.sleep(1)

    rospy.loginfo("Mission Completed, returning to land")
    bebop.mode('6')
    while bebop.armed:
        rospy.sleep(1)

    rospy.loginfo('All done ...')


if __name__ == '__main__':
    try:
        main()
    except rospy.ServiceException:
        print '\n.....Shutting Down....'

