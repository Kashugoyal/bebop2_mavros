#!/usr/bin/env python

import rospy
from drone import drone

def main():
    rospy.init_node('fly')
    bebop = drone()
    bebop.set_gps_home()
    bebop.send_mission_from_file()
    bebop.get_gps_home()
    bebop.mode('4')
    bebop.arm(True)
    bebop.take_off(4)
    rospy.loginfo("Going into auto mode")
    bebop.mode('3')
    while not bebop.mission_completed() and not rospy.is_shutdown():
        # rospy.loginfo("Current waypoint number : {}".format(bebop.curr_wp))
        continue

    rospy.loginfo("Mission Completed")
    bebop.mode('6')


    # bebop.setpoint_from_file()
    # bebop.go_to_pos(-50, 100, 3)
    # bebop.mode('6')
    # rospy.sleep(5)
    # bebop.mode('4')


if __name__ == '__main__':
    try:
        main()
    except rospy.service.ServiceException:
        print '\n.....Shutting Down....'

