#!/usr/bin/env python

import rospy
from ar_tag import drone_adv
from mavros_msgs.msg import HomePosition

def main():
    rospy.init_node('fly')
    bebop = drone_adv()

#############trying ###########

    bebop.send_mission_from_file()
    bebop.mode('4')
    bebop.arm(True)
    bebop.take_off(3)
    rospy.loginfo("Going into auto mode")
    bebop.mode('3')
    rospy.sleep(15)

    bebop.mode('4')
    rospy.sleep(5)
    print "setting home"
    home = bebop.gps_home
    bebop.set_gps_home(curr_gps = True)
    print "waiting before going to first target"
    rospy.sleep(10)

    bebop.go_to_pos(0,0)

    # rospy.sleep(10)

    # bebop.mode('0')
    bebop.go_to_pos(0.0, 5.0)
    # rospy.sleep(10)
    # go forward
    bebop.go_to_pos(5.0, 5.0)
    # rospy.sleep(10)
    # go right
    bebop.go_to_pos(5.0, 0.0)

    bebop.set_gps_home(home.geo.latitude, home.geo.longitude, home.geo.altitude)
    bebop.mode('3')
    
#######################


    while not bebop.mission_completed() and not rospy.is_shutdown():
        # rospy.loginfo("Current waypoint number : {}".format(bebop.curr_wp))
        continue


    rospy.loginfo("Mission Completed, returning to land")
    bebop.mode('6')
    while bebop.armed:
        rospy.sleep(1)

    rospy.loginfo('All done ...')


if __name__ == '__main__':
    try:
        main()
    except:
        print '\n.....Shutting Down....'

