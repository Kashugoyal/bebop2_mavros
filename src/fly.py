#!/usr/bin/env python

import rospy
from ar_tag import drone_adv
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import PoseStamped
import tf


def main():
    rospy.init_node('fly')
    bebop = drone_adv()

#############trying ###########

    bebop.send_mission_from_file()
    bebop.mode('4')
    bebop.arm(True)
    bebop.take_off(3)
    # rospy.loginfo("Going into auto mode")
    # bebop.mode('3')
    # rospy.sleep(15)

    bebop.mode('4')
    rospy.sleep(5)
    # print "setting home"
    # home = bebop.gps_home
    # bebop.set_gps_home(curr_gps = True)
    print "waiting before going to first target"
    # rospy.sleep(10)

    listener = tf.TransformListener()

    targ_pose = PoseStamped()
    targ_pose.header.frame_id = 'drone'
    targ_pose.pose.position.x = 0.0
    targ_pose.pose.position.y = 5.0
    targ_pose.pose.position.z = 0.0
    # targ_pose.pose.orientation.x = 0.0
    # targ_pose.pose.orientation.y = 0.0
    # targ_pose.pose.orientation.z = 0.0
    # targ_pose.pose.orientation.w = 1.0

    # rospy.sleep(10)
    listener.waitForTransform("drone", "local_origin", rospy.Time(), rospy.Duration(4.0))

    target = listener.transformPose('local_origin', targ_pose)
    print target.pose.position.x, target.pose.position.y
    # bebop.mode('0')
    bebop.go_to_pos(target.pose.position.x, target.pose.position.y)
    # rospy.sleep(10)
    # go forward
    targ_pose.pose.position.x = 5.0
    targ_pose.pose.position.y = 0.0
    target = listener.transformPose('local_origin', targ_pose)
    bebop.go_to_pos(target.pose.position.x, target.pose.position.y)
    # bebop.go_to_pos(5.0, 5.0)
    # rospy.sleep(10)
    # go right
    targ_pose.pose.position.x = 0.0
    targ_pose.pose.position.y = -5.0
    target = listener.transformPose('local_origin', targ_pose)
    bebop.go_to_pos(target.pose.position.x, target.pose.position.y)
    # bebop.go_to_pos(5.0, 0.0)

    # bebop.set_gps_home(home.geo.latitude, home.geo.longitude, home.geo.altitude)
    # bebop.mode('3')

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
    except rospy.ServiceException:
        print '\n.....Shutting Down....'

