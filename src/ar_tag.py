#!/usr/bin/env python

import math
import rospy
import tf
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker
from mavros_msgs.srv import WaypointSetCurrent
from geometry_msgs.msg import PoseStamped

from drone import drone

class drone_adv(drone):

    def __init__(self):
        drone.__init__(self)
        self.obstacle_distance = 100.0
        self.obs_flag = False
        self.dodge_obs = rospy.get_param('dodge_obstacle')
        if self.dodge_obs:
            rospy.Subscriber('/obstacle_distance', Float32, self.obs_cb_dodge)
        else:
            rospy.Subscriber('/obstacle_distance', Float32, self.obs_cb)

        # rospy.wait_for_service('/mavros/mission/set_curent')
        self.wp_update = rospy.ServiceProxy('/mavros/mission/set_curent', WaypointSetCurrent)
        # subscriber for obstacle data
        rospy.loginfo('All set ...')

    def obs_cb(self,data):
        self.obstacle_distance = data

        #for stoppping upon obstacle
        if self.obstacle_distance < 0.3:
            if not self.obs_flag:
                self.obs_flag = True
                self.mode('4')
        else:
            if self.obs_flag:
                self.obs_flag = False
                if self.curr_mode == 'GUIDED':
                    self.mode('3')


    def obs_cb_dodge(self,data):
        self.obstacle_distance = data
        if not self.obs_flag:
            if self.obstacle_distance < 0.3:
                self.obs_flag = True
                self.handle_obstacle()
                self.obs_flag = False



    def handle_obstacle(self):
        '''
        trial , still working
        '''
        self.mode('4')
        rospy.sleep(5)
        listener = tf.TransformListener()
        targ_pose = PoseStamped()
        targ_pose.header.frame_id = 'drone'
        targ_pose.pose.position.x = 0.0
        targ_pose.pose.position.y = 5.0
        targ_pose.pose.position.z = 0.0
        listener.waitForTransform("drone", "local_origin", rospy.Time(), rospy.Duration(4.0))
        target = listener.transformPose('local_origin', targ_pose)
        print target.pose.position.x, target.pose.position.y
        # self.mode('0')
        self.go_to_pos(target.pose.position.x, target.pose.position.y)
        rospy.sleep(1)
        # go forward
        targ_pose.pose.position.x = 5.0
        targ_pose.pose.position.y = 0.0
        target = listener.transformPose('local_origin', targ_pose)
        self.go_to_pos(target.pose.position.x, target.pose.position.y)
        rospy.sleep(1)
        # go right
        targ_pose.pose.position.x = 0.0
        targ_pose.pose.position.y = -5.0
        target = listener.transformPose('local_origin', targ_pose)
        self.go_to_pos(target.pose.position.x, target.pose.position.y)
        self.mode('3')



if __name__ == '__main__':
    rospy.init_node('obstacle_handler')
    rospy.spin()