#!/usr/bin/env python

import math
import rospy
import tf
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker
from mavros_msgs.srv import WaypointSetCurrent
from geometry_msgs.msg import PoseStamped, TwistStamped
from copy import deepcopy
from mavros_msgs.msg import HomePosition

from drone import drone

class drone_adv(drone):

    def __init__(self):
        drone.__init__(self)
        self.obstacle_distance = 100.0
        self.obs_flag = False

        # rospy.wait_for_service('/mavros/mission/set_curent')
        self.wp_update = rospy.ServiceProxy('/mavros/mission/set_curent', WaypointSetCurrent)
        # subscriber for ar tag data
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        #publisher for obstacle distance
        self.dist_pub = rospy.Publisher('/obstacle_distance', Float32, queue_size = 5)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel',TwistStamped, queue_size = 5)
        rospy.loginfo('All set ...')

    def ar_cb(self,data):
        
        if data.markers:
            for tag in data.markers:
                distances = []
                distances.append(math.sqrt(tag.pose.pose.position.x**2 + tag.pose.pose.position.y**2 + tag.pose.pose.position.z**2))
                self.obstacle_distance = min(distances)
                self.dist_pub.publish(self.obstacle_distance)
                if not self.obs_flag:
                    if self.obstacle_distance < 0.3:
                        self.obs_flag = True
                        self.handle_obstacle()
                        self.obs_flag = False
        else:
            self.dist_pub.publish(100.0)
            self.obstacle_distance = 100.0

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
        # targ_pose.pose.orientation.x = 0.0
        # targ_pose.pose.orientation.y = 0.0
        # targ_pose.pose.orientation.z = 0.0
        # targ_pose.pose.orientation.w = 1.0
        # rospy.sleep(10)
        listener.waitForTransform("drone", "local_origin", rospy.Time(), rospy.Duration(4.0))
        target = listener.transformPose('local_origin', targ_pose)
        print target.pose.position.x, target.pose.position.y
        # self.mode('0')
        self.go_to_pos(target.pose.position.x, target.pose.position.y)
        # rospy.sleep(10)
        # go forward
        targ_pose.pose.position.x = 5.0
        targ_pose.pose.position.y = 0.0
        target = listener.transformPose('local_origin', targ_pose)
        self.go_to_pos(target.pose.position.x, target.pose.position.y)
        # self.go_to_pos(5.0, 5.0)
        # rospy.sleep(10)
        # go right
        targ_pose.pose.position.x = 0.0
        targ_pose.pose.position.y = -5.0
        target = listener.transformPose('local_origin', targ_pose)
        self.go_to_pos(target.pose.position.x, target.pose.position.y)
        # self.go_to_pos(5.0, 0.0)
        # self.set_gps_home(home.geo.latitude, home.geo.longitude, home.geo.altitude)
        self.mode('3')


if __name__ == '__main__':
    rospy.init_node('obstacle_distance')
    rospy.spin()