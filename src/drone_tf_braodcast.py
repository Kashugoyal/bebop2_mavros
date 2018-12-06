#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class drone():

    def __init__(self):
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.tf_pose)
        

    def tf_pose(self, data):
        if data:
            self.br.sendTransform((data.pose.position.x, data.pose.position.y, data.pose.position.z), ( data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w), rospy.Time.now(), "drone", "local_origin")
        else:
            self.br.sendTransform((0, 0, 0), ( 0, 0, 0, 1.0), rospy.Time.now(), "drone", "local_origin")



if __name__ == '__main__':
    rospy.init_node('drone_tf_broadcaster')
    bebop = drone()
    rospy.spin()
