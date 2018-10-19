#!/usr/bin/env python
import os
import sys
import rospy
import math
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointPush

class drone():

    def __init__(self):
        self.gps_now = NavSatFix()
        self.pose = PoseStamped()
        self.target_pose = PoseStamped()

        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/mission/push')

        rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.gps_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)

        # set target position publisher object
        self.set_pos = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)

        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.send_wp = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

        rospy.loginfo('Drone initialized')

    def arm(self, val):
        if val:
            rospy.loginfo('Arming the motors')
        else:
            rospy.loginfo('Disarming the motors')
        resp = self.arming(val)
        rospy.loginfo(resp)
        if resp.success == False:
            rospy.logerr('Arming Failed, trying again')
            rospy.sleep(3)
            self.arm(val)



    def cart_distance(self, p1, p2):
        '''
        Returns cartesian distance between two points.
        - points in PoseStamped format
        '''
        return math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 + (p1.pose.position.y - p2.pose.position.y)**2)


    def get_gps_home(self):
        return self.gps_home.latitude , self.gps_home.longitude, self.gps_home.altitude


    def go_to_pos(self,x,y,z = 3.0):
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        rospy.loginfo('Going to point {} , {}, {}'.format(x,y,z))
        self.set_pos.publish(self.target_pose)
        while self.cart_distance(self.target_pose, self.pose) > 1:
            if rospy.is_shutdown():
                break
            pass
        rospy.loginfo('Setpoint achieved')

    def mode(self, mode = '0'):
        name = {"0":"STABILIZE",
                "1":"ACRO",
                "2":"ALT_HOLD",
                "3":"AUTO",
                "4":"GUIDED",
                "5":"LOITER",
                "6":"RTL",
                "7":"CIRCLE",
                "8":"POSITION",
                "9":"LAND"}
        rospy.loginfo('Setting the autopilot mode to {}'.format(name[mode]))
        resp  = self.set_mode(custom_mode = mode)
        if resp.success == False:
            rospy.logerr("Failed to set the mode to {}".format(name[mode]))
            return 
        else:
            rospy.loginfo("Mode set successfully")


    def pose_cb(self,data):
        self.pose = data
        return

    def set_gps_home(self):
        if self.gps_now.header.seq:
            self.gps_home = deepcopy(self.gps_now)
            print 'setting home'
        else:
            rospy.logwarn('GPS not available')
            rospy.sleep(1)
            self.set_gps_home()

    def send_mission(self):
        wp1 = Waypoint()
        wp2 = Waypoint()
        wp3 = Waypoint()
        wp4 = Waypoint()
        wp5 = Waypoint()
        mission = []
        wp1.command = 16
        wp2.command = 16
        wp3.command = 16
        wp4.command = 16
        wp5.command = 16
        wp4.x_lat = 42.168019
        wp4.y_long = -88.54291
        mission.append(wp4)
        wp1.x_lat = 42.168019
        wp1.y_long = -88.537836
        mission.append(wp1)
        wp2.x_lat = 42.161040
        wp2.y_long = -88.537836
        mission.append(wp2)
        wp3.x_lat = 42.161040
        wp3.y_long = -88.54291
        mission.append(wp3)
        wp5.x_lat = 42.168019
        wp5.y_long = -88.54291
        mission.append(wp5)
        rospy.loginfo('Sending mission')
        resp = self.send_wp(waypoints = mission)
        rospy.loginfo(resp)

    def send_mission_from_file(self):
        mission = []
        user_dir = os.path.expanduser('~')
        filename = user_dir + '/ardu_ws/src/bebop2_mavros/path.txt'
        with open(filename,'r') as file:
            for row in file:
                wp = Waypoint()
                x,y = row.split(',')
                wp.command = 16
                wp.x_lat, wp.y_long = map(float,row.split(','))
                mission.append(wp)
        rospy.loginfo('Sending mission')
        resp = self.send_wp(waypoints = mission)
        rospy.loginfo(resp)

    def setpoint_from_file(self):
        user_dir = os.path.expanduser('~')
        filename = user_dir + '/ardu_ws/src/bebop2_mavros/path_local.txt'

        with open(filename,'r') as file:
            for row in file:
                y,x = map(float, row.split(','))
                rospy.loginfo('Going to position {0},{1}'.format(x,y))
                self.go_to_pos(x/1.19,y/1.19, 3.0)

    def take_off(self, alt = 100):
        rospy.loginfo('Taking off')
        resp = self.takeoff(altitude = alt)
        while abs(self.pose.pose.position.z - alt) > 0.1:
            if rospy.is_shutdown():
                break
            pass
        rospy.loginfo(resp)

    def gps_cb(self,data):
        if data:
            self.gps_now.header  = data.header
            self.gps_now.latitude = data.latitude
            self.gps_now.longitude = data.longitude
            self.gps_now.altitude = data.altitude
        return


def main():
    rospy.init_node('fly')
    bebop = drone()
    bebop.set_gps_home()
    bebop.send_mission_from_file()
    bebop.get_gps_home()
    bebop.mode('3')
    bebop.arm(True)
    bebop.take_off(4)
    # bebop.setpoint_from_file()
    # bebop.mode('3')
    # bebop.go_to_pos(-50, 100, 3)
    # bebop.mode('6')
    # rospy.sleep(5)
    # bebop.mode('4')


if __name__ == '__main__':
    try:
        main()
    except rospy.service.ServiceException:
        print '\n.....Shutting Down....'

