#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
import sys
from copy import deepcopy


class drone():

    def __init__(self):
        self.gps_now = NavSatFix()
        rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.gps_cb)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.loginfo('Drone initialized')

    def set_gps_home(self):
        self.gps_home = deepcopy(self.gps_now)
        # print self.gps_home.latitude
        print 'setting home'


    def mode(self, mode = '0'):
        resp  = self.set_mode(custom_mode = mode)
        print resp
        print self.gps_now.latitude, self.gps_home.latitude


    def gps_cb(self,data):
        if data:
            self.gps_now.latitude = data.latitude
            self.gps_now.longitude = data.longitude
            self.gps_now.altitude = data.altitude
        return


def main():
    rospy.init_node('fly')
    bebop = drone()
    bebop.set_gps_home()
    bebop.mode('4')
    rospy.sleep(5)
    bebop.mode('0')
    # bebop.set_gps_home()
    rospy.sleep(5)
    bebop.mode('4')

    sys.exit()

if __name__ == '__main__':
    main()
