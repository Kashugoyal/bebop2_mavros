import os
import sys
import rospy
import math
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, BatteryState
from mavros_msgs.msg import Waypoint, WaypointReached, State, HomePosition
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest
from mavros_msgs.srv import CommandTOL, CommandBool, CommandHome
from mavros_msgs.srv import WaypointPush

class drone():

    def __init__(self):
        self.gps_now = NavSatFix()
        self.gps_home = HomePosition()
        self.pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.num_wp = 0
        self.curr_wp = None
        self.armed = False
        self.curr_mode = None
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        rospy.wait_for_service('/mavros/cmd/set_home')
        rospy.wait_for_service('/mavros/mission/push')
        rospy.wait_for_service('/mavros/set_stream_rate')
        #set up topic subscribers
        rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.gps_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_cb)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.wp_cb)
        rospy.Subscriber('/mavros/battery', BatteryState, self.battery_cb)
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        # set target position publisher object
        self.set_pos = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        # set up service proxies
        self.stream = rospy.ServiceProxy('/mavros/set_stream_rate',StreamRate)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.send_wp = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.set_home = rospy.ServiceProxy('/mavros/cmd/set_home',CommandHome)
        # set stream using the service
        self.set_stream()
        rospy.on_shutdown(self.land)
        rospy.loginfo('Drone initialized ...')


    def arm(self, val):
        if val:
            rospy.loginfo('Arming the motors')
        else:
            rospy.loginfo('Disarming the motors')
        resp = self.arming(val)
        rospy.loginfo(resp)
        if resp.success == False:
            rospy.logerr('Arming Failed, trying again')
            rospy.sleep(5)
            self.arm(val)


    def battery_cb(self,data):
        if data:
            if data.percentage < 0.25:
                rospy.logerr("Battery Critically low, landing now")
                rospy.signal_shutdown(" Node closed, battery issue.")
            elif data.percentage < 0.15:
                rospy.logwarn("Battery Low - {:.2%}".format(data.percentage))
        return


    def cart_distance(self, p1, p2):
        '''
        Returns cartesian distance between two points.
        Input points in PoseStamped format
        '''
        return math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 + (p1.pose.position.y - p2.pose.position.y)**2)


    def get_gps_home(self):
        return self.gps_home.latitude , self.gps_home.longitude, self.gps_home.altitude


    def get_gps_now(self):
        return self.gps_now.latitude , self.gps_now.longitude, self.gps_now.altitude

    def go_to_pos(self,x,y,z = 3.0):
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.target_pose.header.frame_id = 'local_origin_ned'
        rospy.loginfo('Going to point {} , {}, {}'.format(x,y,z))
        self.set_pos.publish(self.target_pose)
        while self.cart_distance(self.target_pose, self.pose) > 1:
            if rospy.is_shutdown():
                break
            pass
        rospy.loginfo('Setpoint achieved')


    def home_cb(self, data):
        if data:
            self.gps_home = data
        return

    def land(self):
        self.mode('9')
        rospy.logwarn("Landing the robot right now ..")
        while self.armed:
            continue
        rospy.sleep(1)


    def mission_completed(self):
        if self.curr_wp < self.num_wp:
            return False
        return True


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
        if resp.mode_sent == False:
            rospy.logerr("Failed to set the mode to {}".format(name[mode]))
            return 
        else:
            rospy.loginfo("Mode set successfully")


    def pose_cb(self,data):
        self.pose = data
        return


    def send_mission(self, mission):

        rospy.loginfo('Sending mission')
        resp = self.send_wp(waypoints = mission)
        rospy.loginfo(resp)


    def send_mission_from_file(self):
        mission = []
        user_dir = os.path.expanduser('~')
        # replace with your workspace path
        ws_path = '/ardu_ws/src'
        filename = user_dir + ws_path + '/bebop2_mavros/scripts/path.txt'
        with open(filename,'r') as file:
            for idx, row in enumerate(file):
                wp = Waypoint()
                x,y = row.split(',')
                wp.command = 16
                # wp.z_alt = 4
                wp.x_lat, wp.y_long = map(float,row.split(','))
                mission.append(wp)
        self.num_wp = idx
        rospy.loginfo('Sending mission, {} waypoints'.format(self.num_wp))
        resp = self.send_wp(waypoints = mission)
        rospy.loginfo(resp)


    def set_gps_home(self, lat = 0.0, lon = 0.0, alt = 0.0, curr_gps = False):
        if self.gps_now.header.seq:
            if curr_gps:
                rospy.loginfo('Setting home to curr_gps')
            else:
                rospy.loginfo('setting home to {} , {}, {}'.format(lat,lon,alt))
            resp = self.set_home(curr_gps, lat, lon, alt)
            rospy.loginfo(resp)
        else:
            rospy.logwarn('GPS not available')
            rospy.sleep(1)
            self.set_gps_home(lat, lon, alt, curr_gps)


    def set_stream(self):
        val = StreamRateRequest()
        val.stream_id = 0
        val.message_rate = 10
        val.on_off = 1
        self.stream(val)
        rospy.loginfo('Stream rate set')


    def setpoint_from_file(self):
        user_dir = os.path.expanduser('~')
        filename = user_dir + '/ardu_ws/src/bebop2_mavros/scripts/path_local.txt'

        with open(filename,'r') as file:
            for row in file:
                y,x = map(float, row.split(','))
                rospy.loginfo('Going to position {0},{1}'.format(x,y))
                self.go_to_pos(x/1.19,y/1.19, 3.0)


    def state_cb(self,data):
        if data:
            self.armed = data.armed
            self.curr_mode = data.mode
        return


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


    def wp_cb(self,data):
        if data:
            if rospy.get_rostime().secs - data.header.stamp.secs < 2:
                self.curr_wp = data.wp_seq
        return


    def test_wp():
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
        wp4.z_alt = 4
        mission.append(wp4)
        wp1.x_lat = 42.168019
        wp1.y_long = -88.537836
        wp1.z_alt = 4
        mission.append(wp1)
        wp2.x_lat = 42.161040
        wp2.y_long = -88.537836
        wp2.z_alt = 4
        mission.append(wp2)
        wp3.x_lat = 42.161040
        wp3.y_long = -88.54291
        wp3.z_alt = 4
        mission.append(wp3)
        wp5.x_lat = 42.168019
        wp5.y_long = -88.54291
        wp5.z_alt = 4
        mission.append(wp5)
        self.send_mission(mission)
