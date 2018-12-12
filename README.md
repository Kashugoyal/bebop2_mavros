# Autonomous Drone for farming

### Objective
 The final objective of this project is to develop an autonomous drone for farming. Given a boundary, the drone should be able to generate trajectory to cover the entire field avoiding static prelabelled obstacles, follow the trajectory with a constant velocity and update the trajectory inorder to avoidng dynamiic obstacles in the path.
 
 The current project is a starting point for achieving this goal. Now the software can generate trajectory given a geofenced boundary and semantically labelled static obstacles, and follow the trajectory, and dodge dynamic obstacles. The obstacle are represented using AT Tags now. 
 
### Uploading ardupilot to Bebop.


You can build ardupilot on a Linux system following the steps in the link.

Or you can add a prebuilt binary using the following steps
Download the latest binary using the link http://firmware.ardupilot.org/Copter/2018-04/2018-04-04-15:04/bebop/

-   Install adb (android debug tool):
```sh
sudo apt-get install android-tools-adb
```
- Connect to the Bebop2’s WiFi network (BebopDrone-XXXX).
- Enable adb server by pressing the power button 4 times.
- Connect to the Bebop’s adb server on port 9050:
```sh
adb connect 192.168.42.1:9050
```
- If the previous command returns an error, try again (press the power button 4 times and retry).
- Remount the system partition as writeable:
```sh
adb shell mount -o remount,rw /
```
- Push the arducopter binary to the Bebop2:
```sh
adb mkdir /data/ftp/internal_000/APM
adb push arducopter /data/ftp/internal_000/APM/
```


### Starting ArduPilot
- Kill the regular autopilot:
```sh
adb shell
kk
```
- Launch Copter:
```sh
cd /data/ftp/internal_000/APM
arducopter -A udp:192.168.42.255:14550:bcast -B /dev/ttyPA1 -C udp:192.168.42.255:14551:bcast -l /data/ftp/internal_000/APM/logs -t /data/ftp/internal_000/APM/terrain
```


### Install QGroundControl
QGroundControl helps us to check whether the drone is properly connected to the base computer and to check whether all the sensor data is flowing properly. This will also help in tuning the Drone parameter. 
-  Download QGroundControl.AppImage.
https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
- Install using the terminal commands:
```sh
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  (or double click)
```
### Tuning the Drone
Follow the steps in the link to tune the drone.

Or manually edit the tuning Parameters using QGroundController. The default parmeters for Parrot Bebop2 is available in the file below
https://github.com/ArduPilot/ardupilot/blob/master/Tools/Frame_params/Parrot_Bebop2.param

To edit the params using QGroundControl go to settings>params then search for the parameter to be changed



### Setting up SITL on Linux
SITL expands to Software in the loop. This will help in replacing the drone with a simulation so that ardupilot can be run on a simulated environmet for testing purposes without using a real hardware in the loop. Follow the below steps to setup SITL
- Clone ArduPilot repository from
https://github.com/ArduPilot/ardupilot.git
```sh
cd ardupilot
git submodule update --init --recursive
```
- Install some required packages
```sh
Tools/scripts/install-prereqs-ubuntu.sh -y
```
- Reload the path (log-out and log-in to make permanent):
```sh
. ~/.profile
```

### Testing SITL with QGroundController
- Start the sim_vehicle 
 ```sh
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map
```
- Start QGroundController 
- Takeoff the drone using QG.
- Move to drone using QG by giving a waypoint.
If the drone follows the QG in the simulation also, this means the setup is working

### Installing MAVROS

```sh
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```
install RQT for ease of use
```sh
sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-rqt-robot-plugins
sudo apt-get install python-catkin-tools
```
Connecting with ROS

roslaunch mavros apm.launch fcu_url:=udp://:14855@
rosrun mavros mavsys mode -c 0 (sets the vehicle to mode "0")
rosrun mavros mavsafety arm (to arm the vehicle)

### Trajectory generation, Path following, and obstacle avoidance

- Clone the code
```sh
git clone https://github.com/Kashugoyal/bebop2_mavros.git
```
- Install necessary libraries 
```sh
sudo apt-get install python-requests
```
https://github.com/ros-drivers/usb_cam

### Testing the code using SITL
- Start the Simulation
```sh 
sim_vehicle.py --console --map -L Lake
```
Lake is the pre-saved location where we want the drone in the simulation to start. This can be edited by adding the location to the ---- file
- Start mavros 
```sh
roslaunch bebop2_mavros apm_sim.launch
```

- Start the camera connected to the computer to act as the dronesfront view camera
```sh
rosrun usb_cam_node _video_device:=/dev/video1 _pixel_format:=’yuyv’ _camera_name:=’camera’ _camera_frame_id:=’camera’    
```
- Start the ar_tag detector
```sh
roslaunch bebop2_mavros ar_tag.launch marker_size:=’12’
```
- Start the drone takeoff and 
```sh
rosrun bebop2_mavros fly.py
```
Now the drone should start following the trajectory generated previosly, and whenever there is an obstacle in the path (i.e an AR Tag ), the drone will either stop and wait until the obstacle is removed, or dodge the obstacle and move continue the mission, depending on our input


