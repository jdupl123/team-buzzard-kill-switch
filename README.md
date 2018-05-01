# team-buzzard-kill-switch

# How to autonomous driving

## Connect devices and set ports
Check you content of /dev/ttyUSB and /dev/input/ (for example ls /dev/ttyU* and ls /dev/input/*) and figure out how it changes when you connect the arduino and the gamepad. Remember the names of whatever shows up when you connect them and set the port permissions
sudo chmod 777 <insert_port_name_here>

Unless you add your account to the a user group with the correct permissions, you will have to redo this step every time you lose connection!

## Start a roscore on your machine
roscore

## Launch drivers for arduino and gps device:
roslaunch drivers/drivers.launch

both nodes in that file have a port argument that you might have to change depending on your machine, /dev/ttyACM0 and /dev/ttyUSB0 in my case

## Run joy node
rosrun joy joy_node dev:=/dev/input/js0

where the parameter dev is assigned the port that the gamepad is connected to, in my case /dev/input/js0

## Run joy republisher
python joy_republish.py

## How to drive
Now you should be good to go. You'll have to work out the drive commands according to what it says in joy_republish, that is:

throttle/brake : scaling of axes[1]

if button[7] and axes[1] > 0.9 : {
if axes[6]=-1: Go to neutral
if axes[6]=1: Go to reverse
if axes[7]=1: Go to drive
if axes[7]=-1: Go to park
}

if button[6]: stop ignition
if button[7]: start ignition

if axes[2]<0 and axes[5]<0 : run starter motor

steering: scaling of axes[3]

If you need to work out the numbering of the buttons and axes, look at the raw joy node output, that is once the node is running do 
rostopic list
look for topics called *joy* and then do 
rostopic echo <insert_topic_name_here>
That will give you real time gamepad axes and button states.

## Other stuff
More nodes that were prepared and partially tested but never used on the car are
drive controller: python drive_controller_mode.py
gps: python gps_node.py
waypoints: python gps_waypoints.py
waypoint selector: waypoint_select.py
imu: python imu_node.py
state estimator: python state_estimator.py





