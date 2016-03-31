#!/bin/bash

# open new terminal in the remote machine and baxterised it!!
# runned from baxterisedRemoteTerminal.sh

usage="     $(basename "$0") [-h] [-f 1 or 0] [-i] [-m] [-c] [-b] [-a]
script to baxterise the terminal by running ~/ros_wk/baxter.sh. Also, it may run the freenect KinectB driver attached to the ros core running on the Baxter.

where:
	-h  show this help text
	-f  if it is 0 the Kinect driver is not called. if it is 1 (Default) then the Kinect driver is called.
	-i  set the IP of the local machine to be exported in the ROS_IP variable (Default: 130.251.13.36).
	-m  set the ROS core uri which must be the same used by Baxter (Default: 130.251.13.31)
	-c  set the ROS workspace path (Default: ~/catkin_ws/).
	-b  set the script to Baxterise the terminal on the workspace [-c] (Default: baxter.sh)	
	-a  if it is 0 the terminal will not baxterised and [-c][-b] will not be used. if it is 1 (Default) yes
"
	
# default values
FRENECT_RUN=1		# 0=NO, 1=YES --> run the kinectB driver on the specified muster
IP=130.251.13.36	# this must be the ip of the device in which this script is running
MASTER=130.251.13.31	# this is the ip where the ros core is running (the same of the one used by Baxter)
WS=~/catkin_ws/		# the path to the baxter configuration on this machine
BAXTER=baxter.sh	# the name of the file to configure baxter
BAXTERISE=1		# 0=NO, 1=YES --> baxterise the terminal by runnaning the given baxter.sh in the given workspace

# get input argoments
while getopts ':h:f:i:m:c:b:a:' option; do
  case "$option" in
    h) echo "$usage"
       exit
       ;;
    f) FRENECT_RUN=$OPTARG
       ;;
    i) IP=$OPTARG
       ;;
    m) MASTER=$OPTARG
       ;;
    c) WS=$OPTARG
       ;;
    b) BAXTER=$OPTARG
       ;;
    a) BAXTERISE=$OPTARG
       ;;
    :) printf "missing argument for -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       exit 1
       ;;
   \?) printf "illegal option: -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       exit 1
       ;;
    *)
       echo "$usage" >&2
       exit 1
    ;;
  esac
done
shift $((OPTIND - 1))

# run the kinect driver
if [[ "$FRENECT_RUN" == "1" ]]; then
	# attch to ros installation and workspace
	source /opt/ros/indigo/setup.bash
	source /home/emarolab/catkin_ws/devel/setup.bash

	# attach this to the baxter core
	export ROS_MASTER_URI=http://$MASTER:11311
	export ROS_IP=$IP

	# rus kinect driver
	roslaunch ~/freenect-KinectB.launch &
	# set to 0° for kinect-baxter frame linking (due to calibration)
	rostopic pub /tilt_angle std_msgs/Float64 -- -0
fi

# baxterise the terminal
if [[ "$BAXTERISE" == "1" ]]; then
	cd $WS
	./$BAXTER
fi

