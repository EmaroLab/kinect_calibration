#!/bin/bash

# open the driver of a kinect by remote attaching to ros core

# help statament
usage="     $(basename "$0") [-h] [-i] [-m] [-p] [-l]
script to launch the ros driver for a Kinect and skeletton tracker (openni).

where:
	-h  show this help text
	-i  set the IP of this local machine to be exported in the ROS_IP variable (Default: 130.251.13.36).
	-m  set the ROS core uri in which make the core running (Default: 130.251.13.71)
	-p  set the the path in which the ros launcher for the drivier is located (Default in local home).
	-l  set the name of the file launcher (Defualt: freenect-KinectB.launch)
"

# default values
IP=130.251.13.36		# this must be the ip of the device in which this script is running
MASTER=130.251.13.71		# this is the ip where the ros core is running
LAUNCH_PATH=/home/emarolab/	# the path where the freenect launch file is located
LAUNCHER=openni-KinectB.launch # the name of the freenect launch file located in the given path

# get input argoments
while getopts ':h:i:m:p:l:' option; do
  case "$option" in
    h) echo "$usage"
       exit
       ;;
    i) IP=$OPTARG
       ;;
    m) MASTER=$OPTARG
       ;;
    p) LAUNCH_PATH=$OPTARG
       ;;
    l) LAUNCHER=$OPTARG
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

# attch to ros installation and workspace
source /opt/ros/indigo/setup.bash

# attach this to the baxter core
export ROS_MASTER_URI=http://$MASTER:11311
export ROS_IP=$IP #export ROS_IP=`hostname -I`

# print commands to move kinect
printf "\n"
printf "Kinect control with baxter_common and kinect_aux nodes:\n"
printf "adjust tilt angle by using: \t $  rostopic pub /tilt_angle std_msgs/Float64 %s -15\n", "--"
printf "adjust pan angle by using: \t $  rostopic pub /robot/head/command_head_pan baxter_core_msgs/HeadPanCommand %s 0.0 100\n", "--"

# rus kinect driver
roslaunch $LAUNCH_PATH$LAUNCHER #& # se chiudi il terminale non funziona
