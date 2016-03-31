#!/bin/bash

# open ssh connection and baxterise the terminal. It can launch also the kinect driver

usage="     $(basename "$0") [-h] [-u] [-p] [-x 1 or 0] [-f 1 or 0] [-i] [-m] [-c] [-b] [-a]
script to onpe the ssh connection to the remote terminal. It also baxterise it by running the given script [-b] in the given catkin workspace [-c]. Also, it may run the freenect Kinect driver attached to the ros core shared with Baxter.

where:
	-h  show this help text
	-u  set the user name to do ssh (Default: emarolab)
	-p  user password to connect througth ssh service.
	-x  if it is 0 the ssh connection is normal. If it is 1 (Default) then the -X flag of ssh is used to enable graphic.
	-f  if it is 0 the Kinect driver is not called. if it is 1 (Default) then the Kinect driver is called.
	-i  set the IP of the remote machine to be exported in the ROS_IP variable (Default: 130.251.13.36). Used also to do ssh.
	-m  set the ROS core uri which must be the same used by Baxter (Default: 130.251.13.31)
	-c  set the ROS workspace path of the remote machine (Default: /home/emarolab/catkin_ws/). ATTENTION, do not use symbol like '~' or $'HOME' sice their are resolved on the local machine.
	-b  set the script to Baxterise the terminal on the workspace in the remote machine [-c] (Default: baxter.sh)
	-a  BUGGED !?!?! if it is 0 the terminal will not baxterised and [-c][-b] will not be used. if it is 1 (Default) yes	
"

# default values	
USER=emarolab		# the user name to be connected through ssh service
PASSWORD=eureka		# the password of the user through which connect using ssh service
SSH_GRAPHIC=1		# use (or not use) -X flag on enable graphic through ssh service
FRENECT_RUN=1		# 0=NO, 1=YES --> run the kinectB driver on the specified muster (PASSED TO THE REMOTE SCRIPT  -f)
IP=130.251.13.36	# the ip to connect through ssh (PASSED TO THE REMOTE SCRIPT  -i)
MASTER=130.251.13.31	# the ip of the machine in which the ros core is running (shared with baxter) (PASSED TO THE REMOTE SCRIPT  -m)
WS=/home/emarolab/catkin_ws/		# the path to the baxter configuration on the remote machine (PASSED TO THE REMOTE SCRIPT  -c)
BAXTER=baxter.sh	# the name of the file to configure baxter (PASSED TO THE REMOTE SCRIPT  -b)
BAXTERISE=1		# 0=NO, 1=YES --> baxterise the terminal by runnaning the given baxter.sh in the given workspace

while getopts ':h:u:p:x:f:i:m:c:b:a:' option; do
  case "$option" in
    h) echo "$usage"
       exit
       ;;
    u) USER=$OPTARG
       ;;
    p) PASSWORD=$OPTARG
       ;;
    x) SSH_GRAPHIC=$OPTARG
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


# conenct remotely through ssh service and run the script for baxterise it and possibly run the kinect driver
SSH_PATH=$USER@$IP
if [[ "$SSH_GRAPHIC" == "1" ]]; then
	sshpass -p $PASSWORD ssh -t -X $SSH_PATH "./baxterisedTerminal.sh  -f $FRENECT_RUN -i $IP -m $MASTER -c $WS -b $BAXTER -a $BAXTERISE "
else
	sshpass -p $PASSWORD ssh -t $SSH_PATH "./baxterisedTerminal.sh -f $FRENECT_RUN -i $IP -m $MASTER -c $WS -b $BAXTER -a $BAXTERISE "
fi


