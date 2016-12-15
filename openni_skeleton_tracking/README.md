# Skeleton Tracking

This repository contains a skeleton tracker based on the OPENNI driver that can be used in ROS (indigo as been using to set it up).

You should also follow the [README](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openni_tracker/README.md) in the *openni_tracker* ROS package (contains only one node) for the installation of the driver and utilities.

Nevertheless, this folder contains usefull scripts that can be used to automatise the running of the driver and traker (even from a remote machine). In particular:

| File | Description |
| ------ | ----------- |
| [openni-KinectB.launch](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openni-KinectB.launch) | Modify the standard *openni.launch* with the name of the kinect used by convention in the lab: `cameraB` and runs the [kinect_aux_node](https://github.com/EmaroLab/kinect_calibration/tree/master/emarolab_kincect_driver/kinect_aux) for calibrating the camera. |
|[skeleton.launch](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/skeletton.launch)| Runs the [openni_tracker](https://github.com/EmaroLab/kinect_calibration/tree/master/openni_skeleton_tracking/openni_tracker) node and attach the skeleton the the Baxter reference frames. |
|[openniTerminal.sh](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openniTerminal.sh)| export the `ROS_MASTER_URI` and `ROS_IP` in order to use the Baxter ROS core and runs the [openni-KinectB.launch](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openni-KinectB.launch)|
|[openniBaxterisedTerminal.sh](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openniBaxterisedTerminal.sh)| export the `ROS_MASTER_URI` and `ROS_IP` in order to use the Baxter ROS core. Than, it runs the [openni-KinectB.launch](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openni-KinectB.launch) and set the starting position of the kinect to 0° (parallel to the grand, for consistent calibration). Finally it runs the [skeleton.launch](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/skeletton.launch). **Attention:** be sure that the lastlauncher starts after that the previous completes its operation |
|[openniRemoteTerminal.sh](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openniRemoteTerminal.sh)| runs the [openniBaxterisedTerminal.sh](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openniBaxterisedTerminal.sh) from a remote machine.|
