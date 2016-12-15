# Skeleton Tracking

This repository contains a skeleton tracker based on the OPENNI driver that can be used in ROS (indigo as been using to set it up).

Follow the [README](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openni_tracker/README.md) in the *openni_tracker* folder for the installation of the driver and utilities.

Nevertheless, this folder contains usefull scripts that can be used to automatise the running of the driver and traker (even from a remote machine). In particular:

| File | Description |
| ------ | ----------- |
| [openni-KinectB.launch](https://github.com/EmaroLab/kinect_calibration/blob/master/openni_skeleton_tracking/openni-KinectB.launch) | Modify the standard *openni.launch* with the name of the kinect used by convention in the lab: `cameraB` and runs the [kinect_aux_node](). |
