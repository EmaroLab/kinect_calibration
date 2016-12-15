kinect_aux
==========

This folder contains a **clone** of this repository: https://github.com/muhrix/kinect_aux

Original README contents follows:

This is a standalone ROS Indigo package which provides access to auxiliary features of the Microsoft kinect, allowing control of the tilt angle and LED status, as well as reading accelerometer data.

This package may be used in conjunction with openni_camera.

For more information: http://www.ros.org/wiki/kinect_aux


# Kinect Calibration with baxter

The original code refered above has been modified in order to care of the frame trasformation between the camera and the robot. Such trasformation can be calibrated with the procedure shown in: [baxter_fram_calibration](https://github.com/EmaroLab/kinect_calibration/tree/master/baxter_frame_calibration).
The result of this procedure is a matrix that has to be updated in the file: [kinect_aux_node.cpp](https://github.com/EmaroLab/kinect_calibration/blob/master/emarolab_kincect_driver/kinect_aux/src/kinect_aux_node.cpp), on the variable `baxterCalibration`

This node manage the trasformation between the camera and baxter when the tilt angle of the Kinect, as well as the pan angle of the head of baxter, changes.

**Important**: this node assures a meningfull calibration only if the kinect starts for 0° (parallel to the ground).
