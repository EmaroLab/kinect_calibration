# Kinect driver launchfiles

This folder contains various launchfiles to run our Kinects. 
The default procedure to work with the Baxter is to launch the Kinect through the pc connected to the robot.
You can do that by launching ```baxterizedRemoteTerminal.sh```.

To calibrate the kinect with respect to the Baxter you should add the [kinect_aux](https://github.com/EmaroLab/kinect_calibration/tree/master/emarolab_kincect_driver/kinect_aux) package (that contains only one node) in your workspace; be sure to call `catkin_make`!


**Note:** Before launching it the first time on your machine, you need to exchange the ssh credentials between 
your machine and the pc connected to the Baxter. To do so, run the following command in a terminal:

```ssh emarolab@130.251.13.36```

When you are asked for a password, type *"eureka"*.

This procedure is required **only the first time you log into the Baxter pc from your machine** in order to store the SSH key.
