# References
this repository is a **clone** of: https://github.com/ros-drivers/openni_tracker
follows the original README contents and some more installation instructions.

## OpenNI Tracker

The OpenNI tracker broadcasts the OpenNI skeleton frames using tf.
For more information checkout the ROS Wiki: http://ros.org/wiki/openni_tracker

## NITE Information

The NITE library must be manually installed for openni_tracker to function.  The two versions that are compatible with this package are 1.5.2.21 and 1.5.2.23.

NITE v1.5.2.23 can currently be downloaded from [http://www.openni.ru/openni-sdk/openni-sdk-history-2/](http://www.openni.ru/openni-sdk/openni-sdk-history-2/) and it is also available in this repository folder

* * *
* * *
* * *

# Other Installation information

## Running
to run the **driver** you need to install:
```
sudo apt-get install ros-indigo-openni-camera 
sudo apt-get install ros-indigo-openni-launch
```
Note that *openni_treacker* is not enymore in the uffical ROS repository but you can take it from here and place in your workspace (follow previous specification about NITE, and do: `catkin_make` to apply the changes). Than, run:
```
roslaunch openni_launch openni.launch
rosrun openni_tracker openni_tracker
```
to make the tracker recognise you, put yourself in the *\Psi* pose and wait for a feedback from the openni_tracker node.

## Licencing
you may experience an **error** like:
`InitFromXml failed: Can't create any node of the requested type!`
In this case you have to enter:
```
sudo niLicense -l 0KOIk2JeIBYClPWVnMoRKn5cdY4=
```

## Kinect Patching 
if by doing this you receive a error message concerning that the **Kinect is not connected**, but indeed it is. You need to install a patch that can be found here: https://github.com/avin2/SensorKinect
<p>
To install it, you should perform the following operations (references can be found here: http://www.sustainablenetworks.org/CIS623/?page_id=54):
```
mkdir ~/kinectdriver
cd ~/kinectdriver
git clone https://github.com/avin2/SensorKinect
cd SensorKinect/Bin/
tar xvjf SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2
cd Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh
```


