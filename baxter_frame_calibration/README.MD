# Baxter Kinect Frame calibration procedure

This tutorial explains you how to properly attach the frames from Freenect ROS drivers to the Baxter frame tree. This is the procedure we followed in Emarolab so everything should already be set up, but you may need to repeat it if the Kinect has been moved or you notice inconsistent results.

To follow this procedure you will need to download and install [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) and print some QR Codes. 

You can install **ar_track_alvar** by running in your terminal:

	sudo apt-get install ros-indigo-ar-track-alvar

Then print the QR markers in `table_8_9_10.png`. We will use the three QR codes a s a bundle to obtain more precise results. You can also print them with a slight zoom so that the camera can detect them easier, just rememember to take note of their size once printed (for us 150% zoom resulted in 6cm markers). 

Remember that if you modify the markers size, then you also have to modify the .xml file defining the bundle. If you work with 6cm markers as we did, you can use the attached `table_8_9_10.xml`, while for 4.4cm markers, an .xml file can be recovered from `ar_track_alvar` repository.

##	Notation

|        Notation       |                      Description             	      |
| --------------------- | --------------------------------------------------- |
|        **_m_**        | Marker (`ar_marker_8`) 							  |
|        **_c_**        | Baxter arm camera frame (`/left_hand_camera`)	   	  |
|       **_rgb_**       | Kinect camera frame (`/cameraB_rgb_optical_frame`)  |
|        **_k_**        | Kinect link frame (`/cameraB_link`) 				  |
|  **_m<sub>c</sub>_**  | Marker frame with respect to **_c_**                |
| **_m<sub>rgb</sub>_** | Marker frame with respect to **_rgb_**              |

## General concepts

The purpose of this procedure is to identify a reasonable approximation of the transformation **T<sub>bk</sub>** between the Kinect base frame `/cameraB_link` (`/camera_link` if you are using the default Kinect driver) and a frame from the Baxter tree (e.g. `/head_camera`, but every frame can work equally well). 

To achieve this purpose we will measure `ar_marker_8` relative position with repect of both a Baxter hand camera (e.g. `/left_hand_camera`) and Kinect RGB camera (**_rgb_**). Once we have **_m<sub>c</sub>_** and **_m<sub>rgb</sub>_**, we will use _tf echo_ to get **_m_** with respect to **_k_** in the Kinect tree and **_m_** with respect to the `/head_camera` in the robot tree. Let's call these transformations **T<sub>a</sub>** and **T<sub>c</sub>**, then we can define the desired transformation **T<sub>bk</sub>** as:

**_T<sub>bk</sub>_** = **_T<sub>a</sub>T<sub>b</sub>T<sub>c</sub>_** 

Where matrix **_T<sub>b</sub>_** represents the eventual rotation between **_m<sub>c</sub>_** and **_m<sub>rgb</sub>_**. In our case, such a rotation matrix is not an idendity matrix, due to the fact that _ar\_track\_alvar_ assigns different rotations to QR Codes if they are observed from a regular camera or a Kinect camera. In our case, the following values were found:

**_T<sub>b</sub>_**  = [0 -1  0  0; 
						1  0  0  0; 
						0  0  1  0; 
						0  0  0  1] 

**NOTE:** **_T<sub>b</sub>_** is not always necessary and it does not always correspond to the suggested value. If you are using this guide as a general procedure and not as a way to calibrate our particular setup, please verify **_T<sub>b</sub>_** exact value in your specific case. Usually, this becomes evident by visualizing both **_m<sub>c</sub>_** and **_m<sub>rgb</sub>_** in Rviz.


##	Steps

The following steps should all be performed in a _**baxterized terminal**_ (run `baxter.sh` in your workspace before any other command in every new terminal tab you open):

1. Place a bundle of markers in such a way that both the Kinect and the camera on the left arm of the baxter can see it. You can tilt the Kinect now if you need it but if you do, take note of the tilt angle.
	* Pay attention to tilt limit angles and frequency of tilt operations. The Kinect motors are very sensitive and they should be used offline for setup purposes, not online (e.g. tracking an object). Also, do not ask the Kinect to tilt above the tilt angle limits or it will get struck, forcing you to manually reset its position.
	* If you tilt the Kinect, remember that the tilt angle is _**relative to the horizon**_ and not to an internal reference. This means that your tilt limits may change depending on the Kinect base inclination in your setup. 
	* If you mount the kinect in a position where it cannot reach the neutral position at startup (e.g. perpendicular to the floor), then the kinect will not be able to move.

2. Run the base kinect drivers. Either remotely or directly from the machine hosting the Kinect.

	roslaunch freenect_launch freenect.launch

3. Launch Rviz.

		rviz

4.  You will need one of the Baxter cameras. We will use the left hand camera. make sure that the camera you are using is open and working at the right resolution.

		rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800

5. Check that the parameters in `bundle_calibration_left_hand_camera.launch` are correct (e.g. the marker dimensions, path to bundle .xml file), then launch it:

		cd /YOUR_TEMP_PATH/kinect_calibration/baxter_frame_calibration 
		roslaunch bundle_calibration_left_hand_camera.launch

	Now, you should be able to see on Rviz **_m<sub>c</sub>_**. Take note of its orientation.

6. Retrieve the trasformation between `/head_camera` (parent) and _**m**_ (child):

		rosrun tf tf_echo /head_camera /ar_marker_8

	This will start printing on screen **_m<sub>c</sub>_** as a traslation [x<sub>t</sub>, y<sub>t</sub>, z<sub>t</sub>] and quaternion rotation [x<sub>q</sub>, y<sub>q</sub>, z<sub>q</sub>, w<sub>q</sub>]. If the readings are noisy, collect a reasonable dataset and average the results.

7. Now open Matlab, move to the `baxter_frame_calibration` folder, then compute the homogeneous trasformation matrix as: 

		ta = [xt, yt, zt]'
		qa = [xq, yq, zq, wq]
		Ta=quaternionToHomogenuesTrasnformation(ta,qa)

	Do not close Matlab.

8. Kill the process launched at point **5**.

9. Repeat the same procedure for the Kinect camera. Check `bundle_calibration_kinect_camera.launch` parameters and then launch it:
		
		cd /YOUR_TEMP_PATH/kinect_calibration/baxter_frame_calibration
		roslaunch bundle_calibration_kinect_camera.launch

	Now you should see the marker **_m<sub>rgb</sub>_** on Rviz given with respect to `/camera_link`. Take note of its orientation.

	**NOTE:** At this stage the Baxter frame tree and the kinect frame tree re still separate. If you need to visualize something in Rviz make sure to to modify the reference frame accordingly. This issue will be solved at the end of the preocedure. 

10. Identify _**T<sub>b</sub>**_. Compare the results of point **5** and **8** and check the orientation of  **_m<sub>c</sub>_** with respect to **_m<sub>rgb</sub>_**. If you are using the kinect camera as a depth camera (the default choice of this tutorial),then there should be a rotation between the two frames you are visualizing, resulting in the following transformation matrix.

	**_T<sub>b</sub>_**  = [0 -1  0  0; 
				1  0  0  0; 
				0  0  1  0; 
				0  0  0  1] 
	
	If you are using the kinect like a generica camera (i.e. using the `no_kinect` launch files from `ar_track_alvar`), then **_T<sub>b</sub>_** is just the identity matrix.

11. Retrieve the trasformation between **_m<sub>rgb</sub>_** (parent) and _**k**_ (child):

		rosrun tf tf_echo /ar_marker_8 /camera_link

	let it print some readings and average the results. At the end you should get a transformation described as a traslation [x<sub>t</sub>, y<sub>t</sub>, z<sub>t</sub>] and a quaternion rotation [x<sub>q</sub>, y<sub>q</sub>, z<sub>q</sub>, w<sub>q</sub>].
 
12. Go back to Matlab and type:

		tc = [x, yt, zt]'
		qc = [xq, yq, zq, wq]
		Tc = quaternionToHomogenuesTrasnformation(tc,qc)

13. You can finally compute **_T<sub>bk</sub>_** = **_T<sub>a</sub>T<sub>b</sub>T<sub>c</sub>_**, then transorm it back to the the ROS vector notation.
		
		Tt = Ta * Tb * Tc
		[tt qt] = homegenousTransformToQuaternion(Tt)

	Where `tt` is the translation vector and `qt` is the rotation quaternion.

14. If this is the first time you calibrate your system, try the obtained results by publishing the trasformation that links the Kinect to the robot, e.g.: 
	
		rosrun tf static_transform_publisher 0.0539 0.0099 0.0436 -0.3516 -0.3173 -0.6416 0.6034 /head_camera /camera_link 100

	where the numbers are collected in a vector: 

		[tt qt] = [ttx tty ttz qtx qty qtz qtw]

	Check in rviz that camera link is correctly posed on the robot and that now you can visualize the point cloud with respect to any Baxter frame.

15. (Optional) Move the robot head and check out in rviz that everything is working (e.g. point clouds are correclty behaving). Make sure to move at low speed since now there is a significant load on the head which may result in dangerous overshoots and/or ruin your calibration. 

		rostopic pub /robot/head/command_head_pan baxter_core_msgs/HeadPanCommand -- -0.5 0.2 1
		rostopic pub /robot/head/command_head_pan baxter_core_msgs/HeadPanCommand -- 0.0 0.2 1

16. Modify the calibration values in `kinect_aux_node.cpp`. Compile and run.

The following steps should be followed only if you plan to move the Kinect and dynamically get point clouds in the world frame even when the Kinect is tilting. In the current mounting position this is not possible, but it is possible to achive the same results by using the Baxter head pan/tilt mechanism.

17. Correct your result by the angle you used during frame calibration.

	First, you have to compute _**T<sub>t</sub>**_ expressed in the Kinect base tilt angle: 

	_**T<sub>tilt</sub>**_ = _**T<sub>t</sub>**_\*_**R<sub>tilt</sub>**_

	Where _**R<sub>tilt</sub>**_ is the rotation matrix corresponding to the tilt rotation angle  you used in this calibration procedure. If you did not tilt the Kinect, this corresponds to an identity matrix and you can skip this step. 

	You can compute _**R<sub>tilt</sub>**_ in Matlab in the following way, keep in mind that the tilt angle must be _**expressed in radians**_:

		Rtilt= [cos(tilt), 0, sin(tilt), 0; 0, 1, 0, 0; -sin(tilt), 0, cos(tilt), 0; 0, 0, 0, 1]
		Ttilt = Tt * Rtilt


18. Modify the driver node `kinect_aux_node.cpp` by assigning to variable `baxterCalibration` the obtainted value of _**T<sub>tilt</sub>**_.

19. Finally create a package in your workspace for the driver and compile it. In the Emarolab setup just modify the existing driver package and compile it to apply the change. 

**You can now launch this driver to use the Kinect with the Baxter and move it freely!** 

20. You can verify the results with the following commands to tilt and pan the Kinect:

		rostopic pub /tilt_angle std_msgs/Float64 -- -45
		rostopic pub /robot/head/command_head_pan baxter_core_msgs/HeadPanCommand -- 0.0 10
