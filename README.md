# StereoVision
This is the code for StereoVision emailed to me by Nikita Goldfarb on May 3 2015

Stereo vision

Main packages:
	uvc_camera , uvc_stereo_node	#capture images from the camera
	stereo_image_proc				#rectify images using camera info generated from calibration
									#also generates a depth image/pointcloud but are not used, worse than what camDepth with depth_image_proc creates
									#maybe look into recifying using own program to get rid of unneeded topics produced by this package
	depth_image_proc				#takes depth image created by camDepth and a rgb image to generate a point cloud
	rtabmap_ros						#slam package that takes in pointcloud created by depth_image_proc and odometry.
									#Was not able to get working due to time constraints while testing (3 days before SEcon)

Files used:
	camDepth.py						#takes in the rectified stereo images and creates and publishes a disparity/depth image and rgb image
									#use the trackbars to calibrate the parameters used to create the depth image for best results
									#default values should be fine, but not tested underwater obviously
	stereo.launch					#launches nodes used to take and rectify stereo images from the cameras as well as generate point cloud
	slam.launch 					#launches nodes for rtabmap, slam using pointclouds visible in rviz.  Was not able to get functional 
									#possibly due to tf errors, looks very promising though
									#http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot

Other commands:
	rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.015 right:=/camera/right/image_raw left:=/camera/left/image_raw right_camera:=/camera/right left_camera:=/camera/left
	#runs the stereo calibration program, size and square are the grid size and square size of the chessboard used for calibration
	#http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration

	rosrun image_view image_view image:=topicname
	#view an image being published on a topic, useful to check if images are being rectified after calibration

	rosrun rviz rviz
	#rviz can be used to view the point cloud produced from the depth image and the map produced by rtabmap

Nikita Goldfarb
email any questions to nikitagoldfarb@gmail.com
