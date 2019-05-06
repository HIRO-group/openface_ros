# OpenFace ROS package

## How to run this package

After cloning and building this repo, you can run `roslaunch openface_ros openface_ros.launch` to launch all the setting we need. And then run `rosrun openface_ros openface_realsense` to start main program.

## Functions of OpenFaceRos

* `OpenFaceRos constructor`: For constructor, we need focal length, center of realsense, threshold of distance betwenn gaze vector and target and a flag enable action unit or not.

* `getNose, getLeftPupil, getRightPupil`: These three function will give you position of nose, left pupil and right pupil individually. The location is pixel-based, which means location in showing image.




