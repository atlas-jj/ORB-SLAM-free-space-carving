# ORB-SLAM-free-space-carving
Implementation and annotation of Free Space Carving algorithm (CARV) using ORB-SLAM map points and camera poses.

#### more details in original Orb-slam project: https://github.com/raulmur/ORB_SLAM2

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:your-folder/Examples/ROS
echo $ROS_PACKAGE_PATH$

# set env variables
export ROS_MASTER_URI=http://192.168.1.17:11311
export ROS_HOSTNAME=192.168.1.17

# compile base lib:
1 ORB-SLAM library:chmod +x build.sh
2 ./build.sh

# compile ROS:
1 chmod +x build_ros.sh
2 ./build_ros.sh

if say no rule to make target...
delete build folder and build again.

# source bash to run with ros
source your-folder/Examples/ROS/ORB_CARV_Pub/build/devel/setup.bash
cd your-folder

# run
1.roscore
2.run your camera node, e.g., usb-cam launch file.
#### for compressed image: rosrun image_transport republish compressed in:=/touch/usb_cam/image_raw raw out:=/chris/image
3. rosrun ORB_CARV_Pub Mono Vocabulary/ORBvoc.txt chris_logic_HD720.yaml /camera/image_raw:=/usb_cam/image_raw

# Code Structure
-- src/Modeler : implementation of CARV algorithm
-- src : add map points to CARV model, and display in Pangolin. Modified based on original ORB-SLAM2

more details can be found in src/README.md
