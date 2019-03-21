# ORB-SLAM-free-space-carving

Implementation and annotation of Free Space Carving algorithm (CARV) using ORB-SLAM map points and camera poses.
#### more details in original Orb-slam project: https://github.com/raulmur/ORB_SLAM2

- Maintenance will be available when the author finds a full-time job. Sorry he has to raise two kids with very limited funding.
- More details on free space carving algorithm CARV are in [Incremental Free-Space Carving for Real-Time 3D Reconstruction][1]
- Codes are used in a robotic teleoperation task (from Singapore to Edmonton). Details about this experiment can be found in [Long  range  teleoperation  for  fine  manipulation  tasksunder  time-delay  network  conditions][2].

![](https://github.com/atlas-jj/ORB-SLAM-free-space-carving/blob/master/expOverview.png?raw=true)

- If you find the codes are useful, please cite my [paper][2]. Citations are REALLY valuable, for a PhD student in a not so that famous research group.
- However, if you prefer not to cite due to various reasons (e.g., no enough space in your paper), I totally agree.

# What is it?
- It's basically a real-time method for 3D surface reconstruction.
- It can be used to have a coarse geometric estimation of the unknown remote environment.

![](https://github.com/atlas-jj/ORB-SLAM-free-space-carving/blob/master/Screenshot.png?raw=true)

# How to use?
## set env variables
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:your-folder/Examples/ROS
echo $ROS_PACKAGE_PATH$

## compile base lib:
1. ORB-SLAM library:chmod +x build.sh
2. ./build.sh

## compile ROS:
1 chmod +x build_ros.sh
2 ./build_ros.sh

if errors indicate: no rule to make target...
delete build folder and build again.

## source bash to run with ros
source your-folder/Examples/ROS/ORB_CARV_Pub/build/devel/setup.bash
cd your-folder

## run
1. set env if you networking with other computers:
export ROS_MASTER_URI=http://192.168.1.17:11311
export ROS_HOSTNAME=192.168.1.17
2. roscore
3. run your camera node, e.g., usb-cam launch file.
#### for compressed image: rosrun image_transport republish compressed in:=/touch/usb_cam/image_raw raw out:=/chris/image
4. rosrun ORB_CARV_Pub Mono Vocabulary/ORBvoc.txt chris_logic_HD720.yaml /camera/image_raw:=/usb_cam/image_raw

## Code Structure
-- src/Modeler : implementation of CARV algorithm
-- src : add map points to CARV model, and display in Pangolin. Modified based on original ORB-SLAM2

more details can be found in src/README.md

[1]: https://webdocs.cs.ualberta.ca/~dana/Papers/103dpvt_Lovi.pdf
[2]: TBD
