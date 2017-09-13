# Lab 2 ECE 7785 - Detect a ball and rotate Turtlebot3

The overall goal of the project is to detect the location of a ball and orient TurtleBot 3 to face the ball. From an implementation perspective, this lab deals with Robot Operating System (ROS) and OpenCV (Open-source Computer Vision Library). 

OpenCV methods are used for detecting the ball and ROS is used for linking perception stage to actuation stage. A ROS package named `ball_follower` is created and within the package 2 ROS nodes are implemented - `find_ball` and `drive_wheel`. (The detection script and nodes are implemented in Python)

**Note:** An additional node `ball_rosbag.py` is provided for testing on the rosbag files. This is covered in detail below.

---
## Setting up the ROS package
These instructions will get you a copy of the scripts and running on your local machine for development and testing purposes.

### Download the repository
First download the git repository which contains the ROS package and additional python scripts for user testing.
```
[PC] cd <your_download_directory>
[PC] git clone https://github.com/mouhyemen/ECE7785_Lab2.git 
```

### Move the package to ROS workspace

Simply move the `ball_follower` ROS package from the git folder over to your catkin workspace.

```
[PC] mv <your_download_directory>/ECE7785_Lab2/ball_follower <your_path_to_catkin>/catkin_ws/src
[PC] cd <your_path_to_catkin>/catkin_ws
[PC] catkin_make
[PC] source devel.setup
```
Now you have the `ball_follower` package in your workspace and can try out the nodes inside the package.

---
## Testing the ROS package
The ROS package `ball_follower` can be tested in 3 ways:
* Subscribing to `raspicam_node` on Remote PC and using rosbag files on Remote PC
* Subscribing to `raspicam_node` on TurtleBot3 and testing it on Remote PC (off-board)
* (Optional) Testing color-based perception algorithm using Remote PC's webcam (not part of ROS package)
* Subscribing to `raspicam_node` on TurtleBot3 and testing it on Turtlebot3 (on-board)


### Running rosbag files with *ball_rosbag.py* node

[Download Link](https://drive.google.com/open?id=0B8HUrakRiMyeX3FiQXl2bkhGbVU) - Rosbag Files

Unzip the rosbag file in your working directory. You can use rosbag files to test the node `find_ball` if it is subscribing to the `raspicam_node` and detecting the ball without requiring the TurtleBot 3. 
`ball_rosbag.py` is a ROS node and it needs roscore running, making the workspace, and sourcing the bash file. Also, we need to be in the directory of the rosbag files to play the files.

```
[PC] roscore
[PC] rosrun ball_follower ball_rosbag.py
[PC] cd <ros_bag_files_directory>
[PC] rosbag play --pause view0.bag
```
Press `s` to step through the images or press `spacebar` to play it in real-time.

### Changing the resolution of the Pi-camera

The default launch files provided in the `raspicam_node/launch` directory uses hi-resolution parameters with 30 fps. It can be tweaked to lower the resolution and change the frame rate.

```
[burger] cd ~/catkin_ws/src/raspicam_node/launch
[burger] cp camerav2_1280x960.launch camerav2_320x240.launch
[burger] sudo nano camerav2_320x240.launch
```
Choose your preferred editor (nano, vim) and edit the parameters for `width`, `height`, and `frame`. Make sure to save the file. Now you can re-launch the ```raspicam_node``` running on Turtlebot3 with the launch file we just created.

```
[burger] roslaunch raspicam_node camerav2_1280x960.launch
```

### (Optional) Running the webcam with *ball_detect.py*

The `ball_detect.py` script can be found under `ECE7785_Lab2/test_scripts/` and can be used either for detecting a ball using your laptop's webcam interface. `ball_detect.py` is NOT a ROS node. You do not need roscore running for it. A tracker is also provided for you to play with the HSV threshold values to be able to find color ranges for different colored balls. This is a quick alternative to debugging the script, tracking color thresholds, editing parameters (e.g. Hough Circles) without the overhead of running ROS and/or requiring a TurtleBot3. Simple execute the following commands:

```
[PC] cd <your_download_directory>/ECE7785_Lab2/test_scripts/
[PC] python ball_detect.py
```

---
## On-board deployment to Turtlebot3

### Moving *ball_follower* package to Pi
This can be done in various ways. I am suggesting to use the secure copy command - `scp`.
First, tar the `ball_follower` package on your Remote PC.
```
[PC] cd <your_path_to_catkin>/catkin_ws/src
[PC] tar -cvf follower.tar ball_follower
```
Now, we will securely transfer this tar file over to Pi over a secure encrypted network connection.

```
[PC] scp follower.tar burger@<ip_address>:~/catkin_ws/src/
```
Next, we need to ssh to TurtleBot 3 and untar the securely transferred `follower.tar` file.
```
[burger] cd ~/catkin_ws/src/
[burger] tar -xvf follower.tar
[burger] ls -l
```
The last two commands will untar the `follower.tar` file and check if it is present in the directory.

### Make & source
```
[burger] cd ~/catkin_ws/
[burger] catkin_make
[burger] source devel/setup.bash
```
### Testing ROS nodes on TurtleBot 3
Now, all the nodes will be run on TurtleBot 3's raspberry pi module. You will need to `ssh` on multiple terminal windows to run each node/launch file.

Launch the `raspicam_node` with preferred choice of resolution and `bringup3` for driving the wheels.
```
[burger] roslaunch raspicam_node camerav2_320x240.launch
[burger] roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
Next, we will run the `find_ball.py` and `drive_wheel.py` nodes on TurtleBot 3 to detect the ball and orient the robot respectively.
```
[burger] rosrun ball_follower find_ball.py
[burger] rosrun ball_follower drive_wheel.py
```

---
## Acknowledgments

* Chia-Wen Kuo - For providing the rosbag files
* Bogdan Vlahov - For explaining how to run rosbag files
* Elizabeth - For explaining how to pass messages between nodes
* Cesar Santoyo - For helping to set up `ROS_MASTER_URI` and `ROS_HOSTNAME` parameters
* Dr. Sonia, Sean, and Dave - For getting up and started with ROS


