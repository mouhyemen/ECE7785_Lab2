# Lab 2 ECE 7785 - Detect a ball and rotate Turtlebot3
The overall goal of the project is to detect the location of a ball and rotate the Turtlebot3 robot to orient itself and face the ball. From an implementation perspective, this lab deals with Robot Operating System (ROS) and OpenCV (Open-source Computer Vision Library). 
OpenCV methods are used for detecting the ball and ROS is used for linking perception stage to actuation stage. A ROS package named ```ball_follower``` is created and within the package 2 ROS nodes are implemented - ```find_ball``` and ```drive_wheel```. (The detection script and nodes are implemented in Python)

## Download the repository

These instructions will get you a copy of the scripts and running on your local machine for development and testing purposes.

```
[PC] cd <your_download_directory>
[PC] git clone https://github.com/mouhyemen/ECE7785_Lab2.git 

```

### Moving the package and making it

Simply move the ```ball_follower``` ROS package from the git folder over to your catkin workspace.

```
[PC] mv <your_download_directory>/ECE7785_Lab2/ball_follower <your_path_to_catkin>/catkin_ws/src
[PC] cd <your_path_to_catkin>/catkin_ws
[PC] catkin_make
[PC] source devel.setup
```

### Running the webcam with *ball_detect.py*

The ```ball_detect.py``` script can be found under ```ECE7785_Lab2/test_scripts/``` and can be used either for detecting a ball using your laptop's webcam interface. ```ball_detect.py``` is NOT a ROS node. You do not need roscore running for it. A tracker is also provided for you to play with the HSV threshold values to be able to find color ranges for different colored balls.
```
[PC] cd <your_download_directory>/ECE7785_Lab2/test_scripts/
[PC] python ball_detect.py
```

### Running rosbag files with *ball_rosbag.py*

You can also use rosbag files to test your node ```find_ball``` if it is subscribing to the ```raspicam_node``` running on the Turtlebot3 and receiving stream of images. ```ball_rosbag.py``` is a ROS node and you need roscore running, need to make your catkin workspace, and source your bash file.

```
[burger] roslaunch raspicam_node camerav2_1280x960.launch

[Remote PC] rosrun ball_follower ball_rosbag.py
```

### Changing the resolution of the Pi-camera

The default launch files provided in the ```raspicam_node/launch``` directory uses hi-resolution parameters with 30 fps. It can be tweaked to lower the resolution and change the frame rate.

```
[burger] cd ~/catkin_ws/src/raspicam_node/launch
[burger] cp camerav2_1280x960.launch camerav2_320x240.launch
[burger] sudo nano camerav2_320x240.launch
```
Choose your preferred editor (nano, vim) and edit the parameters for ```width```, ```height```, and ```frame```. Make sure to save the file. Now you can re-launch the ```raspicam_node``` running on Turtlebot3 with the launch file we just created.

```
[burger] roslaunch raspicam_node camerav2_1280x960.launch
```





## On-board deployment to Turtlebot3

### Moving *ball_follower* package to Pi
Tar the *ball_follower* package on your Remote PC
```
```

Untar the file on Raspberry Pi

### 

Add additional notes about how to deploy this on a live system

## Built With

* 
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc


