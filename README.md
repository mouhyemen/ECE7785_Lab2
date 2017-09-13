# Lab 2 ECE 7785 - Detect a ball and rotate Turtlebot3
The overall goal of the project is to detect the location of a ball and rotate the Turtlebot3 robot to orient itself and face the ball. From an implementation perspective, this lab deals with Robot Operating System (ROS) and OpenCV (Open-source Computer Vision Library). OpenCV methods are used for detecting the ball and ROS is used for linking perception stage to actuation stage. A ROS package named *ball_follower* is created and within the package 2 ROS nodes are implemented - *find_ball.py* and *drive_wheel.py*.

## Pereception - Ball Detection (color-based)

These instructions will get you a copy of the scripts and running on your local machine for development and testing purposes.

### Download the repository

```
cd <your_download_directory>
git clone https://github.com/mouhyemen/ECE7785_Lab2.git 

```

### Moving the package and making it

Simply move the *ball_follower* ROS package from the git folder over to your catkin workspace.

```
mv <your_download_directory>/ECE7785_Lab2/ball_follower <your_path_to_catkin>/catkin_ws/src
cd <your_path_to_catkin>/catkin_ws
source devel.setup
```

### Use *ball_detect.py* for webcam testing

The *ball_detect.py* script can be found under *ECE7785_Lab2/test_scripts/* and can be used either for detecting a ball using your laptop's webcam interface. A tracker is also provided for you to play with the HSV threshold values to be able to find color ranges for different colored balls.
```
cd <your_download_directory>/ECE7785_Lab2/test_scripts/
python ball_detect.py
```

### Running rosbag files with *ball_detect.py*

Explain what these tests test and why

```
Give an example
```

### Changing the resolution of the Pi-camera

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
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


