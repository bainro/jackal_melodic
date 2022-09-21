# jackal_melodic
### Setup ROS melodic on Ubunutu 18.04:

```./setup_melodic.sh```

The above script assumes you're using bash,and that this is the only version of ROS installed for the current user.<br/>
See here for more details: [Link](http://wiki.ros.org/melodic/Installation/Ubuntu).

### Run clearpath jackal in gazebo simulation:

```
sudo apt-get install --yes ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation
roslaunch jackal_gazebo jackal_world.launch config:=front_laser &
roslaunch jackal_viz view_robot.launch &
```

### Misc:

If you get python errors about certain packages not being found, yet you're sure they're installed, it could be an issue with the bash variable ```PYTHONPATH```, the version of python being found ```python -V```, or it can be sometimes be resolved by installing with pip instead of apt-get. Eg ```pip3 install rospkg```.

The steps for installing ROS noetic on Ubuntu 20.04 are very similar. Mostly just replacing "melodic" with "noetic" in the apt install commands (including in the BASH scripts). robot.rviz exists for the noetic installation, since as of the time this document was written ros-noetic-jackal-desktop was not installable via apt. For noetic then, do something like this to install and run the simulation:

```
sudo apt install --yes ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
roslaunch jackal_gazebo empty_world.launch &
rviz -d ./robot.rviz
```

Clearpath melodic ROS cheatsheet: [Link](https://www.generationrobots.com/media/ROS_Cheat_Sheet_Melodic.pdf).
Clearpath ROS tutorials: [Link](https://www.clearpathrobotics.com/assets/guides/melodic/ros/index.html).
Clearpath Jackal tutorials: [Link](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/index.html).

#### Left off:
```
rostopic echo imu/data
x, y, z, w = orientation
roll, pitch, yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
# can use yaw for heading substitute, since real bot get's it from magnetometer
```
