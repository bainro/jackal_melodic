# jackal_melodic
### Setup ROS melodic on Ubunutu 18.04:

```./setup_melodic.sh```

The above script assumes you're using bash,and that this is the only version of ROS installed for the current user.<br/>
See here for more details: [Link](http://wiki.ros.org/melodic/Installation/Ubuntu).

### Run clearpath jackal in gazebo simulation:

```
sudo apt-get install --yes ros-melodic-jackal-simulator ros-melodic-jackal-msgs
roslaunch jackal_gazebo empty_world.launch &
rviz -d ./robot.rviz
```

### Misc:

If you get python errors about certain packages not being found, yet you're sure they're installed, it could be an issue with the bash variable ```PYTHONPATH```, the version of python being found ```python -V```, or it can be sometimes be resolved by installing with pip instead of apt-get. Eg ```pip3 install rospkg```.

Clearpath melodic ROS cheatsheet: [Link](https://www.generationrobots.com/media/ROS_Cheat_Sheet_Melodic.pdf).

#### Left off:
self.heading relies on the IMU's magnetometer data, which gazebo does not have || emulate. So for simulation would need something different for heading, or might be able to debug with heading set manually.
