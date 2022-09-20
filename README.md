# jackal_melodic
### Setup ROS melodic on Ubunutu 18.04:

```./setup_melodic.sh```

The above script assumes you're using bash,and that this is the only version of ROS installed for the current user.<br/>
See here for more details: [Link](http://wiki.ros.org/melodic/Installation/Ubuntu).

### Run clearpath jackal in gazebo simulation:

```
sudo apt-get install --yes ros-melodic-jackal-simulator
```

### Misc:

Clearpath melodic ROS cheatsheet: [Link](https://www.generationrobots.com/media/ROS_Cheat_Sheet_Melodic.pdf).

https://answers.ros.org/question/39657/importerror-no-module-named-rospkg/?answer=252335#post-id-252335

Left off:
Have to revert the launch file back. There were env variables not setting correctly & xarco was not being found correctly either:
https://robotics.stackexchange.com/questions/23025/invalid-param-tag-cannot-load-command-parameter-robot-description
https://github.com/nickcharron/waypoint_nav/blob/4c8bb70bd376e776c821fc659f492c55b89a342b/husky_simulator/husky_gazebo/launch/spawn_husky.launch#L32
