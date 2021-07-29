<p align="center">
  <h1 align="center">JetBot ROS2 Application with AWS RoboMaker Support</h1>
</p>

ROS2 version of [JetBot ROS Application](https://github.com/jerwallace/aws-robomaker-jetbot-ros) for arm64/aarch64 machine.

### Colaborators
[Computer Fusion Laboratory (CFL) - Temple University College of Engineering](https://sites.temple.edu/cflab/people/)
* [Animesh Bala Ani](https://animeshani.com/)
* [Dr. Li Bai](https://engineering.temple.edu/about/faculty-staff/li-bai-lbai)

### Download Workspace
```
git clone -b aarch64_ros2 https://github.com/ANI717/aws-robomaker-jetbot-ros
mv aws-robomaker-jetbot-ros jetbot
```

### Set IoT Endpoints
```
cd ~/environment/jetbot/assets/scripts
chmod +x *.sh
./set_iot.sh
```

### Setup Credentials for Joystick Control
Copy AWS Credentials to `~/environment/jetbot/assets/scripts/resources/credentials` file and run following commands. Then download the `~/environment/jetbot/assets/teleop` folder for controlling jetbot with virtual joystick.
```
cd ~/environment/jetbot/assets/scripts
./set_credentials.sh
```

### Build, Source and Run Simulation Workspace
```
cd  ~/environment/jetbot/simulation_ws
rosws update
rosdep update
rosdep  install --from-paths src --ignore-src -r -y
```
```
export DISPLAY=:0
colcon build && source install/setup.bash && ros2 launch jetbot_sim_app circle_launch.py
colcon build && source install/setup.bash && ros2 launch jetbot_sim_app teleop_launch.py
```

### Robot URDF file
Please modify `jetbot_description_launch.py` and `spawn_launch.py` from **jetbot_description** package if you want to use different URDF file.
