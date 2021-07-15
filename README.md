<p align="center">
  <h1 align="center">JetBot ROS Application with AWS RoboMaker Support</h1>
</p>

We modified the [JetBot ROS Application](https://github.com/jerwallace/aws-robomaker-jetbot-ros) for arm64/.aarch64 machine support along with couple of changes for easy user interface. For detailed description of every steps regarding setup workspace, follow the [master](https://github.com/lbaitemple/aws-robomaker-jetbot-ros) branch.

### Colaborators
[Computer Fusion Laboratory (CFL) - Temple University College of Engineering](https://sites.temple.edu/cflab/people/)
* [Animesh Bala Ani](https://animeshani.com/)
* [Dr. Li Bai](https://engineering.temple.edu/about/faculty-staff/li-bai-lbai)

### Download Workspace
```
git clone -b aarch64_melodic https://github.com/ANI717/aws-robomaker-jetbot-ros
mv aws-robomaker-jetbot-ros jetbot
```

### Set IoT Endpoints and Create Docker Container
```
cd ~/environment/jetbot/assets/scripts
chmod +x *.sh
sudo ./set_iot.sh
sudo ./set_docker.sh
```

### Install Dependencies
```
rosdep update
sudo apt install -y cmake gcc g++ qt{4,5}-qmake libqt4-dev
sudo apt -y autoremove
```

### Build, Bundle and Upload Robot Workspace
```
cd  ~/environment/jetbot/robot_ws
rosdep  install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
aws s3 cp ./bundle/output.tar s3://<s3 bucket name>/jetbot_robot_X86_64.tar
```

### Build, Bundle and Upload Simulation Workspace
```
cd  ~/environment/jetbot/simulation_ws
vcs import < .rosinstall
rosdep  install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
aws s3 cp ./bundle/output.tar s3://<s3 bucket name>/jetbot_simulation.tar
```

### Setup Credentials for Joystick Control
Copy AWS Credentials to `~/environment/jetbot/assets/scripts/resources/credentials` file and run following commands. Then download the `~/environment/jetbot/assets/teleop` folder for controlling jetbot with virtual joystick.
```
cd ~/environment/jetbot/assets/scripts
./set_credentials.sh
```

### Robot Deployment
Follow steps from [master](https://github.com/lbaitemple/aws-robomaker-jetbot-ros) branch.

### Test from Cloud9 (Not Necessary)
```
export ROBOT_NAME=joystick1
export MOTOR_CONTROLLER=qwiic
colcon build && source install/setup.bash && roslaunch jetbot_app teleop.launch
colcon build && source install/setup.bash && roslaunch jetbot_sim_app teleop.launch
```
