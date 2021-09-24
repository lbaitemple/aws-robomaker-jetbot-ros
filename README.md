<p align="center">
  <h1 align="center">JetBot ROS2 Application with AWS RoboMaker Support</h1>
</p>

ROS2 version of [JetBot ROS Application](https://github.com/jerwallace/aws-robomaker-jetbot-ros) for arm64/aarch64 machine.

### Colaborators
[Computer Fusion Laboratory (CFL) - Temple University College of Engineering](https://sites.temple.edu/cflab/people/)
* [Animesh Bala Ani](https://www.linkedin.com/in/ani717/)
* [Dr. Li Bai](https://engineering.temple.edu/about/faculty-staff/li-bai-lbai)

# Initialization
### Download Workspace
```
git clone -b ros2 https://github.com/ANI717/aws-robomaker-jetbot-ros
mv aws-robomaker-jetbot-ros jetbot
```

### Set IoT Endpoints
```
cd ~/environment/jetbot/assets/scripts
chmod +x *.sh
./set_iot.sh
```

### Setup Credentials for Joystick Control
Copy AWS Credentials to `~/environment/jetbot/assets/credentials/credentials` file and run following commands. Then download the `~/environment/jetbot/assets/teleop` folder for controlling jetbot with virtual joystick.
```
cd ~/environment/jetbot/assets/scripts
./set_credentials.sh
```

### Create Docker Container for Cross Compilation
```
cd ~/environment/jetbot/assets/scripts
sudo ./set_docker.sh
```

# Robot and Simulation
### Robot URDF file
Please modify `jetbot_description_launch.py` and `spawn_launch.py` from **jetbot_description** package if you want to use different URDF file.

### Build, Source and Run Simulation Workspace
```
cd  ~/environment/jetbot/simulation_ws
sudo apt-get update
rosws update
rosdep update
rosdep  install --from-paths src --ignore-src -r -y
```
```
export DISPLAY=:0
colcon build && source install/setup.bash && ros2 launch jetbot_sim_app circle_launch.py
colcon build && source install/setup.bash && ros2 launch jetbot_sim_app teleop_launch.py
```

### Build, Bundle and Upload Robot Workspace
```
cd ~/environment/jetbot
sudo docker run --rm -ti -v $(pwd):/environment/jetbot  ros2-cross-compile:arm64
```
```
cd /environment/jetbot/robot_ws/
apt update && rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base 'arm64/build' --install-base 'arm64/install'
colcon bundle --build-base 'arm64/build' --install-base 'arm64/install' --bundle-base 'arm64/bundle'
exit
```
```
aws s3 cp ./robot_ws/arm64/bundle/output.tar s3://<<s3 bucket name>>/jetbot-aarch64.tar
```
Run following commands to test Robot Application while cross-compilation.
```
colcon build --build-base 'arm64/build' --install-base 'arm64/install' && \
source arm64/install/setup.bash && \
ros2 launch jetbot_app circle_launch.py
```
```
colcon build --build-base 'arm64/build' --install-base 'arm64/install' && \
source arm64/install/setup.bash && \
ros2 launch jetbot_app teleop_launch.py
```

# Deployment
Deployment Package is `jetbot_app` and launch files are `circle_launch.py` and `teleop_launch.py`.
### Install AWS Greengrass V1 in Jetson Nano
Log into Jetson Nano and install Greengrass core software with following commands.
```
sudo adduser --system ggc_user
sudo addgroup --system ggc_group
wget https://d1onfpft10uf5o.cloudfront.net/greengrass-core/downloads/1.9.1/greengrass-linux-aarch64-1.9.1.tar.gz
sudo tar -xzvf greengrass-linux-aarch64-1.9.1.tar.gz -C /
```
Download AWS ATS endpoint root certificate (CA)
```
cd /greengrass/certs/
sudo wget -O root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem
```

### Run Greengrass
Upload Robot Cirtificate in Jetson nano and install it with following commands.
```
sudo unzip <<robot_cert>>.zip -d /greengrass/
```
Start Greengrass Core.
```
sudo /greengrass/ggc/core/greengrassd start
```

### Check Launch Log
Open `launch.log` file from `/home/ggc_user/.ros/log/<<hash number>>/` directory


# Jetbot Setup
Follow instructions from [here](https://github.com/ANI717/headless_jetsonnano_setup) and [here](https://github.com/lbaitemple/jetbot_nividia_nano) to setup a Jetson Nano in headless mode.
Follow instructions from [ROS2 Dashing Official Site](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html) to install ROS2 in Jetson Nano.
