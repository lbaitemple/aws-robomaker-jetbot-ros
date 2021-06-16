mv JetBot jetbot


cd ~/environment/jetbot/assets/scripts
chmod +x *.sh
sudo ./set_iot.sh
sudo ./set_docker.sh


rosdep update
sudo apt install -y cmake gcc g++ qt{4,5}-qmake libqt4-dev
sudo apt -y autoremove


cd  ~/environment/jetbot/robot_ws
rosdep  install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
aws s3 cp ./bundle/output.tar s3://ani717/jetbot_robot_X86_64.tar


cd  ~/environment/jetbot/simulation_ws
vcs import < .rosinstall
rosdep  install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
aws s3 cp ./bundle/output.tar s3://ani717/jetbot_simulation.tar


cd ~/environment/jetbot/assets/scripts
./set_credentials.sh



export ROBOT_NAME=joystick1
export MOTOR_CONTROLLER=qwiic
colcon build && source install/setup.bash && roslaunch jetbot_app teleop.launch
colcon build && source install/setup.bash && roslaunch jetbot_sim_app teleop.launch