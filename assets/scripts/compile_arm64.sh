echo "yaml file:///environment/jetbot/assets/scripts/jetbot.yaml" > /etc/ros/rosdep/sources.list.d/21-customdepenencies.list
apt install  libqt4-dev  -y
apt update
rosdep fix-permissions && rosdep update 
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base arm64_build --install-base arm64_install --cmake-args "-DPYTHON_EXECUTABLE=/usr/bin/python3" "-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m" "-DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so"
colcon bundle --build-base arm64_build --install-base arm64_install --bundle-base arm64_bundle --apt-sources-list /opt/cross/apt-sources.yaml
