apt update
rosdep fix-permissions && rosdep update 
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install --cmake-args "-DPYTHON_EXECUTABLE=/usr/bin/python3" "-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m" "-DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython3.6m.so/libpython3.6m.so"
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base arm64_bundle --apt-sources-list /opt/cross/apt-sources.yaml
