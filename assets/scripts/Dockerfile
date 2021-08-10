# Animesh Ani (ANI717)
# DOckerfile to create ARM64 ROS 2 Dashing Docker Image

FROM arm64v8/ros:dashing-ros-base-bionic

ENV PATH="/root/.local/bin:${PATH}"

# Copy qemu from the host machine
COPY qemu-aarch64-static /usr/bin
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install keys for colcon bundle
RUN apt-get update && apt-get install -y wget
RUN wget http://repo.ros2.org/repos.key -O - | sudo apt-key add -
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Install Python and colcon and other packages
RUN apt-get update && apt-get install -y \
    python3-apt \
    python3-pip \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install -U colcon-ros-bundle

# Install dependencies
RUN echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/18-aws-python.list \
    && rosdep update
