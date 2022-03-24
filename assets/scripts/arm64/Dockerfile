ARG ROS_VERSION=melodic
ARG UBUNTU_VERSION=bionic
FROM arm64v8/ros:melodic-ros-base-bionic

ENV PATH="/root/.local/bin:${PATH}"
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
# Copy qemu from the host machine
COPY qemu-aarch64-static /usr/bin

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Installed for convenience
WORKDIR /environment/jetbot
RUN apt-get update && apt-get install -y vim

# Add raspicam_node sources to apt

RUN apt-get update && apt-get install -y apt-transport-https \
&& echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list 

## supress error
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654  >> /dev/null 2>&1 



# Install Python and colcon
RUN apt clean  && apt-get update && apt-get install -y \
python \
python3-apt python-pip python3-pip \
&& python3 -m pip install  MarkupSafe==1.1.1 \
&& python3 -m pip install -U colcon-ros-bundle

# Add custom rosdep rules
COPY custom-rosdep-rules/raspicam-node.yaml /etc/ros/rosdep/custom-rules/raspicam-node.yaml
#RUN echo "yaml file:/etc/ros/rosdep/custom-rules/raspicam-node.yaml" > /etc/ros/rosdep/sources.list.d/22-raspicam-node.list \
RUN echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/18-aws-python.list \
&& rosdep update

# Add custom pip rules
COPY custom-pip-rules.conf /etc/pip.conf

# Add custom apt sources for bundling
COPY bionic-sources-arm64.yaml /opt/cross/apt-sources.yaml

