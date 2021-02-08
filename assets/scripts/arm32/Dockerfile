ARG ROS_VERSION=melodic
ARG UBUNTU_VERSION=bionic
FROM arm32v7/ros:melodic-ros-base-bionic

ENV PATH="/root/.local/bin:${PATH}"
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
COPY qemu-arm-static /usr/bin
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

#RUN printf "yaml file:///home/ubuntu/environment/rpi/assets/scripts/jetbot.yaml" > /etc/ros/rosdep/sources.list.d/21-customdepenencies.list

WORKDIR /environment/rpi
RUN apt-get update && apt-get install -y vim

RUN apt-get update && apt-get install -y apt-transport-https \
&& echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list 

## supress error
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8  >> /dev/null 2>&1 

# Install Python and colcon
RUN apt-get update && apt-get install -y \
python \
python3-apt python-pip python3-pip \
&& python3 -m pip install -U colcon-ros-bundle

COPY custom-rosdep-rules/raspicam-node.yaml /etc/ros/rosdep/custom-rules/raspicam-node.yaml
#RUN echo "yaml file:/etc/ros/rosdep/custom-rules/raspicam-node.yaml" > /etc/ros/rosdep/sources.list.d/22-raspicam-node.list \
RUN echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/18-aws-python.list \
&& rosdep update

# Add custom pip rules
COPY custom-pip-rules.conf /etc/pip.conf

# Add custom apt sources for bundling
COPY bionic-sources-arm32.yaml /opt/cross/apt-sources.yaml
