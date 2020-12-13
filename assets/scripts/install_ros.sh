WORK_DIR=$(pwd)
#Add ROS dependencies
cp -a deps/* /etc/ros/rosdep/sources.list.d/ 
echo "yaml file:///$WORK_DIR/jetbot.yaml" > /etc/ros/rosdep/sources.list.d/21-customdepenencies.list

# The following command logs in to the an AWS Elastic Container Repository (ECR) to
# enable your machine to pull a base docker image


#Install Ubuntu dependencies for cross compilation:
apt update &&  apt install -y qemu-user-static

rosdep fix-permissions
sudo -u $USER rosdep update
