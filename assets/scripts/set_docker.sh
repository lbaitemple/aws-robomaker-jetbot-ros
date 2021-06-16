WORK_DIR=$(pwd)

# Add ROS dependencies
cp -a resources/deps/* /etc/ros/rosdep/sources.list.d/
echo "yaml file:///$WORK_DIR/resources/jetbot.yaml" > /etc/ros/rosdep/sources.list.d/21-customdepenencies.list

# The following command logs in to the an AWS Elastic Container Repository (ECR) to
# enable your machine to pull a base docker image
$(aws ecr get-login --no-include-email --registry-ids 593875212637 --region us-east-1)
apt purge qemu-user qemu-user-static -y

# Install Ubuntu dependencies for cross compilation:
apt update &&  apt install -y qemu-user qemu-user-static
cp /usr/bin/qemu-aarch64-static .

# Build Docker Container
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes # This step will execute the registering scripts
docker build -t ros-cross-compile:arm64 .

# Fix ros permissions
rosdep fix-permissions
sudo -u ubuntu rosdep update

# Remove leftovers
rm qemu-aarch64-static