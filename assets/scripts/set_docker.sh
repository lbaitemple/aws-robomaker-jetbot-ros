# Animesh Ani (ANI717)
# This Script creates ROS 2 Dashing Docker Image in ARM64 Architecture for Cross Compilation.

# Remove previously created dockers with same name
docker ps -a | awk '{ print $1,$2 }' | grep ros2-cross-compile:arm64 | awk '{print $1 }' | xargs -I {} docker stop {}
docker ps -a | awk '{ print $1,$2 }' | grep ros2-cross-compile:arm64 | awk '{print $1 }' | xargs -I {} docker rm {}
sudo docker rmi ros2-cross-compile:arm64

# Install Ubuntu dependencies for cross compilation
apt install qemu-user-static -y
cp /usr/bin/qemu-aarch64-static .

# Build Docker Container
docker build -t ros2-cross-compile:arm64 .

# Remove extra files
rm ./qemu-aarch64-static
