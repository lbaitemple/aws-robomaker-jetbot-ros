#Add ROS dependencies
cp -a deps/* /etc/ros/rosdep/sources.list.d/ 
echo "yaml file:///$WORK_DIR/jetbot.yaml" > /etc/ros/rosdep/sources.list.d/21-customdepenencies.list

# The following command logs in to the an AWS Elastic Container Repository (ECR) to
# enable your machine to pull a base docker image

$(aws ecr get-login --no-include-email --registry-ids 593875212637 --region us-east-1)

#Install Ubuntu dependencies for cross compilation:
apt update &&  apt install -y qemu-user-static

#Build Docker Container
docker build -t jetbot-ros -f Dockerfile .

#fix ros permissions
rosdep fix-permissions
sudo -u ubuntu rosdep update
