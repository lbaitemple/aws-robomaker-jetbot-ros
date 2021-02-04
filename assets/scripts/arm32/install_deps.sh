WORK_DIR=$(pwd)
ROBOT_CERTS_FOLDER=$WORK_DIR/../../robot_ws/src/jetbot_app/config
[ ! -d "$ROBOT_CERTS_FOLDER" ] && mkdir -p $ROBOT_CERTS_FOLDER
SIM_CERTS_FOLDER=$WORK_DIR/../../simulation_ws/src/jetbot_sim_app/config
[ ! -d "$SIM_CERTS_FOLDER" ] && mkdir -p $SIM_CERTS_FOLDER
IOTPOLICY="file://../policies/iotpolicy.json"
IOTPOLICYNAME="JetBotPolicy"

#Configure IoT
#Create endpoint


#Create IoT Policy
aws iot create-policy \
--policy-name $IOTPOLICYNAME \
--policy-document $IOTPOLICY

#Create IoT Certificates
#Create two certs for robot_ws and simulation_ws
ROBOT_CERTARN=$(aws iot create-keys-and-certificate --set-as-active \
    --certificate-pem-outfile "$ROBOT_CERTS_FOLDER/certificate.pem.crt" \
    --private-key-outfile  "$ROBOT_CERTS_FOLDER/private.pem.key" \
    --public-key-outfile  "$ROBOT_CERTS_FOLDER/public.pem.key" \
    --query "[certificateArn]" \
    --output text
    )

wget -O $ROBOT_CERTS_FOLDER/root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

SIM_CERTARN=$(aws iot create-keys-and-certificate --set-as-active \
    --certificate-pem-outfile "$SIM_CERTS_FOLDER/certificate.pem.crt" \
    --private-key-outfile  "$SIM_CERTS_FOLDER/private.pem.key" \
    --public-key-outfile  "$SIM_CERTS_FOLDER/public.pem.key" \
    --query "[certificateArn]" \
    --output text
    )

wget -O $SIM_CERTS_FOLDER/root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

chmod 755 $SIM_CERTS_FOLDER/* 
chmod 755 $ROBOT_CERTS_FOLDER/*

#attach policy
aws iot attach-policy \
--policy-name $IOTPOLICYNAME \
--target $ROBOT_CERTARN

aws iot attach-policy \
--policy-name $IOTPOLICYNAME \
--target $SIM_CERTARN


#Add ROS dependencies
cp -a deps/* /etc/ros/rosdep/sources.list.d/ 
echo "yaml file:///$WORK_DIR/jetbot.yaml" > /etc/ros/rosdep/sources.list.d/21-customdepenencies.list

# The following command logs in to the an AWS Elastic Container Repository (ECR) to
# enable your machine to pull a base docker image

$(aws ecr get-login --no-include-email --registry-ids 593875212637 --region us-east-1)

#Install Ubuntu dependencies for cross compilation:
apt update &&  apt install -y qemu-user-static  apt-utils 

#Build Docker Container
#docker build -t jetbot-ros -f Dockerfile .
cd /opt/robomaker/cross-compilation-dockerfile/
sudo rm /opt/robomaker/cross-compilation-dockerfile/qemu/download/qemu*.tar.xz*
sudo bin/build_image.bash
#fix ros permissions
rosdep fix-permissions
sudo -u ubuntu rosdep update
