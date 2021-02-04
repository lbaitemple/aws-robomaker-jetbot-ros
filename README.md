## Building a Robot Application with RoboMaker

### Files below need to be updated through either CFN/script or manually
**Prerequisites - must have the following information before continue the lab - 
IAM role arn, IoT endpoint, Public VPC Subnet IDs (2), security group, S3 bucket, region, accessKeyId, secretAccessKey**

1. roboMakerSettings.json (IAM role arn, IoT endpoint, Public VPC Subnet IDs (2), security group, S3 bucket)
1. assets/teleop/awscreds.js (IoT endpoint, region, accessKeyId, secretAccessKey)


### Clone the Robot Application

1. Open the RoboMaker IDE and navigate to the terminal and clone this Git repo to the development machine:
    ```
    # change to the environment directory
    cd ~/environment
    git clone -b raspberrypiv2 https://github.com/lbaitemple/aws-robomaker-jetbot-ros
    mv aws-robomaker-jetbot-ros rpi
    ```


### Install Dependencies [~15 mins]
1. Open the RoboMaker IDE and navigate to the terminal.

1. In the terminal window, change to the `jetbot/assets` directory 
    ```
    # Run install_dep.sh to install prerequisite
    cd ~/environment/rpi/assets/scripts/arm32
    
    chmod +x *.sh
    
    sudo ./install_deps.sh
    ```
    Use the following command to checek if ros-cross-compile:armhf instance is installed
    ```
    sudo docker image list
    ```
    ![Alt text](img/docker32.png?raw=true "docker image")
    
    
    If not, you will need to remove all image by using
    ```
    sudo docker rm $(sudo docker ps -aq) 
    sudo docker rmi -f $(sudo docker images -q)
    ```
    After that, you back to the bedginning of the process to run.
    ```
    sudo ./reset_image.sh
    ```
    
1. Wait for previous step to complete and in the same terminal window, run the following command to update ROS dependencies 
    ```
    #  Make sure previous step is complete first
    rosdep update
    
    ```

### Run in Simulation and Explore [~30 mins]

### install

```
cd  ~/environment/rpi/simulation_ws
rosws update
rosdep  install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
```

#### To find IoT endpoint, you can use <no need>
### ```
### aws iot describe-endpoint
###```
1. Open the roboMakerSetting.json file in the **jetbot** directory and input S3 bucket, IAM role, MQTT endpoint and VPC public subnet ids and security group ids.

1. Click Run, Add or Edit Configurations, select the roboMakerSettings.json file from jetbot directory

1. Click Run, Workflow, select JetBot Simulation - Build and Bundle  (this process takes about 10 minutes)

1. Click Run, Launch Simulation, JetBot Circle Simulation - This will launch the application in simulation enviornment with the Jetbot rotating in circles. When simulation status displays (running), explore the enviornment in Gazebo.

1. Stop the simulation

1. Click Run, Launch Simulation, JetBot Teleop Simulation - This will launch the application in simulation enviornment where you can drive the Jetbot with the teleop client app. When simulation status displays (running), explore the enviornment in Gazebo.

1. Zip the teleop client app
    ```
    # Make sure you are in the jetbot directory
    $ cd ~/environment/rpi
    $ zip teleop.zip assets/teleop/*
    ```
1. Download the zip file in the file explorer and unzip it on the desktop
1. Open the robogui.html file in a browser and make sure the connection status states Connected
1. Use your mouse to move the joy stick and watch the Jetbot move in the Gazebo window
1. Stop the simulation

### Build, Bundle and Deploy Robot Application in ARMHF Architecture [~15 mins]
1. Open the RoboMaker IDE and navigate to the terminal

1. need to reset the **jetbot** credential if you have not used green grass for sometimee
```
cd ~/environment/rpi/assets/scripts
sudo ./reset_cred.sh
```

1. Change to the **jetbot** directory and build & bundle the ROS application in a docker container
    ```
     cd ~/environment/rpi/robot_ws/src/jetbot_app/nodes
     chmod +x *
     
    # Make sure you are in the jetbot directory
    $ cd ~/environment/rpi
    
    # IMPORTANT: Make sure you are in the jetbot directory
    # Build and bundle the robot application
    sudo docker run --rm -ti -v $(pwd):/environment/rpi ros-cross-compile:armhf

    # You will be dropped into the shell of the docker container
    # the prompt will be similar to the following root@83afb0b35322:/environment/jetbot# 
    (docker)$ cd /environment/rpi
    (docker)$ ./assets/scripts/compile_armhf.sh 

    # Wait until shell script is completed 
    #Type exit or Ctrl-D, to exit the container
    (docker)$ exit
    ```

1. Back in the RoboMaker IDE and navigate to a new terminal
    ```
    # Make sure you exited out of the container in previous step
    # Copy the robot application to S3
    cd ~/environment/rpi/
    aws s3 cp ./robot_ws/armhf_bundle/output.tar s3://<S3-BUCKET-NAME>/rpi/armhf/output.tar
    ```

## Deploying with RoboMaker
When a robot application is deployed to a physical robot, AWS RoboMaker does the following:

- AWS RoboMaker creates or updates a custom Lambda in your account. The Lambda contains the logic needed for deployment. This includes robot application bundle download, ROS launch, pre- and post-checks, and other logic.

- AWS RoboMaker begins deployment to the fleet using the parallelization specified in the deployment configuration.

- AWS RoboMaker notifies AWS IoT Greengrass to run the custom Lambda on the target robot. The daemon running on the robot receives the command and runs the Lambda. If a Lambda is running when the command is received, it and all ROS process on the robot are terminated.

- The Lambda downloads and uncompresses the robot application bundle from Amazon S3. If a pre-launch script was specified, it is run. Then the Lambda starts ROS using the launch file and package specified. If a post-launch script was specified, it is run after ROS is started. Finally, the status of the deployment is updated.

### Create a Robot Application
1. Open the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left pane, choose Development, and then choose Robot applications.

1. Select **Create robot application**.

1. In the Create robot application page, type a Name for the robot application. Choose a name that helps you identify the robot.

1. Select the Robot software suite used by your robot application
    * Select *ROS Melodic*

1. Provide the Amazon S3 path to your bundled robot application file in **ARM64 source file** field. If this robot application is used only in simulations, specify a bundle built for the ARM64 platform. If you use this robot application in a fleet deployment, specify one or more bundles that represent the architectures of the robots in your fleet.

1. Choose Create.


#### Create a Robot Application Version
1. Open the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose Development, and then choose Robot applications.

1. Choose the robot application name.

1. In the Robot applications details page, choose Create new version, and then choose Create.

### Create a Robot

To create a robot:

1. Open the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose Fleet Management, and then choose Robots.

1. Choose Create robot.

1. In the Create robot page, type a name for the robot.

1. Select the Architecture of the robot.
  1. Select the ARM64 architecture for the Sparkfun Robot

1. Under AWS IoT Greengrass group defaults, select a Create new to create a new AWS IoT Greengrass group for the robot. 
    *Optionally, you can select an existing AWS IoT Greengrass group. Each robot must have its own AWS IoT Greengrass group.*

    1. If you use an existing AWS IoT Greengrass group, it must have an IAM role associated with it. To create the role, see Create deployment role.

1. Select a IAM role to assign to the AWS IoT Greengrass group created for the robot. It grants permissions for AWS IoT Greengrass to access your robot application in Amazon S3 and read update status from AWS RoboMaker.

1. Choose Create.

1. In the **Download your Core device** page, choose **Download** to download and store your robot's security resources.


### Configure Robot with Certificates
AWS RoboMaker uses X.509 certificates, managed subscriptions, AWS IoT policies, and IAM policies & roles to secure the applications that run on robots in your deployment environment.

An AWS RoboMake robot is also a Greengrass core. Core devices use certificates and policies to securely connect to AWS IoT. The certificates and policies also allow AWS IoT Greengrass to deploy configuration information, Lambda functions, connectors, and managed subscriptions to core devices

1. On your local machine, open a terminal and navigate to the location of the dowloaded security resources from the previous step.

1. Locate the IP address of robot on the OLED
![Sparkfun Jetbot OLED display](https://cdn.shopify.com/s/files/1/0915/1182/products/14532-SparkFun_Micro_OLED_Breakout__Qwiic_-01_300x.jpg)

1. Unzip your device certificates to the robot (raspberry pi):

    ```
    # Copy the local security resources to the robot (or use jupter lab)
    (from PC)$ scp /path/to/downladed-zip/<robot-certs>.zip jetbot@<ip-addres>:/home/jetbot/robomaker-robot-certs.zip

    # Switch to the root user
    (pi)$ cd ~

    # Unzip the jetbot security credentials to greengrass certificate store
    (pi)$ sudo unzip /home/pi/<greengrass-certs>.zip -d /greengrass
    ```
    Type 'A' when prompted.
    
    ```
     # start greengrass core
    (pi)$ sudo /greengrass/ggc/core/greengrassd start
    
     ```

### Create a Fleet
1. Sign in to the AWS RoboMaker

1. In the left navigation pane, under **Fleet Management**, and then choose **Fleets**.

1. Select Create fleet.

    - In the Create fleet page, type a name for the fleet.


1. Click Create to create the fleet.


#### Register a Robot

1. In the left navigation pane, choose Fleet Management, and then choose Fleets.

1. Select the Name of the fleet you want to modify.

1. In the Fleet details page, select Register.

1. In the Register robots page, select the robot you want to register, then select Register robots.


### Create a Deployment
1. Sign in to the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose Fleet Management, and then choose Deployments.

1. Click Create deployment.

1. In the Create deployment page, under Configuration, select a Fleet.

1. Select a Robot application.

1. Select the Robot application version to deploy. The robot application must have a numbered `applicationVersion` for consistency reasons. If there are no versions listed, or to create a new version, see Creating a Robot Application Version.

1. Under Deployment launch config, specify the Package name: `jetbot_app`

1. Specify the Launch file: `teleop.launch`
  

1. Environment variables, type in an environment Name and Value. Environment variable names must start with A-Z or underscore and consist of A-Z, 0-9 and underscore. Names beginning with “AWS” are reserved.

    - Add the following environment variables:
        - **Key** = `IOT_ENDPOINT` (key must be in all caps exactly) Value = <your IoT endpointAddress> (this is the IOT_ENDPOINT you captured from earlier step in roboMakerSettings.json file)
        - **Key** = `ROBOT_NAME`(key must be in all caps exactly) Value = `joystick1`(do not change the value)
        - **Key** = `MOTOR_CONTROLLER`(key must be in all caps exactly as MOTOR_CONTROLLER) **Value** = `qwiic`
 

1. Specify a Robot deployment timeout. Deployment to an individual robot will stop if it does not complete before the amount of time specified.

1. Click Create to create the deployment job.

On Robot terminal:
```
(pi)$ sudo usermod -G i2c ggc_user
(pi)$ sudo chmod 0666 /dev/i2c-1
```
# Re(pi)$ run the ros
```
(pi)$ cd /greengrass/ggc/core/
(pi)$ sudo ./greengrassd start
```

then wait for 1 minute or 2 minutes, you can do
```
(pi)$ rostopic list
```

------

check file logs at
```
(pi)$ more /greengrass/ggc/var/log/system/runtime.log
(pi)$ more /home/ggc_user/roboMakerDeploymentPackage/log/rosout.log
```
or
```
(pi)$ cd /home/ggc_user/roboMakerDeploymentPackage/log
(pi)$ cd /home/ggc_user/ros/home/deployment-xxxxx/log/
```


Keep track of the progress of the deployment, when copying and extracting completes, the status will change to **Launching**.  

**Congratulations, you can now control your robot with the virtual joystick!**
