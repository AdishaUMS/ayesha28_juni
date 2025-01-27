# **adisha-os**
A ROS2-based framework for Adisha UMS robot development.


## **1. Installation**
We use Docker to develop the robot since the program maybe deployed in various machines. Note that we can directly ```colcon build``` this repository if we are working on a machine that matches the requirements for ROS2 Humble. Or else, do the following steps to begin working with this repository:

### **1.1 Initialization**
This process is used for 2 things: creating a unique ID or namespace for our machine (robot), and getting the local network IP, if one exists. This helps a lot when we work on multiple robots on the same local network. In the container's bash terminal, run the following:

```console
# On host machine
python3 adisha_setup.py
```

We will be asked for an ID/namespace to be given to the machine.

### **1.2 Build the Images** 
Run the following commands to build the Docker image required:

```console
# On host machine
sudo docker build -t adisha-os .
``` 

### **1.3 Start the Containers**
Run the following command:

```console
# On host machine
sudo docker compose up -d
```


## **2. Basic Usage**

### **2.1 Developing and Building**
Please note that the container ```adisha-os``` volume-bind the ```src/``` and ```web/``` directory. Therefore, we might develop the source codes on our local or host machine and build the project on the Docker container. Use the following command to access the Docker container interactively on our terminal:

```console
# On host machine
sudo docker exec -it adisha-os bash
```

#### **a. For Building the ROS Project**
In the container's bash terminal, run the following:

```console
# On Docker container
colcon build
```

Of course, we can do the other things inside the container's bash terminal if needed. 

#### **b. For Building Web**
It is more convenient to develop and build the web on our own machine. But since the ```rclnodejs``` requires ROS2 environment to be sourced, make sure we have ROS2 installed in our machine. Run the following command everytime the web project is reinstalled:

```console
# On host machine
colcon build --packages-select adisha_data adisha_interfaces
source install/setup.bash
cd web/
npm install
npx generate-ros-messages
```

> [!IMPORTANT]
>   Use the same node version (20.11.1).
To ease node version managing, install nvm with:
> ```curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash```.
> Next, install the required node version with:
> ```nvm install 20.11.1```.
> Or, if the version has been installed:
> ```nvm use 20.11.1```.

#### **c. Dynamixel SDK Compatibility**
Adisha team use 3 different types of Dynamixel servo: XL320 (2.0), AX12A (1.0), and MX28 (1.0). The R/W method compatibility are shown below.

| DXL Type | Sync R | Sync W | Bulk R | Bulk W |
|:-:|:-:|:-:|:-:|:-:|
|XL320 (2.0)| NO | **OK** | NO | NO |
|AX12A (1.0)| NO | **OK** | NO | NO |
|MX28 (1.0)| NO | **OK** | **OK** | NO |

### **2.2 Launches**
Remember to source the workspace's bash file everytime we open a new terminal with:

```console
# On Docker container
source install/setup.bash
```

All launch files are contained inside ```adisha_main``` package. Go to ```src/adisha_main/launch/``` to see all the available launch files. Proceed with the following comment to run the launch file:

```console
# On Docker container
ros2 launch adisha_main <launch_file_name.py>
```# 19juni
# 19juni
# ayesha28_juni
# ayesha28_juni
