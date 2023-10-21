# To Do: 

[1] Incomplete Add instructions for port permisions for writing sketches to Arduino

2. Add instructions for setting up ROS network environment variables

# Need to perform only once on a given OS
```
For ethercat_grant, you will likely need to install library libcap-dev

sudo apt install libcap-dev

After running catkin_make, copy executable into /usr/local/bin

sudo cp devel/lib/ethercat_grant/ethercat_grant /usr/local/bin
sudo chmod +s /usr/local/bin/ethercat_grant

Three packages may need to be installed:

sudo apt install libxmlrpcpp-dev liblog4cxx-dev librosconsole-dev
```

# Need to run this with each local repo clone

- start in omniroute_ubuntu_ws

```
cd src/esmacat_master_software

mkdir build
cd build
cmake ..
make

cd ../../plog

mkdir build
cd build
cmake ..
make

cd ../../..

source /opt/ros/noetic/setup.bash

catkin_init_workspace src

# Note you may need to comment out any catkin_make alias in the .basher
sudo nano ~/.bashrc 
source ~/.bashrc


catkin_make
```

# Change terminal starting directory

```
sudo nano ~/.bashrc
xdg-open ~/.bashrc
```

# Setup ethernet
```
change ethenet address for ethercat sheild in launch file:
src/omniroute_operation/launch/test.launch

```

# Run catkin_make after changing things
```
cd omniroute_ubuntu_ws

catkin_make
```

# Launching the ROS three_by_three_interface
```
cd omniroute_ubuntu_ws

source devel/setup.bash

roslaunch omniroute_operation three_by_three_interface.launch
```
```

rosrun rospy_tutorials talker.py 