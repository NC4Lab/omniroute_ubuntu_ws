# start in omniroute_operation_ws
# Need to run this with each local repo clone

```
cd src/esmacat_master_software

mkdir build
cd build
cmake ..
make

cd ../plog

mkdir build
cd build
cmake ..
make

cd ../../..

catkin_make
```

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


# Setup ethernet
```
change ethenet address for ethercat sheild in launch file depending on what computer you are using
```
# TO DO: Add instructions for port permisions for writing sketches to Arduino

# Run catkin_make after changing things
```
cd omniroute_operation_ws

catkin_make
```

# Launching the ROS system test
```
roslaunch omniroute_operation test.launch
```
```