# omniroute_operation_ws

```
cd esmacat_master_software

mkdir build
cd build
cmake ..
make

cd ../..

cd plog

mkdir build
cd build
cmake ..
make

cd ../../..

catkin_make
```
For ethercat_grant, you will likely need to install library libcap-dev
```
sudo apt install libcap-dev
```
After running catkin_make, copy executable into /usr/local/bin
```
sudo cp devel/lib/ethercat_grant/ethercat_grant /usr/local/bin
sudo chmod +s /usr/local/bin/ethercat_grant
```


