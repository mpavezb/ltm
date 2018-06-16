# Installation Guide


## Prerequisites

- OS: Ubuntu 16.04
- ROS:
	- ROS kinetic


### Install mongodb server

```bash
sudo apt-get install mongodb-server
```

### Install C++ drivers

```bash
# get mongodb C++ drivers
git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git

# compile it using scons
sudo apt-get install scons
cd mongo-cxx-driver
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
```

Now, you should now be able to compile the packages using catkin.


### Install mongo python library

See [the python API](https://api.mongodb.com/python/current/) for PyMongo.

```bash
sudo apt install python-pip
python -m pip install pymongo
```

### Installation: Warehouse ROS mongo

The deb install is not complete and does not provide the required plugin for mongo, so this has to be installed from source. See also the [official github repository](https://github.com/ros-planning/warehouse_ros_mongo).

```bash
# get warehouse_ros_mongo repo
git clone https://github.com/mpavezb/warehouse_ros_mongo.git

# dependency for warehouse_ros_mongo
sudo apt-get install ros-kinetic-warehouse-ros
```


#### Verify installation

```bash
roscd && cd ..

# set up server
catkin_make
rosrun warehouse_ros_mongo mongo_wrapper_ros.py
roslaunch warehouse_ros_mongo warehouse.launch --screen

# run tests
catkin_make tests
rosrun warehouse_ros_mongo test_warehouse_ros_mongo_cpp 
```

## Useful Commands

```bash
# look for process using a specific
sudo lsof -i | grep <port>
kill -9 <pid>

# kill mongodb server
kill -9 $(pidof mongod)
```

### Install LTM repository

```bash
# download
roscd && cd ../src
git clone https://github.com/mpavezb/ltm.git

# compile
roscd && cd ..
catkin_make
```
