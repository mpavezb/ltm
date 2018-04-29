# Tutorials: The ltm_samples package

In order to keep the codebase clean, all testing code, samples and scripts (and testing dependencies) have been moved to an standalone repository, the [ltm_samples](https://github.com/mpavezb/ltm_samples) ROS package.

That package serves as a guide for new users to develop custom LTM plugins, designed for their robotic system needs.


## Requirements

```bash
# install ROS smach packages.
sudo apt-get install ros-kinetic-smach ros-kinetic-smach-viewer

# install ROS video to rostopic suite.
sudo apt-get install ros-kinetic-video-stream-opencv ros-kinetic-image-view

# python-faker
pip install faker

# download the ltm_samples package.
roscd && cd ../src
git clone https://github.com/mpavezb/ltm_samples.git

# compile.
roscd && cd ..
catkin_make
```


## Download required files

```bash
# download video samples for image streaming.
rosrun ltm_samples download_videos.bash

# download image samples for the episode generator.
rosrun ltm_samples download_images.bash

# Note: As the download process can fail sometimes, repeat 
# each command until the output only contains green characters.
```


## Testing

### Launching the LTM server

```bash
# launch LTM server.
roslaunch ltm ltm.launch --screen

# (use this to display debug information)
rosconsole set /robot/ltm ros.ltm debug

# Show the ROS API:
rostopic list | grep ltm/
rosservice list | grep ltm/
rosparam list | grep ltm/
```

### Running the LTM server and adding some episodes from JSON files

```bash
# drop current database.
rosservice call /robot/ltm/drop_db "{}"

# Load a single episode from a JSON file.
roslaunch ltm_samples ltm_load_file.launch

# Load a set of episodes from JSON files in a folder.
roslaunch ltm_samples ltm_load_folder.launch
```

### Deploy a fake robot to generate dummy episode information

```bash
# this will deploy a fake robot which can generate dummy emotional and
# location information required for LTM episodes.
roslaunch ltm_samples fake_robot.launch --screen

# deploy a fake camera based on the downloaded videos.
# the second command will show an image_view window of the video. 
roslaunch ltm_samples video_streamer.launch --screen
roslaunch ltm_samples video_streamer.launch view:=true --screen

# Show the ROS API:
# Both launchfiles use the '/robot/fake/' ROS namespace.
rostopic list | grep fake/
rosservice list | grep fake/
rosparam list | grep fake/

```

### Run a smach state machine with dummy states

```bash
# this machine resembles the SPR test of robocup@home competition.
rosrun ltm_samples spr.py

# launch the smach viewer
# the second command will show the smach_viewer.py node displaying the machine states.
roslaunch ltm_samples smach_spr.launch --screen
roslaunch ltm_samples smach_spr.launch viewer:=true --screen
```
