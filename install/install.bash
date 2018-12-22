#!/bin/bash
#
# Run me like this
# > bash install.sh
#

INSTALL_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"


# Prerequisites
# -------------------------------------------

# - MongoDB server
# - scons compiler tool
# - python pip
sudo apt-get install mongodb-server scons python-pip


# MongoDB C++ drivers
# -------------------------------------------

cd ${INSTALL_DIR}

# official driver (legacy version!)
if [[ ! -d mongo-cxx-driver ]]; then
	git clone https://github.com/mongodb/mongo-cxx-driver.git
fi
cd mongo-cxx-driver
git checkout 26compat

# compile it using scons
# sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors


# MongoDB Python driver
# -------------------------------------------
python -m pip install pymongo


# ltm_addons
# -------------------------------------------
sudo apt-get install ros-kinetic-smach ros-kinetic-smach-viewer ros-kinetic-video-stream-opencv ros-kinetic-image-view
pip install faker

