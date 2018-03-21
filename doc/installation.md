# Installation Guide


## Prerequisites

- OS: Ubuntu 16.04
- Java 8 JDK: openjdk-8
- Prolog: swi-prolog
- ROS:
	- ROS kinetic
	- rosjava
	- knowrob



### Installation

1st install ROS kinetic, then proceed with the following:

```bash
# Java 8
sudo apt install openjdk-8-jdk

# swi-prolog
sudo apt install swi-prolog

# ros java
sudo apt install ros-kinetic-rosjava

# knowrob requirements
sudo apt install ros-kinetic-tf2-web-republisher

```

Configure the environment:
```bash
# on the .bashrc file add this:
export JAVA_HOME=/usr/lib/jvm/default-java
export SWI_HOME_DIR=/usr/lib/swi-prolog
export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH
```

Get knowrob:



### Tools

#### Protege Editor

- WebPage: https://protege.stanford.edu/

```bash
# protege
PROTEGE_ROOT=/home/mpavezb/workspaces/memoria/software/local/Protege-5.2.0
alias protege="cd $PROTEGE_ROOT && ./run.sh"
```
