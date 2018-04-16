# Installation Guide by source code


## Prerequisites

- OS: Ubuntu 16.04
- Java 8 JDK: openjdk-8
- Prolog: swi-prolog
- ROS:
	- ROS kinetic
	- rosjava
	- knowrob


## Java Installation

### JAVA 7 Installation

Download and uncompress:
```bash
# Download tar: 
# jdk-7u80-linux-i586.tar.gz

# uncompress:
cd <root>
mkdir -p local
tar zxf jdk-7u80-linux-i586.tar.gz -C local/
```

Modify the -rc file:
```bash
LOCAL_ROOT=/home/mpavezb/workspaces/memoria/software/local
export JAVA_HOME=$LOCAL_ROOT/jdk1.7.0_80
export PATH="$JAVA_HOME/bin:$PATH"
```

### JAVA 8 Installation

Download and uncompress:
```bash
# Download tar: 
# jdk-8u161-linux-i586.tar.gz

# uncompress:
cd <root>
mkdir -p local
tar zxf jdk-8u161-linux-i586.tar.gz -C local/
```

Modify the -rc file:
```bash
LOCAL_ROOT=/home/mpavezb/workspaces/memoria/software/local
export JAVA_HOME=$LOCAL_ROOT/jdk1.8.0_161
export PATH="$JAVA_HOME/bin:$PATH"
```


### Some problems with older versions of knowrob

Consideraciones:
- Utilizar Oracle JAVA, no basta con OpenJDK.
- Utilizar versión 7 SDK: Problemas de dependencias en v8 y v9. Missing security provider SunEC.
- Utilizar versión 32bits. Para poder ejecutarlo en pepper.
- Instalación local, para evitar conflictos de versiones y utilizar mismo procedimiento en pepper.


## Prolog

### JPL (link between java and prolog)

Download from: https://github.com/SWI-Prolog/packages-jpl/releases

```bash
cd <root>
mkdir -p local
tar zxf packages-jpl-7.7.10.tar.gz -C local/

```

### SWI-PROLOG

- http://www.swi-prolog.org/
- Versión: 7.6.4
- Build yourself or install it from package manager. More info on: http://www.swi-prolog.org/build/Debian.html
- Tested with Version 6.6.4, 64bits

#### Installation

```bash
# Download sources
# swipl-7.6.4.tar.gz

# uncompress
cd <root>
mkdir -p local
tar zxf swipl-7.6.4.tar.gz -C local/

# build depends
# see also: http://www.swi-prolog.org/build/Debian.html
# it also requires java 8 JDK
sudo apt-get install \
        build-essential autoconf curl chrpath pkg-config \
        ncurses-dev libreadline-dev libedit-dev \
        libunwind-dev \
        libgmp-dev \
        libssl-dev \
        unixodbc-dev \
        zlib1g-dev libarchive-dev \
        libossp-uuid-dev \
        libxext-dev libice-dev libjpeg-dev libxinerama-dev libxft-dev \
        libxpm-dev libxt-dev \
        libdb-dev \
        libpcre3-dev \
        junit

# docs dependencies
sudo apt-get install \
        texlive-latex-extra \
        texlive-fonts-extra \
        texlive-fonts-extra-doc \
        texlive-fonts-recommended \
        texlive-fonts-recommended-doc

# prepare build
# see also: http://www.swi-prolog.org/build/unix.html
cd local/swipl-7.6.4
cp -p build.templ build
mkdir -p /home/mpavezb/workspaces/memoria/software/local/swipl

# edit build file
# PREFIX=/home/mpavezb/workspaces/memoria/software/local/swipl

# build
./build

# Modify -rc file
export SWI_HOME_DIR=/usr/lib/swi-prolog
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-1.8.0-openjdk-amd64/jre/lib/amd64/server
```



### Rosjava

- requires java 8 JDK in ubuntu 14

#### INDIGO
```bash
cd /home/mpavezb/workspaces/memoria/code
mkdir -p deps_ws/src
cd deps_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/rosjava/rosjava/indigo/rosjava.rosinstall
wstool update

source /opt/ros/indigo/setup.zsh
rosdep update
rosdep install --ignore-src --from-paths .

sudo chmod 666 /opt/ros/indigo/share/maven/rospack_nosubdirs
cd ..
catkin_make
cd src
```

#### KINETIC 

##### Instalado desde source en kinetic, pero tiene algunos problemas

##### instalacion desde deb

```
sudo apt-get install ros-kinetic-rosjava
```





```bash
	# java
	#LOCAL_ROOT=/home/mpavezb/workspaces/memoria/software/local
	#export JAVA_HOME=$LOCAL_ROOT/jdk1.8.0_161
	#export PATH="$JAVA_HOME/bin:$PATH"
	#export LD_LIBRARY_PATH=$JAVA_HOME/jre/lib/amd64:$JAVA_HOME/jre/lib/amd64/server:$LD_LIBRARY_PATH

	# prolog
	#export SWI_HOME_DIR=/usr/lib/swi-prolog
	#export SWI_HOME_DIR=$LOCAL_ROOT/swipl/lib/swipl-7.6.4
	#export PATH="$LOCAL_ROOT/swipl/bin:$PATH"

```