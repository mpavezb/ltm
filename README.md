# LTM - Long Term Memory for Robots

Long Term Memory for robots using ROS.

## Overview

The episodic LTM (EpLTM) is a very relevant challenge for service robotics, it allows us to enhance human-robot interaction (HRI), robot understanding about the environment, and the way in which it can handle tasks. This repository holds the ROS implementation code for the LTM project developed by Matías Pavez in his [C.Sc and E.E Thesis](https://github.com/mpavezb/memoria).

This projects provides a LTM implementation with Episodic, Semantic and Emotional components. Episodic information relates to *What* happened, *When* it happened, and *Where*. The *What* relates to the Semantic Memory, which can contain any kind of information, while this is defined through ROS messages. Stored episodes count with historical and emotional relevances, to give better tools for episode retrieval queries. 

The robot location (*Where*), the Semantic Memory (*What*) and emotion information is collected through plugins defined by the user. So some plugins must be implemented following a *pluginlib* API.

Some plugins are already defined on the `ltm_addons` and `ltm_samples` ROS packages. The `ltm_addons` package also provides a SMACH interface to the LTM server, which allows the collection of SMACH states as LTM episodes.

The project is built over ROS and MongoDB. ROS works as the robotics framework which enables the communication between robot components, while MongoDB stores all episodic and semantic information.


## Documentation

- [Installation](doc/installation.md)
- [LTM for the Bender robot](https://github.com/uchile-robotics/bender_ltm)
- [Thesis](https://github.com/mpavezb/memoria) (in spanish)
- [Future Work](doc/proposed.md)


## LTM Suite - ROS packages:

- [ltm](https://github.com/mpavezb/ltm)
- [ltm_addons](https://github.com/mpavezb/ltm_addons)
- [ltm_samples](https://github.com/mpavezb/ltm_samples)
- [ltm_db](https://github.com/mpavezb/ltm_db)
