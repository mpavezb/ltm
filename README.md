# LTM - Long Term Memory for Robots

Long Term Memory for robots using ROS. Thesis project.

## Overview

The episodic LTM (EpLTM) is a very relevant challenge for service robotics, it allows us to enhance human-robot interaction (HRI), robot understanding about the environment, and the way in which it can handle tasks. This repository holds the ROS implementation code for the LTM project developed by Matías Pavez in his final C.Sc and E.E Thesis. See also [the thesis repository](https://github.com/mpavezb/memoria).


This projects provides a LTM implementation with Episodic, Semantic and Emotional components. Episodic information relates to *What* happened, *When* it happened, and *Where*. The *What* relates to the Semantic Memory, which can contain any kind of information, while this is defined through ROS messages. Stored episodes count with historical and emotional relevances, to give better tools for episode retrieval queries. 


The robot location (*Where*), the Semantic Memory (*What*) and emotion information is collected through plugins defined by the user. So some plugins must be implemented following a *pluginlib* API.


Some plugins are already defined on the `ltm_addons` and `ltm_samples` ROS packages. The `ltm_addons`  package also provides a SMACH interface to the LTM server, which allows the collection of SMACH states as LTM episodes.

The project is built on ROS and MongoDB. ROS works as the robotics framework which enables the communication between robot components, while MongoDB stores all episodic and semantic information.


## Related ROS packages:

- [ltm_addons](https://github.com/mpavezb/ltm_addons)
- [ltm_samples](https://github.com/mpavezb/ltm_samples)


## Documentation

- [Thesis](https://github.com/mpavezb/memoria) (in spanish).
- [Installation guide](doc/installation.md)
- [Tutorials](doc/tutorials.md)
