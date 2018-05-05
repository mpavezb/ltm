#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import smach
import types
import rospy
from ltm.srv import *
from ltm.msg import *


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class LTMData(object):
    # TODO: use inheritance to represent Nodes and Leafs

    def __init__(self):
        self.when = When()
        self.uid = None
        self.type = None
        self.label = None

    def get_episode(self):
        episode = Episode()
        episode.uid = self.uid
        episode.type = self.type
        episode.when = self.when
        return episode


def node_start_cb(self, manager):
    manager.cb_node_start(self)


def node_end_cb(self, manager):
    manager.cb_node_end(self)


def leaf_start_cb(self, manager):
    manager.cb_leaf_start(self)


def leaf_end_cb(self, manager):
    manager.cb_leaf_end(self)


class Manager(object):
    __metaclass__ = Singleton
    # TODO: try/except for everything related to the manager, but raise machine exceptions. We cannot let the
    # machine die for a LTM related error.

    def __init__(self):
        # ROS clients
        rospy.loginfo("Building LTM/SMACH interface manager...")
        self.add_episode_client = None
        self.register_episode_client = None

    def initialize(self):
        self.add_episode_client = rospy.ServiceProxy('/robot/ltm/add_episode', AddEpisode)
        self.register_episode_client = rospy.ServiceProxy('/robot/ltm/register_episode', RegisterEpisode)

        # Wait for ROS services
        rospy.loginfo("... waiting LTM services.")
        # self.add_episode_client.wait_for_service()
        print self.register_episode_client.resolved_name
        self.register_episode_client.wait_for_service()
        rospy.loginfo("... LTM server is up and running.")

    def introspect(self, state, label="root"):
        if isinstance(state, smach.Container):
            # Node is a state machine.
            children = state.get_children()
            print " - node: " + label + " has (" + str(len(children)) + ") children"
            for child_label in children:
                self.introspect(children[child_label], child_label)
        else:
            # Node is a state (leaf).
            print " - leaf: " + label

    def setup(self, state):
        self.initialize()
        self.setup_tree(state, "root")

    def setup_callbacks(self, state, start_cb, end_cb):
        # assign callbacks to the state instance.
        fn_start = types.MethodType(start_cb, state)
        fn_end = types.MethodType(end_cb, state)

        # wrapper function for state.execute method.
        fn_execute = state.execute

        def ltm_execute(_self, ud=smach.UserData()):
            # Raise any caught exception, but always execute the callbacks.
            # The default UserData is required, as nodes won't always get a 'ud' arg.
            fn_start(self)
            try:
                outcome = fn_execute(ud)
            finally:
                fn_end(self)
            return outcome

        state.execute = types.MethodType(ltm_execute, state)

    def setup_tree(self, state, label):
        # While smach.State don't define start and termination callbacks,
        # smach.Container does, but it does not let us access the class variables.
        # So we redefine the 'execute' method with pre and post hooks.
        # Those hooks have access to this Manager instance (self) and all state data.

        # set default LTM data field
        state.ltm = LTMData()
        state.ltm.label = label

        if isinstance(state, smach.Container):
            # Node is a state machine.
            self.setup_callbacks(state, node_start_cb, node_end_cb)

            # recurse towards leafs
            children = state.get_children()
            for child_label in children:
                self.setup_tree(children[child_label], child_label)
        else:
            # Node is a state (leaf).
            self.setup_callbacks(state, leaf_start_cb, leaf_end_cb)

    def cb_leaf_start(self, state):
        # TODO: try/except for ROS stuff.
        state.ltm.when.start = rospy.Time.now()
        state.ltm.type = Episode.LEAF
        try:
            state.ltm.uid = self.register_episode_client()
        except rospy.ServiceException:
            # There aren't any available uids. DB is full.
            pass
        rospy.logerr("[LEAF][" + str(state.ltm.uid) + "][" + state.ltm.label + "] - START callback.")

    def cb_leaf_end(self, state):
        # TODO: try/except for ROS stuff.
        state.ltm.when.end = rospy.Time.now()
        episode = state.ltm.get_episode()
        rospy.logerr("[LEAF][" + str(state.ltm.uid) + "][" + state.ltm.label + "] - END callback.")

    def cb_node_start(self, state):
        rospy.logerr("[NODE][" + str(state.ltm.uid) + "][" + state.ltm.label + "] - START callback.")

    def cb_node_end(self, state):
        rospy.logerr("[NODE][" + str(state.ltm.uid) + "][" + state.ltm.label + "] - END callback.")
