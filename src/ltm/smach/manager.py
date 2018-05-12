#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import types
import smach
import rospy
from ltm.srv import *
from ltm.msg import *


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class LTMStateData(object):
    """
    This class serves as a data object for storing LTM information on smach states.
    """

    def __init__(self):
        # Registered data
        self.registered = False
        self.tags = set()

        # Tree introspection data
        self.label = ""
        self.parent = None  # kept None for root

        # Only registered nodes get a UID.
        # This value changes every time the state is executed.
        # 'uid' is set to None when state is not running
        self.uid = None

    def clear(self):
        self.uid = None

    def setup(self, label, parent):
        self.clear()
        self.label = label
        self.parent = parent

    def register(self, tags):
        self.clear()
        self.registered = True
        self.tags = set(tags)

    def is_registered(self):
        return self.registered

    def mark_unregistered(self):
        self.clear()
        self.registered = False
        self.tags = set()


class EpisodeData(object):
    # TODO: use inheritance to represent Nodes and Leafs

    def __init__(self):
        self.uid = None
        self.when = When()
        self.type = None
        self.parent_id = None
        self.children_ids = set()

    def get_episode(self):
        episode = Episode()
        episode.uid = self.uid
        episode.type = self.type
        episode.parent_id = self.parent_id
        episode.children_ids = list(self.children_ids)
        episode.info.source = "smach"
        episode.info.creation_date = rospy.Time.now()
        episode.when = self.when
        return episode


def cb_node_start(self, manager):
    manager.cb_node_start(self)


def cb_node_end(self, manager):
    manager.cb_node_end(self)


def cb_leaf_start(self, manager):
    manager.cb_leaf_start(self)


def cb_leaf_end(self, manager):
    manager.cb_leaf_end(self)


class Manager(object):
    __metaclass__ = Singleton
    # TODO: try/except for everything related to the manager, but raise machine exceptions. We cannot let the
    # machine die for a LTM related error.

    def __init__(self):
        # ROS clients
        rospy.loginfo("[LTM]: Building LTM/SMACH interface manager...")
        self.add_episode_client = None
        self.register_episode_client = None
        self.registered_episodes = dict()

    def initialize(self):
        self.add_episode_client = rospy.ServiceProxy('/robot/ltm/add_episode', AddEpisode)
        self.register_episode_client = rospy.ServiceProxy('/robot/ltm/register_episode', RegisterEpisode)

        # Wait for ROS services
        rospy.loginfo("[LTM]: ... waiting LTM services.")
        self.add_episode_client.wait_for_service()
        self.register_episode_client.wait_for_service()
        rospy.loginfo("[LTM]: ... LTM server is up and running.")

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

        # force root to be registered
        if not self.is_registered(state):
            self.register_state(state, ["_unregistered_root"])

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

    def setup_tree(self, state, label, parent_state=None):
        # While smach.State don't define start and termination callbacks,
        # smach.Container does, but it does not let us access the class variables.
        # So we redefine the 'execute' method with pre and post hooks.
        # Those hooks have access to this Manager instance (self) and all state data.

        # set default LTM data field
        if not self.is_registered(state):
            state.ltm = LTMStateData()
            state.ltm.mark_unregistered()

        # setup LTM data
        state.ltm.setup(label, parent_state)

        if isinstance(state, smach.Container):
            # Node is a state machine.
            if self.is_registered(state):
                self.setup_callbacks(state, cb_node_start, cb_node_end)

            # recurse towards leafs
            children = state.get_children()
            for child_label in children:
                self.setup_tree(children[child_label], child_label, state)
        else:
            # Node is a state (leaf).
            if self.is_registered(state):
                self.setup_callbacks(state, cb_leaf_start, cb_leaf_end)

    @staticmethod

    def register_state(state, tags):
        rospy.loginfo("[LTM]: - registering state (" + str(state.__class__.__name__) + ") with tags: " + str(tags))
        state.ltm = LTMStateData()
        state.ltm.register(tags)

    @staticmethod
    def is_registered(state):
        # Node is considered as registered for LTM purposes if state.ltm attribute is present and marked as registered.
        return hasattr(state, 'ltm') and isinstance(state.ltm, LTMStateData) and state.ltm.is_registered()

    def update_branch_uids(self, state):
        # retrieve parent id
        child_id = state.ltm.uid
        child_label = state.ltm.label
        (parent_id, parent_label) = self.update_branch_uids_rec(state, child_id, child_label)

        # update data
        rospy.logdebug("[LTM]: - Node [" + parent_label + "] was set as parent of node [" + state.ltm.label
                       + "]. (" + str(parent_id) + ") > (" + str(child_id) + ").")
        data = self.registered_episodes[child_id]
        data.parent_id = parent_id

    def update_branch_uids_rec(self, state, child_id, child_label):
        """
        Updates parent_id of the state and children_ids of the next parent.
        returns the next parent_id
        """
        parent = state.ltm.parent

        # state is root
        # - root is always registered and running
        # - add child_id
        if not parent:
            root_id = state.ltm.uid
            root_label = state.ltm.label
            data = self.registered_episodes[root_id]
            if child_id != root_id:
                data.children_ids.add(child_id)
                rospy.logdebug("[LTM]: - Node [" + child_label + "] was added as child of node [" + root_label
                               + "].\n -> (" + str(root_id) + ") > " + str(list(data.children_ids)))
            # No child should get this value!. All children will get it from the next if statement.
            return 0, "<void>"

        # parent state is registered (and running)
        # - add child_id
        # - return parent_id
        if parent.ltm.is_registered():
            parent_id = parent.ltm.uid
            parent_label = parent.ltm.label
            data = self.registered_episodes[parent_id]
            data.children_ids.add(child_id)
            rospy.logdebug("[LTM]: - Node [" + child_label + "] was added as child of node [" + parent_label +
                           "].\n -> (" + str(parent_id) + ") > " + str(list(data.children_ids)))
            return parent_id, parent_label

        # parent is not registered: recursion
        return self.update_branch_uids_rec(parent, child_id, child_label)

    def save_episode(self, episode, label):
        rospy.loginfo("[LTM]: sending episode [" + label + "](" + str(episode.uid) + ") to the LTM server.")
        try:
            req = AddEpisodeRequest()
            req.episode = episode
            req.replace = False
            self.add_episode_client(req)
        except rospy.ServiceException:
            rospy.logwarn("[LTM]: service exception. Couldn't save episode [" + label + "]("
                          + str(episode.uid) + ") into the LTM server.")

    def cb_leaf_start(self, state):
        # TODO: try/except for ROS stuff.
        # get a uid
        try:
            req = RegisterEpisodeRequest()
            req.gather_emotion = True
            req.gather_location = True
            req.gather_streams = True
            req.gather_entities = True
            req.replace = False
            req.generate_uid = True
            req.uid = 0
            res = self.register_episode_client(req)
        except rospy.ServiceException:
            rospy.logwarn("[LTM]: There aren't any available uids. DB is full. This state will not be recorded.")
            return
        rospy.logdebug("[LTM]: - LEAF [" + state.ltm.label + "](" + str(res.uid) + ") - START callback.")

        # set new uid
        state.ltm.clear()
        state.ltm.uid = res.uid

        # compute episode data
        data = EpisodeData()
        data.uid = res.uid
        data.type = Episode.LEAF
        data.when.start = rospy.Time.now()

        # register episode data
        self.registered_episodes[res.uid] = data

        # update uids for parents and children on this branch
        self.update_branch_uids(state)

    def cb_leaf_end(self, state):
        # TODO: try/except for ROS stuff.
        uid = state.ltm.uid
        rospy.logdebug("[LTM]: - LEAF [" + state.ltm.label + "](" + str(uid) + ") - END callback.")

        # build new episode information
        data = self.registered_episodes[uid]
        episode = data.get_episode()
        episode.tags = list(state.ltm.tags)
        episode.when.end = rospy.Time.now()

        # send episode to server
        self.save_episode(episode, state.ltm.label)

        # remove traces
        state.ltm.clear()
        del self.registered_episodes[uid]

    def cb_node_start(self, state):
        # get a uid
        try:
            req = RegisterEpisodeRequest()
            req.gather_emotion = True
            req.gather_location = True
            req.gather_streams = True
            req.gather_entities = True
            req.replace = False
            req.generate_uid = True
            req.uid = 0
            res = self.register_episode_client(req)
        except rospy.ServiceException:
            rospy.logwarn("[LTM]: There aren't any available uids. DB is full. This state will not be recorded.")
            return
        rospy.logdebug("[LTM]: - NODE [" + state.ltm.label + "](" + str(res.uid) + ") - START callback.")

        # set new uid
        state.ltm.clear()
        state.ltm.uid = res.uid

        # compute episode data
        data = EpisodeData()
        data.uid = res.uid
        data.type = Episode.EPISODE
        data.when.start = rospy.Time.now()

        # register episode data
        self.registered_episodes[res.uid] = data

        # update uids for parents and children on this branch
        self.update_branch_uids(state)

    def cb_node_end(self, state):
        uid = state.ltm.uid
        rospy.logdebug("[LTM]: - NODE [" + state.ltm.label + "](" + str(uid) + ") - END callback.")

        # build new episode information
        data = self.registered_episodes[uid]
        episode = data.get_episode()
        episode.tags = list(state.ltm.tags)
        episode.when.end = rospy.Time.now()

        # episode did not register children -> set as LEAF and warn.
        if not episode.children_ids:
            episode.type = Episode.LEAF
            rospy.logwarn("[LTM]: - NODE [" + state.ltm.label + "](" + str(uid)
                          + ") does not register any children. Considering this episode as a LEAF.")

        # send episode to server
        self.save_episode(episode, state.ltm.label)

        # remove traces
        state.ltm.clear()
        del self.registered_episodes[uid]
