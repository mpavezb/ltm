#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

from os import listdir
from os.path import abspath, isdir, isfile, join
import rospy
from ltm.json_parser import JsonParser
from ltm.srv import *


class LoaderNode(object):

    def __init__(self):
        self._is_file = None
        self.loader = JsonParser()
        self.saved_episodes = []
        self.failed_episodes = []
        self.root_episodes = []

        # ROS parameters
        self.source = rospy.get_param("~source", None)
        self.setup_path()

        # ROS clients
        rospy.loginfo("Waiting for LTM server to be up.")
        self.register_episode_client = rospy.ServiceProxy('ltm/register_episode', RegisterEpisode)
        self.add_episode_client = rospy.ServiceProxy('ltm/add_episode', AddEpisode)
        self.update_tree_client = rospy.ServiceProxy('ltm/update_tree', UpdateTree)

        # Wait for ROS services
        self.add_episode_client.wait_for_service()
        self.update_tree_client.wait_for_service()

    def setup_path(self):
        if self.source is None:
            rospy.logerr("Source parameter cannot be empty.")
            exit(1)
        self.source = abspath(self.source)
        if isdir(self.source):
            self._is_file = False
            rospy.loginfo("Loading directory: " + self.source)
        elif isfile(self.source):
            self._is_file = True
            rospy.loginfo("Loading JSON file: " + self.source)
        else:
            rospy.logerr("Source parameter is not a valid filename or directory: '" + self.source + "'")
            exit(1)

    def save(self, episode):
        try:
            # register episode
            reg_req = RegisterEpisodeRequest()
            reg_req.gather_emotion = False
            reg_req.gather_location = False
            reg_req.gather_streams = False
            reg_req.gather_entities = False
            reg_req.replace = True
            reg_req.generate_uid = False
            reg_req.uid = episode.uid
            self.register_episode_client(reg_req)

            # add episode
            req = AddEpisodeRequest()
            req.episode = episode
            req.replace = True
            res = self.add_episode_client(req)
            if res.succeeded:
                self.saved_episodes.append(episode.uid)
                if episode.uid == episode.parent_id:
                    self.root_episodes.append(episode.uid)
                return True
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        self.failed_episodes.append(episode.uid)
        return False

    def load_json(self):
        if self._is_file:
            self.load_file(self.source)
            if len(self.saved_episodes) == 1:
                self.update_tree(self.saved_episodes[0])
        else:
            self.load_folder()

    def load_file(self, filename):
        data = self.loader.load_json(filename)
        episode = self.loader.json_to_episode(data)
        return self.save(episode)

    def update_tree(self, uid):
        rospy.loginfo(" - updating tree with root uid: '" + str(uid) + "'.")
        try:
            self.update_tree_client(uid)
            return True
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        return False

    def load_folder(self):
        # get JSONs
        files = [f for f in listdir(self.source) if isfile(join(self.source, f)) and f.endswith('.json')]
        files.sort()
        if not files:
            rospy.logwarn("No .json files were found.")
            return

        # load JSONs
        for filename in files:
            full_filename = join(self.source, filename)
            rospy.loginfo(" - loading json: " + filename)
            self.load_file(full_filename)
        rospy.loginfo("The following episodes were saved: " + str(self.saved_episodes))
        rospy.loginfo("The following episodes were not saved: " + str(self.failed_episodes))
        rospy.loginfo("The following roots were saved: " + str(self.root_episodes))

        # update tree
        for uid in self.root_episodes:
            self.update_tree(uid)


def main():
    try:
        rospy.init_node("ltm_json_loader")
        node = LoaderNode()
        node.load_json()
    except rospy.ROSInterruptException:
        pass
