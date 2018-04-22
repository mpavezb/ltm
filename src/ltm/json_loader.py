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

        # ROS parameters
        self.source = rospy.get_param("~source", None)
        self.setup_path()

        # ROS clients
        rospy.loginfo("Waiting for LTM server to be up.")
        self.add_episode_client = rospy.ServiceProxy('ltm_server/add_episode', AddEpisode)

        # Wait for ROS services
        self.add_episode_client.wait_for_service()

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
            req = AddEpisodeRequest()
            req.episode = episode
            self.add_episode_client(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def load_json(self):
        if self._is_file:
            self.load_file(self.source)
        else:
            self.load_folder()

    def load_file(self, filename):
        data = self.loader.load_json(filename)
        episode = self.loader.json_to_episode(data)
        self.save(episode)

    def load_folder(self):
        files = [f for f in listdir(self.source) if isfile(join(self.source, f)) and f.endswith('.json')]
        files.sort()
        if not files:
            rospy.logwarn("No .json files were found.")
            return
        for filename in files:
            full_filename = join(self.source, filename)
            rospy.loginfo(" - loading json: " + filename)
            self.load_file(full_filename)


def main():
    try:
        rospy.init_node("ltm_json_loader")
        node = LoaderNode()
        node.load_json()
    except rospy.ROSInterruptException:
        pass
