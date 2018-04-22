#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
from ltm.json_parser import JsonParser
from ltm.srv import *


class LoaderNode(object):

    def __init__(self):
        # parameters
        self.source = rospy.get_param("~source", None)
        if self.source is None:
            raise ValueError("Filename cannot be empty.")
        rospy.loginfo("Using filename: " + self.source)

        # clients
        self.add_episode_client = rospy.ServiceProxy('ltm_server/add_episode', AddEpisode)

        # wait for services
        rospy.wait_for_service('ltm_server/add_episode')

    def save(self, episode):
        try:
            req = AddEpisodeRequest()
            req.episode = episode
            self.add_episode_client(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def load_json(self):
        loader = JsonParser()
        data = loader.load_json(self.source)
        episode = loader.json_to_episode(data)
        self.save(episode)


def main():
    try:
        rospy.init_node("ltm_json_loader")
        node = LoaderNode()
        node.load_json()
    except rospy.ROSInterruptException:
        pass
