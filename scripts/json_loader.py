#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import roslib; roslib.load_manifest('ltm')
import rospy
from ltm.json_parser import JsonParser
from ltm.srv import *


class LoaderNode(object):

    def __init__(self):
        # parameters
        self.filename = rospy.get_param("~filename", None)
        if self.filename is None:
            raise ValueError("Filename cannot be empty.")
        rospy.loginfo("Using filename: " + self.filename)

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
            print "Service call failed: %s"%e

    def load_json(self):
        loader = JsonParser()
        data = loader.load_json(self.filename)
        episode = loader.json_to_episode(data)
        self.save(episode)


def main():
    rospy.init_node("ltm_json_loader")
    node = LoaderNode()
    node.load_json()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
