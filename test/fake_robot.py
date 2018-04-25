#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from ltm.srv import *
from ltm.msg import *
from geometry_msgs.msg import Point

import random


class Robot(object):

    def __init__(self):
        rospy.loginfo("Your fake robot is ready to tell you lies.")

        # parameters
        self.where_locations = rospy.get_param("~where/locations", ["living_room", "kitchen"])
        self.where_areas = rospy.get_param("~where/areas", ["arena"])
        self.where_map = rospy.get_param("~where/map_name", "map")
        self.where_frame_id = rospy.get_param("~where/frame_id", "/map")

        # ROS API
        rospy.Service('~get_location', GetEpisode, self.service_get_where)
        rospy.Service('~get_emotion', GetEpisode, self.service_get_emotion)

    def service_get_where(self, req):
        rospy.loginfo("Generating a fake location of the robot.")
        position = Point()
        position.x = random.uniform(-10.0, 10.0)
        position.y = random.uniform(-10.0, 10.0)

        # generate Where field
        where = Where()
        where.position = position
        where.map_name = self.where_map
        where.frame_id = self.where_frame_id
        where.location = random.choice(self.where_locations)
        where.area = random.choice(self.where_areas)

        # return response
        episode = Episode()
        episode.where = where
        res = GetEpisodeResponse()
        res.succeeded = True
        res.episode = episode
        return res

    def service_get_emotion(self, req):
        rospy.loginfo("Generating fake emotions for the robot.")
        emotions = EmotionalRelevance()
        emotions.software = "fake_robot_emo_generator"
        emotions.software_version = "1.0.0"
        emotions.registered_emotions = []
        emotions.registered_values = []
        emotions.value = 0.5
        emotions.emotion = EmotionalRelevance.JOY

        # return response
        episode = Episode()
        episode.relevance.emotional = emotions
        res = GetEpisodeResponse()
        res.succeeded = True
        res.episode = episode
        return res


def main():
    rospy.init_node("ltm_fake_robot")
    Robot()

    # Wait for ctrl-c to stop the application
    rospy.on_shutdown(lambda: rospy.logwarn("Ha det bra!."))
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass