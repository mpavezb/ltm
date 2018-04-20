#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'
import json
import rospy
import time as py_time
from ltm.msg import *
from calendar import timegm
from std_msgs.msg import Time as std_time
from geometry_msgs.msg import Pose


# -------------------------------------------------------------------
# required to load json files with python strings instead of unicode.
# -------------------------------------------------------------------
def json_load_byteified(file_handle):
    return _byteify(
        json.load(file_handle, object_hook=_byteify),
        ignore_dicts=True
    )


def _byteify(data, ignore_dicts=False):
    # if this is a unicode string, return its string representation
    if isinstance(data, unicode):
        return data.encode('utf-8')
    # if this is a list of values, return list of byteified values
    if isinstance(data, list):
        return [_byteify(item, ignore_dicts=True) for item in data]
    # if this is a dictionary, return dictionary of byteified keys and values
    # but only if we haven't already byteified it
    if isinstance(data, dict) and not ignore_dicts:
        return {
            _byteify(key, ignore_dicts=True): _byteify(value, ignore_dicts=True)
            for key, value in data.iteritems()
        }
    # if it's anything else, return it in its original form
    return data
# -------------------------------------------------------------------


class JsonLoader(object):

    def __init__(self):
        self.filename = rospy.get_param("~filename", None)
        self.data = None
        if self.filename is None:
            raise ValueError("Filename cannot be empty.")
        rospy.loginfo("Using filename: " + self.filename)

    def load_file(self):
        self.data = json_load_byteified(open(self.filename))

    def dump_to_ltm(self):
        episode = self.json_to_episode(self.data)
        # print self.data
        print episode

    def json_to_episode(self, data):
        episode = Episode()
        episode.uid = data['uid']
        episode.type = data['type']
        episode.tags = data['tags']
        episode.info = self.json_to_field_info(data['info'])
        episode.what = self.json_to_field_what(data['what'])
        episode.where = self.json_to_field_where(data['where'])
        episode.when = self.json_to_field_when(data['when'])
        episode.relevance = self.json_to_field_relevance(data['relevance'])
        return episode

    def json_to_field_info(self, data):
        info = Info()
        info.episode_number = data['episode_number']
        info.n_usages = data['n_usages']
        info.source = data['source']
        info.creation_date = self.str_to_ros_time(data['creation_date'])
        info.last_access = self.str_to_ros_time(data['last_access'])
        info.ltm_version = data['ltm_version']
        info.ros_version = data['ros_version']
        info.os = data['os']
        info.os_version = data['os_version']
        return info

    # TODO: handle streams and entities
    @staticmethod
    def json_to_field_what(data):
        what = What()
        what.parent_id = data['parent_id']
        what.children_ids = data['children_ids']
        what.features = data['features']
        what.feature_values = data['feature_values']
        return what

    def json_to_field_where(self, data):
        where = Where()
        where.pose = self.json_to_ros_pose(data['pose'])
        where.frame_id = data['frame_id']
        where.map_name = data['map_name']
        where.location_name = data['location_name']
        where.location_area = data['location_area']
        return where

    def json_to_field_when(self, data):
        when = When()
        when.start = self.str_to_ros_time(data['start'])
        when.end = self.str_to_ros_time(data['end'])
        return when

    def json_to_field_relevance(self, data):
        relevance = Relevance()
        relevance.emotional = self.json_to_field_emotional_relevance(data['emotional'])
        relevance.historical = self.json_to_field_historical_relevance(data['historical'])
        return relevance

    @staticmethod
    def json_to_field_emotional_relevance(data):
        emotional = EmotionalRelevance()
        emotional.software = data['software']
        emotional.software_version = data['software_version']
        emotional.registered_emotions = data['registered_emotions']
        emotional.registered_values = data['registered_values']
        emotional.emotion = data['emotion']
        emotional.value = data['value']
        return emotional

    def json_to_field_historical_relevance(self, data):
        historical = HistoricalRelevance()
        historical.value = data['value']
        historical.last_update = self.str_to_ltm_date(data['last_update'])
        historical.next_update = self.str_to_ltm_date(data['next_update'])
        return historical

    @staticmethod
    def json_to_ros_pose(data):
        pose = Pose()
        position = data['position']
        orientation = data['orientation']
        pose.position.x = position['x']
        pose.position.y = position['y']
        pose.position.z = position['z']
        pose.orientation.x = orientation['x']
        pose.orientation.y = orientation['y']
        pose.orientation.z = orientation['z']
        pose.orientation.w = orientation['w']
        return pose

    # ignores nanoseconds
    @staticmethod
    def str_to_ros_time(string):
        time = std_time()
        utc_time = py_time.strptime(string, "UTC_%Y/%m/%d_%H:%M:%S.%f")
        epoch_time = timegm(utc_time)
        time.data.secs = epoch_time
        time.data.nsecs = 0
        return time

    @staticmethod
    def str_to_ltm_date(date_str):
        date = Date()
        values = date_str.split("/")
        date.year = int(values[0])
        date.month = int(values[1])
        date.day = int(values[2])
        return date


def main():
    rospy.init_node("ltm_json_loader")

    node = JsonLoader()
    node.load_file()
    node.dump_to_ltm()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
