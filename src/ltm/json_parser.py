#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

# json
import json

# time manipulation
import time
from calendar import timegm

# ROS
from rospy import Time
from ltm.msg import *
from geometry_msgs.msg import Point

from platform import uname
from os import environ


class JsonParser(object):

    def __init__(self):
        pass

    # -------------------------------------------------------------------
    # required to load json files with python strings instead of unicode.
    # -------------------------------------------------------------------
    def json_load_byteified(self, file_handle):
        return self._byteify(
            json.load(file_handle, object_hook=self._byteify),
            ignore_dicts=True
        )

    def _byteify(self, data, ignore_dicts=False):
        # if this is a unicode string, return its string representation
        if isinstance(data, unicode):
            return data.encode('utf-8')
        # if this is a list of values, return list of byteified values
        if isinstance(data, list):
            return [self._byteify(item, ignore_dicts=True) for item in data]
        # if this is a dictionary, return dictionary of byteified keys and values
        # but only if we haven't already byteified it
        if isinstance(data, dict) and not ignore_dicts:
            return {
                self._byteify(key, ignore_dicts=True): self._byteify(value, ignore_dicts=True)
                for key, value in data.iteritems()
            }
        # if it's anything else, return it in its original form
        return data
        # -------------------------------------------------------------------

    def load_json(self, filename):
        return self.json_load_byteified(open(filename))

    def json_to_episode(self, data):
        episode = Episode()
        episode.uid = data['uid']
        episode.type = data['type']
        episode.tags = data['tags']
        self.get_data(episode, 'children_tags', data)
        episode.parent_id = data['parent_id']
        episode.children_ids = data['children_ids']
        episode.info = self.json_to_field_info(data['info'])
        episode.what = self.json_to_field_what(data['what'])
        episode.where = self.json_to_field_where(data['where'])
        episode.when = self.json_to_field_when(data['when'])
        episode.relevance = self.json_to_field_relevance(data['relevance'])
        return episode

    @staticmethod
    def auto_info():
        info = Info()
        info.n_usages = 0
        info.source = "text-json"
        info.creation_date = Time.now()
        info.last_access = Time.now()
        info.ltm_version = "0.0.0"
        info.ros_version = environ.get("ROS_DISTRO")
        info.os_version = ''.join(uname())
        return info

    @staticmethod
    def get_data(field, name, data, fun=(lambda x: x)):
        if name in data:
            setattr(field, name, fun(data[name]))
            return True
        return False

    @staticmethod
    def use_auto_data(data):
        return data is "~"

    def json_to_field_info(self, data):
        info = Info()
        if self.use_auto_data(data):
            return self.auto_info()
        self.get_data(info, 'n_usages', data)
        self.get_data(info, 'source', data)
        self.get_data(info, 'creation_date', data, self.str_to_ros_time)
        self.get_data(info, 'last_access', data, self.str_to_ros_time)
        self.get_data(info, 'ltm_version', data)
        self.get_data(info, 'ros_version', data)
        self.get_data(info, 'os_version', data)
        return info

    # TODO: handle streams and entities
    def json_to_field_what(self, data):
        what = What()
        if self.use_auto_data(data):
            return what
        self.get_data(what, 'features', data)
        self.get_data(what, 'feature_values', data)
        return what

    def json_to_field_where(self, data):
        where = Where()
        if self.use_auto_data(data):
            return where
        self.get_data(where, 'frame_id', data)
        self.get_data(where, 'map_name', data)
        self.get_data(where, 'position', data, self.json_to_ros_position)
        self.get_data(where, 'location', data)
        self.get_data(where, 'area', data)
        self.get_data(where, 'children_locations', data)
        self.get_data(where, 'children_areas', data)
        self.get_data(where, 'children_hull', data)
        return where

    def json_to_field_when(self, data):
        when = When()
        if self.use_auto_data(data):
            return when
        self.get_data(when, 'start', data, self.str_to_ros_time)
        self.get_data(when, 'end', data, self.str_to_ros_time)
        return when

    def json_to_field_relevance(self, data):
        relevance = Relevance()
        if self.use_auto_data(data):
            return relevance
        self.get_data(relevance, 'emotional', data, self.json_to_field_emotional_relevance)
        self.get_data(relevance, 'historical', data, self.json_to_field_historical_relevance)
        return relevance

    def json_to_field_emotional_relevance(self, data):
        emotional = EmotionalRelevance()
        if self.use_auto_data(data):
            return emotional
        self.get_data(emotional, 'software', data)
        self.get_data(emotional, 'software_version', data)
        self.get_data(emotional, 'registered_emotions', data)
        self.get_data(emotional, 'registered_values', data)
        self.get_data(emotional, 'emotion', data)
        self.get_data(emotional, 'value', data)
        self.get_data(emotional, 'children_emotions', data)
        self.get_data(emotional, 'children_values', data)
        return emotional

    def json_to_field_historical_relevance(self, data):
        historical = HistoricalRelevance()
        if self.use_auto_data(data):
            return historical
        self.get_data(historical, 'value', data)
        self.get_data(historical, 'last_update', data, self.str_to_ltm_date)
        self.get_data(historical, 'next_update', data, self.str_to_ltm_date)
        return historical

    def json_to_ros_position(self, data):
        position = Point()
        self.get_data(position, 'x', data)
        self.get_data(position, 'y', data)
        self.get_data(position, 'z', data)
        return position

    # ignores nanoseconds
    @staticmethod
    def str_to_ros_time(string):
        # time module supports up to microsecond resolution (ROS standard is nanosecond)

        # parse date
        no_nsec = string[:23] if len(string) > 23 else string
        utc_time = time.strptime(no_nsec, "UTC_%Y/%m/%d_%H:%M:%S")
        epoch_time = timegm(utc_time)

        # compute nanoseconds
        nsecs_str = string[24:] if len(string) > 24 else string
        nsecs = int(nsecs_str) * 10**(9-len(nsecs_str))

        return Time(epoch_time, nsecs)

    @staticmethod
    def str_to_ltm_date(date_str):
        date = Date()
        values = date_str.split("/")
        date.year = int(values[0])
        date.month = int(values[1])
        date.day = int(values[2])
        return date
