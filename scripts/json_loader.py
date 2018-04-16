#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'
import json
import rospy


class JsonLoader(object):

    def __init__(self):
        self.filename = rospy.get_param("~filename", None)
        self.data = None
        if self.filename is None:
            raise ValueError("Filename cannot be empty.")
        rospy.loginfo("Using filename: " + self.filename)

    def load_file(self):
        self.data = json.load(open(self.filename))

    def dump_to_ltm(self):
        print self.data


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