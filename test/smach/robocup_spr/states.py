#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach


class Talk(smach.State):
    def __init__(self, text=""):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.text = text

    def execute(self, userdata):
        rospy.logwarn('[state]| Talk |: ' + self.text)
        rospy.sleep(1.0)
        return 'succeeded'


class LookForPeople(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.logwarn('[state]| Look for people |: I am looking for people. Where are you hiding?.')
        rospy.sleep(1.0)
        return 'succeeded'


class AnalyzePeople(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.logwarn('[state]| Analyze people |: mmm ... <beep> i am gathering information about you <bop>.')
        rospy.sleep(1.0)
        return 'succeeded'
