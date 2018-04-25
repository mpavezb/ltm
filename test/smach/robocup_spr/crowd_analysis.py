#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from states import Talk, LookForPeople, AnalyzePeople


def get_look_for_crowd_sm():
    sm = smach.StateMachine(outcomes=['succeeded'])

    with sm:
        smach.StateMachine.add(
            'INIT_TALK',
            Talk("Where are you?. I will find you sooner or later."),
            transitions={'succeeded': 'LOOK_FOR_PEOPLE'}
        )

        smach.StateMachine.add(
            'LOOK_FOR_PEOPLE',
            LookForPeople(),
            transitions={'succeeded': 'END_TALK'}
        )

        smach.StateMachine.add(
            'END_TALK',
            Talk("I found you."),
            transitions={'succeeded': 'succeeded'}
        )

    return sm


def get_analyze_crowd_sm():
    sm = smach.StateMachine(outcomes=['succeeded'])

    with sm:

        smach.StateMachine.add(
            'ANALYZE_PEOPLE',
            AnalyzePeople(),
            transitions={'succeeded': 'GIVE_PEOPLE_INFO'}
        )

        smach.StateMachine.add(
            'GIVE_PEOPLE_INFO',
            Talk("You are all very ugly .. uhggg."),
            transitions={'succeeded': 'succeeded'}
        )

    return sm


def get_instance():
    sm = smach.StateMachine(outcomes=['succeeded'])

    with sm:
        smach.StateMachine.add(
            'INIT_TALK',
            Talk("I want to play. Let's find some people first."),
            transitions={'succeeded': 'LOOK_FOR_CROWD'}
        )

        smach.StateMachine.add(
            'LOOK_FOR_CROWD',
            get_look_for_crowd_sm(),
            transitions={'succeeded': 'ANALYZE_CROWD'}
        )

        smach.StateMachine.add(
            'ANALYZE_CROWD',
            get_analyze_crowd_sm(),
            transitions={'succeeded': 'END_TALK'}
        )

        smach.StateMachine.add(
            'END_TALK',
            Talk("That was a lot of fun."),
            transitions={'succeeded': 'succeeded'}
        )

    return sm


def main():
    try:

        rospy.init_node('robocup_spr_crowd_analysis')

        # build machine
        sm = get_instance()

        # smach introspection server
        sis = smach_ros.IntrospectionServer('robocup_spr_crowd_analysis_sis', sm, '/SM_SPR_CROWD_ANALYSIS')
        sis.start()

        # execute machine
        sm.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
