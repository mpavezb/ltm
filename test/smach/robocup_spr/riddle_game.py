#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from states import Talk


def get_instance():
    sm = smach.StateMachine(outcomes=['succeeded'])

    with sm:
        smach.StateMachine.add(
            'INIT_TALK',
            Talk("I want to play riddles."),
            transitions={'succeeded': 'END_TALK'}
        )

        # TODO

        smach.StateMachine.add(
            'END_TALK',
            Talk("That was a lot of fun."),
            transitions={'succeeded': 'succeeded'}
        )

    return sm


def main():
    try:
        rospy.init_node('robocup_spr_riddle_game')

        # build machine
        sm = get_instance()

        # smach introspection server
        sis = smach_ros.IntrospectionServer('robocup_spr_riddle_game_sis', sm, '/SM_SPR_RIDDLE_GAME')
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
