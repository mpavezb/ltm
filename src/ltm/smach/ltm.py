#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import smach
import types


def introspect(sm, label="root"):
    if isinstance(sm, smach.StateMachine):
        children = sm.get_children()
        print " - node: " + label + " has (" + str(len(children)) + ") children"
        for child_label in children:
            introspect(children[child_label], child_label)
    else:
        print " - leaf: " + label


def start_cb(userdata, init_states, label):
    print " - running START callback for state: " + label


def termination_cb(userdata, terminal_states, outcome, label):
    print " - running TERMINATION callback for state: " + label


def leaf_start_cb(self, ud, label):
    print " - running START callback for LEAF: " + label


def leaf_termination_cb(self, ud, label):
    print " - running TERMINATION callback for LEAF: " + label


def setup(state, label="root"):
    if isinstance(state, smach.StateMachine):
        # register cb for nodes
        state.register_start_cb(start_cb, [label])
        state.register_termination_cb(termination_cb, [label])

        # recurse
        children = state.get_children()
        for child_label in children:
            setup(children[child_label], child_label)
    else:
        # leaf
        fn_start = types.MethodType(leaf_start_cb, state)
        fn_termination = types.MethodType(leaf_termination_cb, state)
        fn_execute = state.execute

        def ltm_execute(self, ud):
            fn_start(ud, label)
            try:
                outcome = fn_execute(ud)
            finally:
                fn_termination(ud, label)
            return outcome

        state.execute = types.MethodType(ltm_execute, state)
