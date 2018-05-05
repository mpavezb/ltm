#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

from manager import Manager


def setup(machine):
    # Manager().introspect(machine)
    Manager().setup(machine)


def register_state(state, tags=list()):
    Manager().register_state(state, tags)
