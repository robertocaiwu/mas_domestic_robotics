#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_turn_base_to_action'],
   package_dir={'mdr_turn_base_to_action': 'ros/src/mdr_turn_base_to_action'}
)

setup(**d)
