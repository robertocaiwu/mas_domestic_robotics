#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['SpeechRecognition', 'mdr_speech_recognition'],
   package_dir={'SpeechRecognition': 'common/src/SpeechRecognition',
                'mdr_speech_recognition': 'ros/src/mdr_speech_recognition'}
)

setup(**d)
