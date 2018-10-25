#!/usr/bin/env python
from __future__ import print_function
import os
import rospy
from std_msgs.msg import String
from SpeechRecognition.KaldiRecognizer import KaldiRecognizer
import speech_recognition as sr

class SpeechRecognizer(object):

    def __init__(self):
        rospy.init_node("speech_recognizer")
        self.pub = rospy.Publisher("speech_recognizer", String, latch=True, queue_size=1)
        self.model_directory = '{0}/speech/py-kaldi-asr/data/models/nnet3_en_f' \
            .format(os.path.expanduser('~'))
        self.recognizer = KaldiRecognizer()
        self.recognizer.load_kaldi_model(model_directory=self.model_directory, language='en-US')
        self.microphone = sr.Microphone(device_index=6,sample_rate=16000)
        self.recorder = sr.Recognizer()

    def recognize(self):
        with self.microphone as source:
            self.recorder.adjust_for_ambient_noise(source)

        try:
            while not rospy.is_shutdown():
                rospy.loginfo('Listening...')
                with self.microphone as source:
                    audio = self.recorder.listen(source)
                    rospy.loginfo('Got a sound; recognizing...')

                    recognized_speech = ""
                    try:
                        recognized_speech = self.recognizer.recognize_kaldi(audio)
                    except Exception:
                        rospy.logerr("Could not understand audio.")
                    if recognized_speech != "":
                        rospy.loginfo("You said: " + recognized_speech)
                        self.pub.publish(recognized_speech)

        except Exception as exc:
            rospy.logerr(exc)

def main():
    speech_recognizer = SpeechRecognizer()
    speech_recognizer.recognize()
    rospy.spin()
