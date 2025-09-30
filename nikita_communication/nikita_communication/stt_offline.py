#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Intergrated by Angelo Antikatzidis https://github.com/a-prototype/vosk_ros
# Source code based on https://github.com/alphacep/vosk-api/blob/master/python/example/test_microphone.py from VOSK's example code

# Tuned for the python flavor of VOSK: vosk-0.3.31
# If you do not have vosk then please install it by running $ pip3 install vosk
# If you have a previous version of vosk installed then update it by running $ pip3 install vosk --upgrade
# Tested on ROS Noetic & Melodic. Please advise the "readme" for using it with ROS Melodic

# This is a node that intergrates VOSK with ROS and supports a TTS engine to be used along with it
# When the TTS engine is speaking some words, the recognizer will stop listenning to the audio stream so it won't listen to it self :)

# It publishes to the topic speech_recognition/vosk_result a custom "speech_recognition" message
# It publishes to the topic speech_recognition/final_result a simple string
# It publishes to the topic speech_recognition/partial_result a simple string

#########
# origin take from the ros-vosk package from Angelo Antikatzidis (Apache License 2.0)
# very strongly adapted for robot "Nikita" needs by Christian Stein
#########


import os
import sys
import json
import queue
import vosk
import sounddevice as sd

import pathlib
import time
from threading import Thread


class SpeechToTextOffline(Thread):
    def __init__(self, robotnames, cb):
        Thread.__init__(self)
        self.cb = cb
        self.robotnames = robotnames
        self.isStopListeningRequested = False

        # change the name of the model to match the downloaded model's name
        model = 'vosk-model-small-en-us-0.15'

        # TODO is there an ROS alternative? Do we need to copy the model to the install folder?
        self.model_dir = pathlib.Path(__file__).parent.joinpath('../models/', model).resolve()


        if not os.path.exists(self.model_dir):
            print("Could not find a model at:")
            print(self.model_dir)
            print("Please download a model for your language from https://alphacephei.com/vosk/models")
            print("and unpack as 'model' in the folder /models.")
            # rospy.signal_shutdown('no model installed!')
            exit()

    def stop_listening(self):
        self.isStopListeningRequested = True

    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))

    def is_robotname_in_text(self, text):
        for robotname in self.robotnames:
            if robotname in text:
                return True
        return False

    def run(self):
        self.q = queue.Queue()

        input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if input_dev_num == -1:
            print('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:

        samplerate = int(device_info['default_samplerate'])
        # rospy.set_param('vosk/sample_rate', samplerate)
        # rospy.set_param('vosk/blocksize', self.blocksize)

        model = vosk.Model(str(self.model_dir))

        try:
            # https://python-sounddevice.readthedocs.io/en/0.4.3/api/raw-streams.html#sounddevice.RawInputStream
            # with sd.RawInputStream(samplerate=samplerate, blocksize=16000, device=input_dev_num, dtype='int16',
            #                    channels=1, callback=self.stream_callback):
            with sd.RawInputStream(samplerate=samplerate, blocksize=2000, device=input_dev_num, dtype='int16',
                                   channels=1, callback=self.stream_callback):
                rec = vosk.KaldiRecognizer(model, samplerate)
                isRecognized = False
                result_text = ""

                while not self.isStopListeningRequested:
                    data = self.q.get()
                    if rec.AcceptWaveform(data):
                        # In case of final result
                        result = rec.FinalResult()
                        diction = json.loads(result)
                        lentext = len(diction["text"])

                        if lentext > 2:
                            result_text = diction["text"]
                            print(result_text)
                            isRecognized = True
                        else:
                            isRecognized = False
                        # Resets current results so the recognition can continue from scratch
                        rec.Reset()

                    if isRecognized:
                        # rospy.loginfo("speech_recognition_offline: " + result_text)
                        time.sleep(0.1)
                        isRecognized = False
                        if self.is_robotname_in_text(result_text):
                            break

        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))

        input_dev_num = None
        self.isStopListeningRequested = False
        self.cb()
