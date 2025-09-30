#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Projektname
# Ivanka
# Projektnummer
# 436090190916
# Projekt-ID
# ivanka-316619

# stt service account
# "client_email": "my-stt-sa@ivanka-316619.iam.gserviceaccount.com"

# pip3 install SpeechRecognition
# sudo apt-get install portaudio19-dev
# pip3 install pyaudio

# pip3 install google-api-python-client
# pip3 install google-cloud-speech
# pip3 install oauth2client

# ivanka-316619-bd6c9a573ccf.json

import os
from threading import Thread
import speech_recognition as sr
import pathlib

class SpeechToTextOnline(Thread):
    def __init__(self, cb):
        Thread.__init__(self)
        self.cb = cb

        # TODO is there an ROS alternative? Do we need to copy the model to the install folder?
        credentials_file = pathlib.Path(__file__).parent.joinpath('../keys/SERVICE_ACCOUNT_KEY.JSON').resolve()
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = str(credentials_file)
        # self.pub = rospy.Publisher('speech_recognition_online', String, queue_size=10)

    def run(self):
        r = sr.Recognizer()
        file = sr.Microphone()
        recog = "das habe ich nicht verstanden"

        with file as source:
            print("speech_recognition_online starts")
            audio = r.adjust_for_ambient_noise(source)
            audio = r.listen(source, timeout=10,  phrase_time_limit=6)

        try:
            # recog = r.recognize_google_cloud(audio, language = 'en-US')
            recog = r.recognize_google_cloud(audio, language='de-DE')
            print("speech_recognition_online: " + recog)

        except sr.UnknownValueError as u:
            print(u)
            print("Google Cloud Speech CRecognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Cloud Speech CRecognition service; {0}".format(e))

        if self.cb:
            self.cb(recog)


if __name__ == "__main__":
    stt = SpeechToTextOnline(None)
    stt.run()
