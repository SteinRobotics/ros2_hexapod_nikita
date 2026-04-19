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

import logging
import os
from threading import Thread

import speech_recognition as sr

from nikita_communication import package_resource_path

class SpeechToTextOnline(Thread):
    def __init__(self, cb, logger=None):
        Thread.__init__(self)
        self.cb = cb
        self.logger = logger or logging.getLogger(__name__)

        credentials_file = package_resource_path('keys', 'SERVICE_ACCOUNT_KEY.JSON')
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = str(credentials_file)

    def run(self):
        r = sr.Recognizer()
        mic = sr.Microphone()
        recog = "das habe ich nicht verstanden"

        try:
            with mic as source:
                self.logger.info("speech_recognition_online starts")
                r.adjust_for_ambient_noise(source)
                audio = r.listen(source, timeout=10, phrase_time_limit=6)
        except sr.WaitTimeoutError:
            self.logger.warning("Speech recognition timed out, no speech detected")
            if self.cb:
                self.cb(recog)
            return

        try:
            recog = r.recognize_google_cloud(audio, language='de-DE')
            self.logger.info("speech_recognition_online: " + recog)
        except sr.UnknownValueError as u:
            self.logger.warning(str(u))
            self.logger.warning("Google Cloud Speech Recognition could not understand audio")
        except sr.RequestError as e:
            self.logger.error(f"Could not request results from Google Cloud Speech Recognition service; {e}")

        if self.cb:
            self.cb(recog)


if __name__ == "__main__":
    stt = SpeechToTextOnline(None)
    stt.run()
