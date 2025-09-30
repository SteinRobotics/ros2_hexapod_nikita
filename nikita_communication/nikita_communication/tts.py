#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# online
# pip3 install gTTS

# offline
# https://wiki.ubuntuusers.de/eSpeak_NG/
# sudo apt-get install espeak-ng espeak-ng-espeak mbrola

from gtts import gTTS
import subprocess
import os.path
import re
import pathlib
from nikita_communication.music_player import MusicPlayer

class TextToSpeech():
    def __init__(self, music_player, language):
        self.language = language
        self.music_player = music_player

        # TODO is there an ROS alternative? Do we need to copy the model to the install folder?
        self.tts_cache = pathlib.Path(__file__).parent.joinpath('../tts_cache/').resolve()

    def run(self, text, cb=None):
        # print("run tts: " + text)
        text_shorten = re.sub("[^a-zA-Z0-9]+", "", text)[:60]
        text_file = text_shorten.lower() + ".mp3"
        filename = self.tts_cache / text_file

        try:
            if not os.path.exists(filename):
                tts = gTTS(text=text, lang=self.language)
                tts.save(str(filename))

            self.music_player.play(str(filename))

        except Exception as e:
            print(e)
            print("offline tts")
            subprocess.call(["espeak-ng","-v" + self.language + "+f3", text], stderr=subprocess.STDOUT)

        if cb:
            cb()

if __name__=="__main__":
    music_player = MusicPlayer()
    tts = TextToSpeech(music_player, "de")
    tts.run("Hallo wie geht es dir")


