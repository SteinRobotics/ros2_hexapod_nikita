#!/usr/bin/env python3

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import os
# Hide the pygame support prompt
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import time
import pygame.mixer as mixer
import pathlib


class MusicPlayer():
    def __init__(self):
        # TODO is there an ROS alternative? Do we need to copy the model to the install folder?
        self.sound_dir = pathlib.Path(__file__).parent.joinpath('../soundfiles/').resolve()
        mixer.init()
        mixer.music.set_volume(0.9)  #between 0.0 and 1.0

    def play(self, filename, volume=0.5, cb=None):
        mixer.music.set_volume(volume)  #between 0.0 and 1.0
        if filename.endswith(".wav"):
            self.play_wav(self.sound_dir / "wav" / filename)
        elif filename.endswith(".mp3"):
            self.play_mp3(self.sound_dir / "mp3" / filename)

        if cb:
            cb()

    def play_random_music(self):
        self.play_wav(self.sound_dir / "mp3" / "musicfox_hot_dogs_for_breakfast.mp3")

    def is_busy(self):
        return mixer.music.get_busy()
    

    def play_wav(self, wav):
        soundObj = mixer.Sound(wav)
        channel = soundObj.play()

        while channel.get_busy() and not self.is_stop_requested:
            time.sleep(0.1)
        mixer.music.stop() # not sure this one is correct

    def play_mp3(self, mp3):
        mixer.music.load(mp3)
        # mixer.music.play(-1, 0.0) # -1: endless, 0.0: from beginning
        mixer.music.play()

    def stop_playing(self, cb=None):
        mixer.music.stop()
        if cb:
            cb()

if __name__ == "__main__":
    player = MusicPlayer()
    player.play_random_music()
