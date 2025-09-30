#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os.path
from stoppable_thread import StoppableThread
import pathlib
# import openai

# https://github.com/uesleibros/OpenGPT
# free to use GPT


class ChatBotThread(StoppableThread):
    def __init__(self, chat_question, cb):
        StoppableThread.__init__(self)
        self.cb = cb
        self.chat_question = chat_question

    def run(self):
        context = "Lebensbejahender Serviceroboter namens Nikita"
        message = self.chat_question
        response = openai.Completion.create(
            engine="gpt-3.5-turbo",
            prompt=f"Chat:\n{context}\nUser: {message}\n",
            max_tokens=100
        )

        reply = response.choices[0].text.strip()
        reply = reply.split('Chat: ', 1)[-1]
        reply = reply.split('Nikita: ', 1)[-1]
        print(reply)

        if self.cb:
            self.cb(reply)


class ChatBot():
    def __init__(self):
        # TODO is there an ROS alternative? Do we need to copy the model to the install folder?
        openai_api_keyfile = pathlib.Path(__file__).parent.joinpath('../keys/openai_key.txt').resolve()

        with open(openai_api_keyfile) as f:
            openai_api_key = f.read()

        openai.organization = "org-xQl8WlHgNR27XbF1YDirVmIp"
        openai.api_key = openai_api_key

    def setChatRequest(self, chat_question, callback):
        self.chat_question = chat_question
        self.cb = callback

        self.chatBotThread = ChatBotThread(chat_question, callback)
        self.chatBotThread.start()

    def is_thread_running(self):
        self.chatBotThread.stopped()

if __name__=="__main__":
    chatBot = ChatBot()
    chatBot.setChatRequest("Hallo, wie ist dein Name?", None)



