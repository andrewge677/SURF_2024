#! /usr/bin/env python3

import os
import json
from google.cloud import speech
from google.cloud.speech import SpeechClient, RecognitionConfig, RecognitionAudio
import speech_recognition as sr
import sounddevice as sd
import time
import numpy as np

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = os.path.join('/home/student/ros_ws_py3/surf-test-426821-4c653702a368.json')

def record_audio(timeout, callback):
        fs = 16000    # Adjust the sampling frequency as needed
        # print("Listening... Press Ctrl+C to stop.")
        try:
                audio_data = sd.rec(int(fs * timeout), samplerate=fs, channels=1, dtype=np.int16)
                callback()    # Indicate that recording has started
                sd.wait()
                return audio_data.flatten().tobytes()
        except KeyboardInterrupt:
                print("\nRecording stopped.")

def recognize_speech(timeout=7, max_attempts=3):
        client = speech.SpeechClient()
        attempts = 0

        while attempts < max_attempts:
                def callback():
                        print("Recording...")
                        # print()
                config = RecognitionConfig(
                        encoding=RecognitionConfig.AudioEncoding.LINEAR16,
                        sample_rate_hertz=16000,
                        language_code="en-US",
                )
                audio_data = record_audio(timeout, callback)
                audio = RecognitionAudio(content=audio_data)
                response = client.recognize(config=config, audio=audio)

                human_hist = {'speech':[], 'text':[]}

                if response.results:
                        transcripts = (response.results[0].alternatives[0].transcript).lower()
                        confidence = response.results[0].alternatives[0].confidence
                        if transcripts:
                                # print("Transcript: {}".format(transcripts))
                                human_hist['speech'].append(audio_data)
                                human_hist['text'].append([transcripts, confidence])
                                # return transcripts, confidence
                                return transcripts
                        else:
                                attempts +=1
                                if attempts < max_attempts:
                                    print('I Could not hear you, please repeat.')
                                    print("Please repeat your utterance (Attempt {}/{}).".format(attempts, max_attempts-1))                                
                else:
                        attempts += 1
                        if attempts < max_attempts:
                                print('I Could not hear you, please repeat.')
                                print("Please repeat your utterance (Attempt {}/{}).".format(attempts, max_attempts-1))

        print("Maximum number of attempts reached. Exiting.")
        human_hist['speech'].append(audio_data)
        human_hist['text'].append(['', 0.0])
        return '', 0.0

while True:
        print(recognize_speech())



# PYTHON 3 FILE