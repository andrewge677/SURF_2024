#!/usr/bin/env python3

import os
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from google.cloud import speech
from google.cloud.speech import SpeechClient, RecognitionConfig, RecognitionAudio
import sounddevice as sd
import threading

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/student/ros_ws_py3/surf-test-426821-4c653702a368.json'

class SpeechRecognitionHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        response = {'status': 'running'}
        self.wfile.write(json.dumps(response).encode())

    def do_POST(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        
        transcript = recognize_speech()
        response = {'transcript': transcript}
        self.wfile.write(json.dumps(response).encode())

def record_audio(timeout, callback):
    fs = 16000  # Adjust the sampling frequency as needed
    print("Listening... Press Ctrl+C to stop.")
    try:
        audio_data = sd.rec(int(fs * timeout), samplerate=fs, channels=1, dtype='int16')
        callback()  # Indicate that recording has started
        sd.wait()
        return audio_data.flatten().tobytes()
    except KeyboardInterrupt:
        print("\nRecording stopped.")
        return None

def recognize_speech():
    client = speech.SpeechClient()
    config = RecognitionConfig(
        encoding=RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code="en-US",
    )
    def callback():
        print("Recording...")

    audio_data = record_audio(7, callback)
    if audio_data is None:
        print("Did not detect audio. Retrying.")
        return ''

    audio = RecognitionAudio(content=audio_data)
    response = client.recognize(config=config, audio=audio)

    if response.results:
        transcripts = response.results[0].alternatives[0].transcript.lower()
        print(transcripts)
        return transcripts
    else:
        print("Did not transcribe text. Retrying.")
        return ''

def run_server():
    server_address = ('', 8080)
    httpd = HTTPServer(server_address, SpeechRecognitionHandler)
    print('Running speech recognition server...')
    httpd.serve_forever()

if __name__ == "__main__":
    server_thread = threading.Thread(target=run_server)
    server_thread.start()
