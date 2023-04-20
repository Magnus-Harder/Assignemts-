#from robot_client import *
import whisper
import os
# Load the model
model = whisper.load_model('base')
# Get transcription
text = model.transcribe(str(os.getcwd())+"/test.wav")['text'].lower()

# Print resulting text
print(text)