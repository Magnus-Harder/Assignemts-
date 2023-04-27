#%%
#from robot_client import *
import whisper
import os
# Load the model
model = whisper.load_model('base')
# Get transcription
text = model.transcribe("../audiocapture.mp3")['text']
text = text.lower()
#%%
def get_command(robot):

    #text = robot.listen()
    #text = text.lower()

    if "error" in text:
        print("Restate your command")
        #robot.say("Restate your command")
        return get_command(robot)

    if "move" in text:
        print("Move Command")
        if "north" in text:
            return 
        elif "south" in text:
            return
        elif "east" in text:
            return "East"
        elif "west" in text:
            return
        else:
            return #robot.say("Move Command is missing a direction")
        
    elif "push" in text:
        if "north" in text:
            return 
        elif "south" in text:
            return
        elif "east" in text:
            return
        elif "west" in text:
            return
        else:
            return #robot.say("Push Command is missing a direction")
    

    else:
        #robot.say("Command not recognized. Please try again.")
        return #get_command(robot)


# Print resulting text
get_command(None)
#print(text)
# %%
