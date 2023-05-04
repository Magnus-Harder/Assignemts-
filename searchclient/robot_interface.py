import socket
import msgpack
import time
import math
import sys
import whisper
import random

model = whisper.load_model('base')

"""
Using the robot agent type differs from previous agent types.
  - Firstly, install additional package 'msgpack'.
    Use pip for installation, like this: 
        'python -m pip install numpy msgpack'. 
    Exact steps may vary based on your platform and Python 
    installation. 

  - Secondly, you don't need the Java server for the robot. So, the command
    to start the search client in the terminal is different, for example:
        'python searchclient/searchclient.py -robot -ip 192.168.0.102 -level levels/SAsoko1_04.lvl'
    runs the searchclient with the 'robot' agent type on the robot at IP 192.168.0.102.

  - To connect to the robots, connect to the Pepper hotspot. To reduce
    the load on the hotspot, please disconnect between your sessions.
    
  - A good starting point is using something similar to the 'classic' agent type and then
    replacing it with calls to the 'robot' interface.
"""

# for linux and mac
# export PYTHONPATH=${PYTHONPATH}:/home/seb/Downloads/python-sdk/lib/python2.7/site-packages


def degrees(degree):
    '''
    :param degree: angle in degrees  
    :return: angle in radians
    '''
    return degree * (math.pi / 180)

# Robot Client Class for use with the robot server class
class RobotClient():
    def __init__(self, ip):
        self.ip = ip

        '''
        Sometimes the robots can change their IP address. This is a workaround for that. 

        You will also need to change the port numbers and the login in the server class.
        
        '''

        if self.ip == '192.168.1.104':
            port = 5007  # if port fails you have from 5000-5009
        elif self.ip == '192.168.1.106':
            port = 5017  # if port fails you have from 5010-5019
        elif self.ip == '192.168.1.107':
            port = 5021 # if port fails you have from 5020-5029
        elif self.ip == '192.168.1.108':
            port = 5033 # if port fails you have from 5030-5039


        self.host = socket.gethostname()  # as both code is running on same pc
        self.port = port  # socket server port number

        self.client_socket = socket.socket()  # instantiate
        self.client_socket.connect((self.host, self.port))  # connect to the server


        '''
        Direction mapping for the robot. The orientation of the robot is given by the 
        angle of the world coordinate system. We will assume that world/level coordinate 
        system is aligned with classroom such that north is facing the direction of the 
        board/screens.

        '''
        self.direction_mapping = { 'Move(N)': 90,
                        'Move(E)': 0,
                        'Move(S)': 270,
                        'Move(W)': 180,
                        'Push(N,N)': 90,
                        'Push(E,E)': 0,
                        'Push(S,S)': 270,
                        'Push(W,W)': 180}
        
        self.direction = 0

    def forward(self,distance,block):
        """
        Commands the robot to move forward a given distance in meters. 

        Parameters
        ----------
        'distance' : float
            The distance to move forward in meters.
        'block' : bool
            If true, the robot will wait until the motion is completed before continuing with the next command.
            If false, the robot will continue immediately. 
        """
        forward_cmd = {
            'type': 'forward',
            'distance': float(distance),
            'block': block
        }
        message = msgpack.packb(forward_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024).decode() 
    
    def say(self, s):
        """
        Commands the robot to speak out the given sentence.

        The speech is generated using the onboard text-to-speech synthesis.
        Its intonation can sometimes be a bit strange. It is often possible to improve the
        understandability of the speech by inserting small breaks a key location in the sentence.
        This can be accomplished by inserting \\pau=$MS\\ commands, where $MS is the length of the
        break in milliseconds, e.g., robot.say("Hello \\pau=500\\ world!") will cause the robot to
        pause for 500 milliseconds before continuing with the sentence.

        Parameters
        ----------
        'sentence' : string
            The sentence to be spoken out loud.
        """
        say_cmd = {
            'type': 'say',
            'sentence': s
        }
        message = msgpack.packb(say_cmd, use_bin_type=True)
        self.client_socket.send(message)  # send message
        data = self.client_socket.recv(1024)

    def turn(self,angle,block):
        """
        Commands the robot to turn around its vertical axis.

        The position of the robot will remain approximately constant during the motion.
        Expect that the actually turned angle will vary a few degrees from the commanded values.
        The speed of the motion will be determined dynamically, i.e., the further it has to turn,
        the faster it will move.

        Parameters
        ----------
        'theta' : float
            The angle to turn in radians in the counter-clockwise direction.
        """
        turn_cmd = {
            'type': 'turn',
            'angle': float(angle),
            'block': block
        }
        message = msgpack.packb(turn_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024)

    
    def stand(self):
        '''
        Commands the robot to stand up in a straight position.

        '''

        stand_cmd = {
            'type': 'stand'
        }
        message = msgpack.packb(stand_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024)

    def shutdown(self):
        ''''
        Shuts down the robot.

        '''	
        shutdown_cmd = {
            'type': 'shutdown'
        }
        message = msgpack.packb(shutdown_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024)

    def move(self,x,y,theta,block):
        '''
        Commands the robot to move to a given position and orientation. Three degrees of freedom are specified: 
        the x and y coordinates of the position and the orientation theta. The position is specified in meters
        relative to the robot's initial position. The orientation is specified in radians relative to the robot's
        initial orientation. 

        Parameters
        ----------
        'x' : float
            The x coordinate of the position in meters.
        'y' : float
            The y coordinate of the position in meters.
        'theta' : float
            The orientation in radians.
        'block' : bool
            If true, the function will block until the robot has reached the target position.
        '''
        move_cmd = { 
            'type': 'move',
            'x': float(x),
            'y': float(y),
            'theta': float(theta),
            'block': block
        }
        message = msgpack.packb(move_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024)

    def close(self):
        '''
        Closes the connection to the robot.
        '''
        if self.client_socket:
            self.client_socket.close()

    def onLeds(self):
        '''
        Commands the robot to turn on the LEDs on its head.
        '''
        onLeds_cmd = {
            'type': 'onLeds'
        }
        message = msgpack.packb(onLeds_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024)
    
    def offLeds(self):
        '''
        Commands the robot to turn off the LEDs on its head.
        '''
        offLeds_cmd = {
            'type': 'offLeds'
        }
        message = msgpack.packb(offLeds_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024)
    
    def declare_direction(self, move):
        '''
        Commands the robot to say the direction it is moving in.

        Parameters
        ----------
        'move' : string
            The direction the robot is moving in.
        
        Returns
        -------
            Makes the robot say the direction it is moving in.
        '''
        direction = {'Move(N)': "I am going North",
                     'Move(E)': 'I am going East',
                     'Move(S)': 'I am going South',
                     'Move(W)': 'I am going West',
                     'Push(N,N)': 'I am pushing North',
                     'Push(E,E)': 'I am pushing East',
                     'Push(S,S)': 'I am pushing South',
                     'Push(W,W)': 'I am pushing West'}
        return self.say(direction[move])
    
    def listen(self, duration=3, channels=[0,0,1,0],playback=False):

        '''
        Commands the robot to listen for a given duration.

        Parameters:
        -----------
        'duration' : int
            The duration of the listening in seconds.
        'channels' : list
            The channels to listen on. The default is [0,0,1,0] which 
            means that the robot will listen on the front microphone. 
            You can also listen on other channels by changing the list.
        playback : bool
            If true, the robot will play back the audio it has recorded.
        
        Returns:
        --------
            The audio data recorded by the robot is saved in a folder
            /tmp/ which needs to exist on the given computer. This can 
            then used for speech recognition using Whisper.

        '''

        listen_cmd = {
            'type': 'listen',
            'duration': duration,
            'channels': channels,
            'playback': playback
        }
        message = msgpack.packb(listen_cmd, use_bin_type=True)
        self.client_socket.send(message)
        data = self.client_socket.recv(1024)

        # animations = [
        #     "animations/Stand/Gestures/Thinking_1",
        #     "animations/Stand/Gestures/Thinking_3",
        #     "animations/Stand/Gestures/Thinking_4",
        #     "animations/Stand/Gestures/Thinking_6",
        #     "animations/Stand/Gestures/Thinking_8",
        # ]

        # # når robotten starter med at lytte
        # robot.player.playFile("/opt/aldebaran/share/naoqi/wav/begin_reco.wav")
        # robot.leds.fadeRGB("FaceLeds", 0.0, 1.0, 0.0, 0.4)
        # robot.behavior.startBehavior(random.choice(animations))

        # # når robotten stopper med at lytte
        # robot.player.playFile("/opt/aldebaran/share/naoqi/wav/end_reco.wav")
        # robot.leds.fadeRGB("FaceLeds", 0.0, 0.0, 1.0, 0.4)

    
    
class Move_Robot:
    def __init__(self, direction, move_distance=0.5):
         self.direction = direction
         self.direction_mapping = { 'Move(N)': 90,
                        'Move(E)': 0,
                        'Move(S)': 270,
                        'Move(W)': 180,
                        'Push(N,N)': 90,
                        'Push(E,E)': 0,
                        'Push(S,S)': 270,
                        'Push(W,W)': 180}
         self.move_distance = move_distance

    def get_rotaiton(self, end_direction):
    
        to_rotate = end_direction - self.direction
        if to_rotate > 180:
            to_rotate = to_rotate - 360
        elif to_rotate < -180:
            to_rotate = to_rotate + 360
        
        # update direction
        self.direction += to_rotate
        self.direction = self.direction % 360

        return to_rotate
    
    def speack_action(self,action,robot):
        if action == 'None':
            pass
        else:
            robot.say(action)

    def execute(self,action,robot):
        if action == 'None':
            pass
        else:
            end_direction = self.direction_mapping[action]
            rotation = self.get_rotaiton(end_direction)
            self.speack_action(action,robot)
            robot.turn(degrees(rotation),block=True)
            robot.forward(self.move_distance,block=True)
    





if __name__ == '__main__':
    # get the ip address of the robot
    ip = sys.argv[1]

    # connect to the server and robot
    robot = RobotClient(ip)
    mover = Move_Robot(90)

    # test the robots speech
    robot.stand()

    #robot.say('I am executing plan. Please watch out!')
    robot.say('I am connected!')

    # test the robots listening
    #robot.listen(3, playback=True)
    plan = [
        'Move(N)',
        'Move(W)',
        'Move(N)',
        'Push(E,E)',
        'Move(S)',
        'Move(S)',
    ]

    for action in plan:
        time.sleep(1)
        mover.execute(action,robot)

    robot.say("Yes i just solved your level")

    # shutdown the robot
    robot.shutdown()
