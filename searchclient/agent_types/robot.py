
#%%
# coding: utf-8
#
# Copyright 2021 The Technical University of Denmark
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    runs the searchclient with the 'robot' agent type on the robot at IP 192.168.0.102. See a
    list of robot IPs at the github page for the course.
  - To connect to the robots, connect to the Pepper hotspot. To reduce
    the load on the hotspot, please disconnect between your sessions.
    
  - A good starting point is using something similar to the 'classic' agent type and then
    replacing it with calls to the 'robot' interface.
"""
from utils import *
from robot_interface import *
from domains.hospital.actions import *
from search_algorithms.graph_search import graph_search
import time



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
            end_direction = self.direction_mapping[action.name]
            rotation = self.get_rotaiton(end_direction)
            robot.declare_action(action.name)
            robot.turn(degrees(rotation),block=True)
            robot.forward(self.move_distance,block=True)

def get_command(robot):

    text = robot.listen(duration=10)
    text = text.lower()

    if "break" in text:
        return True

    if "error" in text:
        robot.say("Restate your command")
        return get_command(robot)

    if "move" in text:
        if "north" in text:
            return 
        elif "south" in text:
            return
        elif "east" in text:
            return
        elif "west" in text:
            return
        else:
            robot.say("Move Command is missing a direction. Please try again.")
            return get_command(robot)
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
            robot.say("Push Command is missing a direction. Please try again.")
            return get_command(robot)
    
    # State possible subgoals
    elif "subgoal" in text:
        # TODO: Implement subgoal
        # TODO: Speak reacheable subgoals
        return
    else:
        robot.say("Command not recognized. Please try again.")
        return get_command(robot)


    
    
def robot_agent_type(level, initial_state, action_library, goal_description, frontier, robot_ip):

    # Write your robot agent type here!

    # Solve the problem using the graph search algorithm
    action_set = [action_library] * level.num_agents

    planning_success, plan = graph_search(initial_state, action_set, goal_description, frontier)

    if not planning_success:
        print("Unable to solve level.", file=sys.stderr)
        return
    
    print(f"Found solution of length {len(plan)}", file=sys.stderr)

    robot = RobotClient(robot_ip)
    mover = Move_Robot(90)

    # Define the robot client and the move robot class
    Command_mode = True
    while Command_mode:

        # Get the command from the user
        robot.say("Please give me a command")
        command = get_command(robot)
        

        # Break out of the loop if the command is true
        if command == True:
            Command_mode = False
        else:
            mover.execute(command,robot)
        


    

    if True:
        print(plan)
    else:
        # Test out the robots microphone. The server will let you know when the robot is listening.
        robot = RobotClient(robot_ip)
        mover = Move_Robot(90)
        robot.listen(3, playback=True)

        # test the robots speech
        robot.stand()

        # The robot will announce that it is executing the plan
        robot.say('I am executing plan. Please watch out!')

        # Implement your solution here!

        plan = ROBOT_ACTION_LIBRARY
        for action in plan:
            time.sleep(1)
            mover.execute(action,robot)
        

        # Wait until the robot is done speaking
        time.sleep(3)

        # close the connection
        robot.close()
