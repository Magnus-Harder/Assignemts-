
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
from search_algorithms.all_optimal_plans import all_optimal_plans
from agent_types.goal_recognition import goal_recognition_agent_type, DisjunctiveGoalDescription
import time

# Load the whisper model
import whisper
model = whisper.load_model('base')

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
            robot.declare_direction(action.name)
            robot.turn(degrees(rotation),block=True)
            robot.forward(self.move_distance,block=True)

 

    
def robot_agent_type(level, initial_state, action_library, goal_description, frontier, robot_ip):

    # Write your robot agent type here!

    # Solve the problem using the graph search algorithm
    action_set = [action_library] * level.num_agents
    action_dict = {action.name: action for action in action_library}

    # define the mover
    Mover = Move_Robot(90)

    def listen(robot):
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

        audio = robot.listen(5)

        # # når robotten stopper med at lytte
        # robot.player.playFile("/opt/aldebaran/share/naoqi/wav/end_reco.wav")
        # robot.leds.fadeRGB("FaceLeds", 0.0, 0.0, 1.0, 0.4)

        text = model.transcribe("tmp/test.wav")['text']

        return text.lower()

    def get_command(robot):

        robot.say("I am listeing")
        time.sleep(1.5)

        text = listen(robot)

        if "break" in text:
            robot.say("Stopping Command Mode")
            return True

        if "error" in text:
            robot.say("Restate your command")
            return get_command(robot)

        if "move" in text:
            if "north" in text:
                action = "Move(N)"
                return action
            elif "south" in text:
                action = "Move(S)"
                return action
            elif "east" in text:
                action = "Move(E)"
                return action
            elif "west" in text:
                action = "Move(W)"
                return action
            else:
                robot.say("Move Command is missing a direction. Please try again.")
                time.sleep(5)
                return get_command(robot)
        elif "push" in text:
            if "north" in text:
                action = "Push(N,N)"
                return action
            elif "south" in text:
                action = "Push(S,S)"
                return action
            elif "east" in text:
                action = "Push(E,E)"
                return action
            elif "west" in text:
                action = "Push(W,W)"
                return action
            else:
                robot.say("Push Command is missing a direction. Please try again.")
                time.sleep(5)
                return get_command(robot)
        
        # State possible subgoals
        elif "sub" in text or "goal" in text:
            return "subgoal"
        else:
            robot.say("Command not recognized. Please try again.")
            time.sleep(5)
            return get_command(robot)

    def interpret_action(robot):
        
        robot.say("I am listeing")

        text = listen(robot)
        
        if "move" in text:
            if "north" in text:
                action = "Move(N)"
                return action
            elif "south" in text:
                action = "Move(S)"
                return action
            elif "east" in text:
                action = "Move(E)"
                return action
            elif "west" in text:
                action = "Move(W)"
                return action
            else:
                robot.say("Move Command is missing a direction. Please try again.")
                return interpret_action(robot)
        elif "push" in text:
            if "north" in text:
                action = "Push(N,N)"
                return action
            elif "south" in text:
                action = "Push(S,S)"
                return action
            elif "east" in text:
                action = "Push(E,E)"
                return action
            elif "west" in text:
                action = "Push(W,W)"
                return action
            else:
                robot.say("Push Command is missing a direction. Please try again.")
                return interpret_action(robot)

        robot.say("Command not recognized. Please try again.")
        return interpret_action(robot)

    def Helper_mode(robot):
        robot.say("Following your level i will search for a solution to help you!")

        succes, policy = goal_recognition_agent_type(initial_state, action_library, goal_description, frontier)

        possible_goals = [goal_description.get_sub_goal(index) for index in range(goal_description.num_sub_goals())]

        goals = DisjunctiveGoalDescription(possible_goals)

        state = initial_state

        if succes is False:
            robot.say("I am unable to help you. Solution strategy not found.")
        else:
            while len(possible_goals) > 0:
            
                robot.say("I am ready to help you. I will follow Your Moves")

                while not any(pos.is_goal(state) for pos in possible_goals):
                    robot.say("Declare your action")
                    
                    agent_action = action_dict[interpret_action(robot)]
                    helper_action = policy[state]

                    if state.is_applicable((agent_action,helper_action)) is False:
                        robot.say("This action is not applicable. Please try again.")
                        continue

                    joint_action = (agent_action, helper_action)

                    robot.say("I will execute my the action")
                    Mover.execute(helper_action,robot)


                    state = state.result(joint_action)

                
                robot.say("We have reached a subgoal. I will now search for a new solution")
                possible_goals = [pos for pos in possible_goals if not pos.is_goal(state)]

                if len(possible_goals) == 0:
                    robot.say("We have reached the final goal. Congratulations!")
                    return None
                else:

                    policy = goal_recognition_agent_type(state, action_library, None, frontier, sub_goals=possible_goals)
                    if policy is False:
                        robot.say("I am unable to further help you. Solution strategy not found.")
                        return None
                    else:
                        robot.say("I have found a new solution. Let's continue")
                        continue

    
    def subgoal_mode(robot):
        #State subgoals
        possible_subgoals = [goal_description.get_sub_goal(index) for index in range(goal_description.num_sub_goals())]
        solved,solution_graph = all_optimal_plans(initial_state, action_set, possible_subgoals, frontier,debug=True)
        
        if not solved:
            print("Unable to solve level.", file=sys.stderr)
            return Command_mode(robot)
        
        sub_goals = solution_graph.consistent_goals

        
        robot.say(f"I have found the following {len(sub_goals)} subgoals")

        sub_goals_key = {}

        for sub_goal in sub_goals:
            sub_goals_key[sub_goal[1]] = sub_goal
            robot.say(f"Box {sub_goal[1]} moved to position {sub_goal[0]}" )

      
        #get input 
        sub_goal = None
        while True:
            robot.say("Please choose a subgoal")
            robot.say("I am listeing")
            time.sleep(1.5)
            text = listen(robot)

            for key in sub_goals_key.keys():
                if key in text:
                    sub_goal = sub_goals_key[key]
                    robot.say(f"Executing subgoal {key}")

            if sub_goal is None:
                robot.say("Subgoal not recognized. Please try again.") 
            else:
                break         

        #output action sequence
        while True:
            if len(solution_graph.get_actions_and_results_consistent_with_goal(sub_goal)) == 0:
                robot.say("Subgoal Reached")
                break
            action, solution_graph = solution_graph.get_actions_and_results_consistent_with_goal(sub_goal)[0]


            Mover.execute(action,robot)

        return None


        

        

    def Command_mode(robot):

        robot.say("Entering Command mode loop")
        stop = False
        while not stop:
            action = get_command(robot)

            # Break out of the loop if the command is true
            if action is True:
                stop = True
            if action == "subgoal":
                subgoal_mode(robot)
            else:
                # declare the action and Move
                action = action_dict[action]
                Mover.execute(action,robot)

        return None


    # Define the solve mode
    def solve_mode(robot):
    
        planning_success, plan = graph_search(initial_state, action_set, goal_description, frontier)

        if not planning_success:
            print("Unable to solve level.", file=sys.stderr)
            return
        
        print(f"Found solution of length {len(plan)}", file=sys.stderr)
        print(plan)

        # Execute the plan
        for action in plan[0]:

            # declare the action
            Mover.execute(action,robot)
        
        robot.say("I have reached the goal")

        return None
    
    # Interpret the mode to enter
    def Interpret_mode(robot):
        
        robot.say("Please state wich mode to run!")
        time.sleep(2)

        text = listen(robot)

        robot.say(text)

        if "helper" in text:
            Mode = "Helper"
            Helper_mode(robot)
            robot.say("Exiting Helper Mode")
            Interpret_mode(robot)
        elif "command" in text:
            Mode = "Command"
            Command_mode(robot)
            robot.say("Exiting Command Mode")
            Interpret_mode(robot)
        elif "solve" in text:
            Mode = "solve"
            solve_mode(robot)
            robot.say("Exiting Solve Mode")
            Interpret_mode(robot)
        elif "quit" in text or "exit" in text:
            robot.say("Exiting Program")
            return None 
        else:
            robot.say("Mode not recognized. Please try again.")
            time.sleep(10)
            return Interpret_mode(robot) 




    # Define the robot client
    robot = RobotClient(robot_ip)
    


    Mode = Interpret_mode(robot)


        


    # shutdown the robot
    robot.say("Shutting down")
    robot.close()
    time.sleep(1.5)


