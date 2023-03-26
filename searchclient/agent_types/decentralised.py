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

from search_algorithms.graph_search import graph_search
from utils import *


def decentralised_agent_type(level, initial_state, action_library, goal_description, frontier):
    # Create an action set where all agents can perform all actions
    action_set = [action_library] * level.num_agents

    # Here you should implement the DECENTRALISED-AGENTS algorithm.
    # You can use the 'classic' agent type as a starting point for how to communicate with the server, i.e.
    # use 'print(joint_action_to_string(joint_action), flush=True)' to send a joint_action to the server and
    # use 'parse_response(read_line())' to read back an array of booleans indicating whether each individual action
    #   in the joint action succeeded.
    n_agtenst = level.num_agents

    pi = []
    succes = []
    for idx in range(n_agtenst):
        # Find the color of the agent
        agent_char = initial_state.agent_positions[idx][1]
        agent_color = level.colors[agent_char]

        # Create a monochrome problem
        monochrome_problem = initial_state.color_filter(agent_color)
        monochrome_goal_description = goal_description.color_filter(agent_color)

        # Run graph search
        plan_succes , plan = graph_search(monochrome_problem, [action_library], monochrome_goal_description, frontier)
        pi.append(plan)
        succes.append(plan_succes)

    if not all(succes):
        print("Unable to solve level", file=sys.stderr)
        return
    
    print(pi,file=sys.stderr)
    # Using all individual plans, create a joint plan
    while not all([len(pi_i) == 0 for pi_i in pi]):

        # Initlize joint action
        joint_action = []

        # Get each Agent Action
        for idx in range(n_agtenst):
            # If the plan is empty, add a no-op else add the next action
            if len(pi[idx]) == 0:
                joint_action.append(action_library[0])
            else:
                joint_action.append(pi[idx][0][0])

        # Send joint action to server
        print(joint_action_to_string(joint_action), flush=True)

        # Get response from server
        response = parse_response(read_line())

        # Check if all actions were succesful
        if any(response):
            for idx in range(n_agtenst):
                if response[idx] and len(pi[idx]) > 0:
                    # Execute action by removing it from the plan
                    pi[idx].pop(0)

        # If all actions failed, print error and return
        if not any(response):
            print("Unable to solve level", file=sys.stderr)
            print(pi,file=sys.stderr)
            return
        
        # If all actions except no-ops failed, print error and return
        not_finished = sum(len(pi_i) != 0 for pi_i in pi)

        for idx in range(n_agtenst):
            if not response[idx] and len(pi[idx]) != 0:
                not_finished -= 1
            
        # if not_finished == 0 and :
        #     print("Unable to solve level", file=sys.stderr)
        #     print(pi,file=sys.stderr)
        #     return
