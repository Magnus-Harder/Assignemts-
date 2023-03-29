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

from domains.hospital import HospitalGoalDescription
from search_algorithms.graph_search import graph_search
from utils import *




def helper_agent_type(level, initial_state, action_library, goal_description, frontier):

    # Here you should implement the HELPER-AGENT algorithm.
    # Some tips are:
    # - From goal_description, you should look into color_filter and get_sub_goal to create monochrome and subgoal problems.
    # - You should handle communication with the server yourself and check successes of joint actions.
    #   Look into classic.py to see how this is done.
    # - You can create an action set where only a specific agent is allowed to move as follows:
    #   action_set = [[GenericNoOp()]] * level.num_agents
    #   action_set[agent_index] = action_library
    # - You probably want to create a helper function for creating the set of negative obstacle subgoals.
    #   You can then create a new goal description using 'goal_description.create_new_goal_description_of_same_type'
    #   which takes a list of subgoals.
    
    def get_agent0_pos(state_in):
        for pos in state_in.agent_positions:
            if pos[1] == agent:
                return pos[0]

    def get_helper_goal(color,positions_path):
        c_state = initial_state.color_filter(color)
        helper_agent = c_state.agent_positions[0][1]
        helper_box = c_state.box_positions[0][1]
        goal_list = []
        for pos_list in positions_path:
            for pos in pos_list: #It comes in a list of pos tuples because it can move the box also
                goal_list.append((pos,helper_agent,False))
                goal_list.append((pos,helper_box,False))
        
        return goal_description.create_new_goal_description_of_same_type(goal_list), helper_agent
            
    total_generated = 0 #For counting states across all searches
    
    num_agents = level.num_agents

    agent = "0" #The actor in the simplified domain
    agent_color = level.colors[agent]
    
    action_set = [[GenericNoOp()]] * num_agents
    action_set[0] = action_library
    
    # Createin a monochrome problem
    monochrome_problem = initial_state.color_filter(agent_color)
    monochrome_goal_description = goal_description.color_filter(agent_color)
    
    plan_succes , plan, runinfo = graph_search(monochrome_problem, action_set, monochrome_goal_description, frontier, info_dict=True)
    assert plan_succes, f"Agent 0 not able to solve monochrome problem!!"
    total_generated += runinfo["Generated"]
    
    path0 = []
    path0old = []
    state_loop = monochrome_problem
    for action in plan:
        pos_list, _ = action[0].conflicts(0,state_loop)
        state_loop = state_loop.result(action)
        path0.append(pos_list)
        path0old.append(get_agent0_pos(state_loop))
    
    #Debugging!
    # print(f"--PLAN--\n", plan,"\n", file=sys.stderr)
    # print(f"--PATH--\n", path0, path0old,"\n", file=sys.stderr)

    pi = []
    for p in range(len(plan)):
        naive_joint_action = [GenericNoOp()] * level.num_agents
        naive_joint_action[0] = plan[p][0]
        pi.append(naive_joint_action)
    # Exetuting plan
    p = 0
    while(len(pi) != p):
        print(joint_action_to_string(pi[p]), flush=True)
        
        line = read_line()
        execution_successes = parse_response(line)

        if execution_successes[0] == False:
            # Agent 0 is stuck now
            remaining_plan = pi[p:]
            remaining_path = path0[p:]
            curr_state = initial_state.result_of_plan(pi[:p])

            # print(f"--pos--\n", get_agent0_pos(curr_state),"\n", file=sys.stderr)
            # print(f"--REMAINING PLAN--\n", remaining_plan,"\n", file=sys.stderr)
            # print(f"--REMAINING POS--\n", remaining_path,"\n", file=sys.stderr)
            
            action_failed = pi[p][0]
            occupied, _ = action_failed.conflicts(0,curr_state) #ASSUMING AGENT O INDEX IS 0 (for now)
            for occ_pos in occupied:
                blocking_obj = curr_state.object_at(occ_pos)
            assert blocking_obj != "", "There is no object blocking but plan failed!"
            blocking_color = level.colors[blocking_obj]
            #print(f"--Helper Color--\n", blocking_color,"\n", file=sys.stderr)
            
            helper_goal, helper_agent = get_helper_goal(blocking_color,remaining_path)
            
            helper_action_set = [[GenericNoOp()]] * num_agents
            helper_action_set[int(helper_agent)] = action_library

            h_success, h_plan, h_runinfo = graph_search(curr_state, helper_action_set, helper_goal, frontier, info_dict=True)
            total_generated += h_runinfo["Generated"]
            assert h_success, f"Helper {helper_agent} cannot solve problem"

            new_pi = pi[:p] + h_plan + pi[p:]
            pi = new_pi
            #print(f"--HEPLER PLAN--\n", h_success,h_plan, helper_goal,"\n", file=sys.stderr)
        else:
            p += 1
    
    print(f"--Total Generated--\n # ", total_generated," #\n", file=sys.stderr)

    
    
    
    
        
        
    
