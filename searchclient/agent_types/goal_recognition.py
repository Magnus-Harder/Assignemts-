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

import random
from search_algorithms.all_optimal_plans import all_optimal_plans,MultiParentNode
from search_algorithms.and_or_graph_search import and_or_graph_search
#from searchclient.domains.hospital.actions import *
from copy import deepcopy
from utils import *
from time import time


actor_AGENT = 0
HELPER_AGENT = 1


class DisjunctiveGoalDescription:
    """
    DisjunctiveGoalDescription is a wrapper class which allow for the representation of multiple possible
    goal descriptions. It has the same 'is_goal' method as a GoalDescription object and can therefore be
    used in the same places, but in contrast to the regular GoalDescription object, it returns True when
    one of its give goals are satisfied, thus allowing for a logical 'OR' to be expressed.
    """

    def __init__(self, possible_goal_descriptions):
        # possible_goal_descriptions should be a list of goals
        self.possible_goals = possible_goal_descriptions

    def is_goal(self, state):
        for possible_goal in self.possible_goals:
            if possible_goal.is_goal(state):
                return True
        return False


class GoalRecognitionNode:
    """
    GoalRecognitionNode is a wrapper class which can be used for implementing AND-OR based graph search.
    It allow a hospital state object and a solution graph object to be integrated into a single object, which
    the methods 'get_applicable_actions' and 'result' as required by the AND-OR graph search.
    Note that the usage of this class is completely optional and you are free to implement your goal recognition
    in a different manner, if you so desire.

    """

    def __init__(self, state, solution_graph):
        self.state = state
        self.solution_graph = solution_graph

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.state == other.state and self.solution_graph == other.solution_graph
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.state, self.solution_graph))

    def get_applicable_actions(self, action_set):
        # Here we are only interested in the actions of the helper, but state.get_applicable_actions will return a list
        # of joint actions, where the actor action is always GenericNoOp().
        applicable_joint_actions = self.state.get_applicable_actions(action_set)
        applicable_actions = [joint_action[HELPER_AGENT] for joint_action in applicable_joint_actions]
        return applicable_actions

    def result(self, joint_action):

        # The result method should return a new GoalRecognitionNode which contains the resulting state and the
        # solution graph obtained from executing the joint_action in the current state.
        raise NotImplementedError()


def solution_graph_results(recognition_node, helper_action):

    # This results method can be used as the 'results' function for the AND-OR graph-search.
    # It takes a GoalRecognitionNode (or something else if you choose to not use the GoalRecognitionNode class) and
    # the action taken by the helper, i.e., the chosen OR-branch.
    # This function should then return all of the possible outcomes, i.e., the possible AND-nodes.
    raise NotImplementedError()

def remove_identical_pairs(lst):
    unique_pairs = set()
    result = []

    for pair in lst:
        pair_tuple = tuple(pair)
        if pair_tuple not in unique_pairs:
            unique_pairs.add(pair_tuple)
            result.append(pair)

    return result

def results_goalrec(state, helper_action,state_2_node,coolor_actor):
    # Building the Results() function containing the indeterminism
    # If performing two of the same actions is possible from the state,
    # this result is added as a possible outcome..
    
    
    state_mono = state.color_filter(coolor_actor)
    node = state_2_node[state_mono]
    
    goals = node.consistent_goals
    succesful_actions = []
    for goal in goals:
        actor_actions = node.get_actions_and_results_consistent_with_goal(goal)
        #print("GOOOOOOOOODIIIIIEEEEESSSS",file=sys.stderr)
        #print(actor_actions,file=sys.stderr)
        #print(type(actor_actions),file=sys.stderr)
        #print(helper_action,file=sys.stderr)
        #print(type(helper_action),file=sys.stderr)
        pos_opt_dirs = [dir[0] for dir in actor_actions]
        #print(pos_opt_dirs,file=sys.stderr)
        for action in pos_opt_dirs:
            if state.is_applicable([action,helper_action]):
                succesful_actions.append([action,helper_action])
            else:
                succesful_actions.append([GenericNoOp(),GenericNoOp()])
                
        final_succesfulactions =  remove_identical_pairs(succesful_actions)
        #print(final_succesfulactions,file=sys.stderr)
        #print("BOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOIIIIIIIIIIIIIIIIIIIIIIIIIII",file=sys.stderr)
        #print(state.result([GenericNoOp(),GenericNoOp()]),file=sys.stderr)
        
        states = [state.result(actiones) for actiones in final_succesfulactions]
        
        return states
        #return([state.result(actiones) for actiones in final_succesfulactions])
        #print(succesful_actions,file=sys.stderr)
        #print(list(set(succesful_actions)),file=sys.stderr)
        #return [state.results(sta) for sta in list(set(succesful_actions))]
    
    """
    standard_case = state.result(helper_action)
    if standard_case.is_applicable(helper_action):
        broken_case = standard_case.result(helper_action)
        return [standard_case, broken_case]
    else:
        return [standard_case]"""

def and_or_graph_search_helper(initial_state, action_set, goal_description, results, dic_state2node: dict,actor_colorr):
    #print("ACTION SET: ", action_set,file=sys.stderr)
    def Or_search(state,path,depth):
        
        #print(state,file=sys.stderr)
        #print(goal_description.is_goal(state),file=sys.stderr)
        if goal_description.is_goal(state):
            return {}

        # Check if state is in path
        if state in path: # Loop
            if cyclic:
                return "Loop"
            else:
                return False
        
        # Check if depth is reached
        if depth == 0:
            return False

        # loop over all actions
        #print("APPLC: ", state.get_applicable_actions(action_set),file=sys.stderr)
        attempted_actions = []
        #print(state)
        
        for action in deepcopy(state).get_applicable_actions(action_set):
            #print(action)
            #print("ACTION IS " + str(action),file=sys.stderr)
            if action[1] in attempted_actions:
                continue
            attempted_actions.append(action[1])
            # Note Plan is a policy
            path_for_plan = deepcopy(path) 
            path_for_plan.append(deepcopy(state))

            # Find the next states
            new_states = results(deepcopy(state),action[1],dic_state2node,actor_colorr)
            
            # Get plan from And_search
            plan = And_search(new_states, path_for_plan, depth)

            # See if plan is succesfull
            if plan != False:
                # Add action to plan and return
                plan[deepcopy(state)] = action[0]
                return plan  
        
        # If no plan is found return false
        return False


    def And_search(states,path,depth):
        new_depth = depth - 1 # Update depth for next Or-search call
        
        plan = {}
        planis = [Or_search(deepcopy(state),path,new_depth) for state in states]
        
        #Added cyclic case
        if cyclic:
            # Ensure that not all plans are loop and that no plan is False
            if any(p == False for p in planis) or all(p == "Loop" for p in planis):
                return False
            else:
                for p in planis:
                    if p != "Loop": #Can't update the plan if loop
                        plan.update(p)

        else:
            # Where was return failiure if any "OR-leaf" is not goal node???
            if all(p != False for p in planis):
                for p in planis:
                    # Add succsesfull plans
                    plan.update(p)
            else:
                return False
        
        # Return plan
        return plan
    cyclic = False

    # Can now go to next part with propper fail return
    mulipl = 1
    start_t = time()
    ## Iterative deepening search missing from prev. implementation
    depth_budget = int(20) ## Should be inf, but don't want to waste time with unreasonable searches
    for d in range(depth_budget):
        if d%mulipl == 0:
            print(f"Searching depth of [{d}] after {int(time()-start_t)}s", file=sys.stderr)
        
        plan = Or_search(initial_state,[],d) # Find plan or fail at curr depth
        
        if plan != False:
            # Return Policy and worst case length
            for k, v in plan.items():
                print(k,v,"\n",file=sys.stderr)
            return d, plan
    
    if d+1 >= depth_budget:
        print(f"\nDepth budget of {depth_budget} reached witout cutoff!!" ,file=sys.stderr)
        if not cyclic:
            print(f"\nNo non-looping path to goal found\n" ,file=sys.stderr)
        return 0, {}
    else:
        print("This should be unreachable:&")
        raise NotImplementedError("idfk")


def goal_recognition_agent_type(level, initial_state, action_library, goal_description, frontier): #Why is there a fronteir?
    
    #STEPS
    
    action_set = [action_library,action_library]
    all_goals_reached = False
    actor_color = level.colors[str(actor_AGENT)]
    goal_mono = goal_description.color_filter(actor_color)
    initial_state_mono = initial_state.color_filter(actor_color)
    possible_goals = [] #List of sub-goals for the actor
    for index in range(goal_mono.num_sub_goals()):
        sub_goal = goal_mono.get_sub_goal(index)
        possible_goals.append(sub_goal)
    
    goals = DisjunctiveGoalDescription(possible_goals)
    bool,Mult_par_n,state2node_dict = all_optimal_plans(initial_state_mono, action_set, possible_goals, frontier,
                          debug=False,ret_statdic = True)
    
    
    worst_case_length, plan = and_or_graph_search_helper(initial_state, action_set, goals, results_goalrec, state2node_dict,actor_colorr = actor_color)
    print(plan,file=sys.stderr)
    """
    while all_goals_reached != True:
        cur_goal = random.choice(possible_goals)
        possible_goals.remove(cur_goal)
        
        #Agent chooses an action
        possible_actions = Mult_par_n.get_actions_and_results_consistent_with_goal(cur_goal)
        action,node = random.choice(possible_actions)
        #helper_plan = and_or_graph_search_helper(initial_state, action_set, goal_description, results, cur_root_node: MultiParentNode, dic_state2node: dict)
        print(action,file=sys.stderr)
        box_not_arrived = True
        
        #print(joint_action_to_string([action,"NoOp"]]), flush=True)
        
        #Helper finds plan
        
        
        #Plan execution
        while box_not_arrived:
            #print("nothing",file=sys.stderr)
            
            #HELPER CHOOSES AN ACTION
            helper_choice = GenericNoOp()
            
            
            print(str(action) + "|" + str(helper_choice),flush=True)
            _ = parse_response(read_line())
            
            #ACTOR CHOOSES AN ACTION
            choices = node.get_actions_and_results_consistent_with_goal(cur_goal)
            if len(choices) != 0:
                action,node = random.choice(choices)
            else:
                break
            #box_not_arrived = False

            
            

    
    
    
    
    
    
    #print([bool,Mult_par_n], file=sys.stderr)
    
    #and_or_graph_search(initial_state, action_set, goal_description)
    #print(Mult_par_n.get_applicable_actions(action_set), file = sys.stderr)
    #print(Mult_par_n.get_actions_and_results_consistent_with_goal(possible_goals[0]),file = sys.stderr)

    #joint_action = [(Move(S), NoOp), (Move(E), Pull(E,N)), (Move(E), Push(W,W)), (Push(E,E), Push(W,N)), (Move(N), Move(W)), (Push(N,E), Move(N))]
    #print(joint_action_to_string(joint_action), flush=True)
    #print(("Move(S)|NoOp"), flush=True)
    #_ = parse_response(read_line())
    
    #print(("Move(E)|NoOp"), flush=True)
    #_ = parse_response(read_line())
    ## --------------- ##
    
    #raise NotImplementedError()
"""


