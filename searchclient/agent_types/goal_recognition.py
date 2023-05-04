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



def results_goalrec(state, helper_action, dic_state2node, coolor_actor):
    # results function for the AND-OR graph-search.
    
    # Initialize an empty list to store the resulting states.
    states = []

    # We get state mono purely such that we may use it as a key in the dictionary.
    state_mono = state.color_filter(coolor_actor)

    # Retrieve the corresponding node from dictionary contatining all nodes with states as keys
    node = dic_state2node[state_mono]

    # Loop through the optimal actions and their resulting states for the actor.
    for action, new_state in node.optimal_actions_and_results.items():
        # Create a new state 'state_2' by applying the helper_action while the actor performs a no-op.
        state_2 = state.result([GenericNoOp(), helper_action])

        # Check if the resulting state_2 is applicable for the actor to perform the action.
        if state_2.is_applicable([action, GenericNoOp()]):
            # If it is applicable, add the state resulting from both the actor and helper performing their actions.
            states.append(state.result([action, helper_action]))
        else:
            # If it is not applicable, append the original state without any changes.
            states.append(state)

    # Return the list of resulting states.
    return states

def and_or_graph_search_helper(initial_state, action_set, goal_description, results, dic_state2node: dict,actor_colorr):
    #print("ACTION SET: ", action_set,file=sys.stderr)
    def Or_search(state,path,depth):
        
        #check if goal is reached
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

        
        for action in deepcopy(state).get_applicable_actions([[GenericNoOp()], action_set[1]]):
            path_for_plan = deepcopy(path)
            path_for_plan.append(deepcopy(state))

            # Find the next states
            new_states = results(deepcopy(state),action[1], dic_state2node,coolor_actor = actor_colorr)
            
            # Get plan from And_search
            plan = And_search(new_states, path_for_plan, depth)

            # See if plan is succesfull
            if plan != False:
                # Add action to plan and return
                plan[deepcopy(state)] = action[1]
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
                    # Add succesfull plans
                    plan.update(p)
            else:
                return False
        
        # Return plan
        return plan
    cyclic = True

    # Can now go to next part with propper fail return
    mulipl = 1
    start_t = time()
    ## Iterative deepening search missing from prev. implementation
    depth_budget = int(10) ## Should be inf, but don't want to waste time with unreasonable searches
    for d in range(depth_budget):
        if d%mulipl == 0:
            print(f"Searching depth of [{d}] after {int(time()-start_t)}s", file=sys.stderr)
        
        plan = Or_search(initial_state,[],d) # Find plan or fail at curr depth
        
        if plan != False or False:
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


#Goal Recognition using AND-OR Graph Search and a helper
def goal_recognition_agent_type(level, initial_state, action_library, goal_description, frontier, sub_goals = None): #Why is there a fronteir?
    

    #Initializing values needed for all_optimal_plans
    action_set = [action_library,action_library]
    all_goals_reached = False
    actor_color = level.colors[str(actor_AGENT)]
    goal_mono = goal_description.color_filter(actor_color)
    initial_state_mono = initial_state.color_filter(actor_color)
    possible_goals = [] #List of sub-goals for the actor
    for index in range(goal_mono.num_sub_goals()):
        sub_goal = goal_mono.get_sub_goal(index)
        possible_goals.append(sub_goal)

    if sub_goals != None:
        possible_goals = sub_goals


    #Running all_optimal_plans
    goals = DisjunctiveGoalDescription(possible_goals)
    bool,Mult_par_n,state2node_dict = all_optimal_plans(initial_state_mono, action_set, possible_goals, frontier,
                          debug=False,ret_statdic = True)
    
    #Using values from all_optimal_plans to do and_or_graph_search
    worst_case_length, plan = and_or_graph_search_helper(initial_state, action_set, goals, results_goalrec, state2node_dict,actor_colorr = actor_color)
    
    #Setting the state so we can keep track of changes to the state
    current_state = initial_state

    return plan 
    
    ##-----------------
    ##--- MAIN LOOP ---
    ##-----------------
    goals_done = []
    while len(possible_goals) != 0:
        #Setting and removing a goal, such that we can cycle through all goals one at a time
        cur_goal = random.choice(possible_goals)
        possible_goals.remove(cur_goal)
        
        #Agent receives current possible actions
        possible_actions = Mult_par_n.get_actions_and_results_consistent_with_goal(cur_goal)
        print(possible_actions, file = sys.stderr)
        action,node = random.choice(possible_actions)
        
        ##Execute and or graph search with actor taking 'random' optimal path to chosen goal
        while True:
            #Helper chooses an action
            helper_choice = plan[current_state]
            
            #Helper registers move with server
            print(joint_action_to_string([GenericNoOp(),helper_choice]), flush=True)
            bool_suc = parse_response(read_line())

            #Actor registers move with server
            print(joint_action_to_string([action,GenericNoOp()]), flush=True)
            bool_suc = parse_response(read_line())

            #If actor action fails, actor tries again and state stays consistent with this lack of movement
            if bool_suc[1] == True:
                current_state = current_state.result([action,helper_choice])
                choices = node.get_actions_and_results_consistent_with_goal(cur_goal)
            else:
                current_state = current_state.result([GenericNoOp(),helper_choice])
            
            #If end goal reached, break loop and move on to next goal
            if len(choices) != 0:
                action,node = random.choice(choices)
            else:
                print("Done!",file=sys.stderr)
                break
        
        ##Getting new plan and policy for the current state, ignoring completed goals
        new_state_mono = current_state.color_filter(actor_color)
        goals_done.append(deepcopy(cur_goal.goals[0]))
        joint_goals = []
        for remaining_goal in possible_goals:
            new_olds = deepcopy(goals_done)
            new_olds.append(remaining_goal.goals[0])
            joint_goals.append(cur_goal.create_new_goal_description_of_same_type(new_olds))

        possible_goals = joint_goals #Fuck
        
        bool,Mult_par_n,state2node_dict = all_optimal_plans(new_state_mono, action_set, joint_goals,
                                                    frontier,debug=False,ret_statdic = True)
        
        if bool == False:
            print("tough luck kid, better luck next time", file = sys.stderr)
        # bool,Mult_par_n,state2node_dict = all_optimal_plans(new_state_mono, action_set, possible_goals,
        #                                                     frontier,debug=False,ret_statdic = True)
        worst_case_length, plan = and_or_graph_search_helper(current_state, action_set, goals, results_goalrec, state2node_dict,actor_colorr = actor_color)


            
            
            

"""
class GoalRecognitionNode: #NOT USED
    GoalRecognitionNode is a wrapper class which can be used for implementing AND-OR based graph search.
    It allow a hospital state object and a solution graph object to be integrated into a single object, which
    the methods 'get_applicable_actions' and 'result' as required by the AND-OR graph search.
    Note that the usage of this class is completely optional and you are free to implement your goal recognition
    in a different manner, if you so desire.


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


def solution_graph_results(recognition_node, helper_action): #NOT USED

    # This results method can be used as the 'results' function for the AND-OR graph-search.
    # It takes a GoalRecognitionNode (or something else if you choose to not use the GoalRecognitionNode class) and
    # the action taken by the helper, i.e., the chosen OR-branch.
    # This function should then return all of the possible outcomes, i.e., the possible AND-nodes.
    raise NotImplementedError()

"""