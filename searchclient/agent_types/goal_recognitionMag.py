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

import sys
sys.setrecursionlimit(10000)

import random
from search_algorithms.all_optimal_plans import all_optimal_plans
from search_algorithms.and_or_graph_search import and_or_graph_search
from utils import *
from domains.hospital.actions import *

from copy import deepcopy
actor_AGENT = 0
HELPER_AGENT = 1

NoOp = NoOpAction()

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

    def is_goal(self, belief_node):
        for possible_goal in self.possible_goals:
            if possible_goal.is_goal(belief_node.state):
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

    def __init__(self, state, solution_graph,NoOp):
        self.state = state
        self.solution_graph = solution_graph
        self.NoOp = NoOp

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

        # cheack if the applicability of the joint action is correct
        if not self.state.is_applicable(joint_action):
            # agents action is not applicable agent does nothing
            new_state = self.state.result((self.NoOp,joint_action[HELPER_AGENT]))

            return GoalRecognitionNode(new_state, self.solution_graph, self.NoOp)
        # Check if the joint action is conflicting
        if self.state.is_conflicting(joint_action):
            return GoalRecognitionNode(self.state, self.solution_graph,self.NoOp)

        # Get the new state and solution graph
        new_state = self.state.result(joint_action)
        new_solution_graph = self.solution_graph.optimal_actions_and_results[joint_action[actor_AGENT]]

        return GoalRecognitionNode(new_state, new_solution_graph,self.NoOp)


        


def solution_graph_results(recognition_node, helper_action):

    # This results method can be used as the 'results' function for the AND-OR graph-search.
    # It takes a GoalRecognitionNode (or something else if you choose to not use the GoalRecognitionNode class) and
    # the action taken by the helper, i.e., the chosen OR-branch.
    # This function should then return all of the possible outcomes, i.e., the possible AND-nodes.
    
    # Get possible percepts for the actor assuming optimal actions
    possible_percepts = recognition_node.solution_graph.optimal_actions_and_results.keys()

    
    # Make sure all join actions are applicable
    results = [recognition_node.result((percepts,helper_action)) for percepts in possible_percepts]

    return results





def goal_recognition_agent_type(level, initial_state, action_library, goal_description, frontier):
    # You should implement your goal recognition agent type here. You can take inspiration on how to structure the code
    # from your previous helper and non deterministic agent types.
    # Note: Similarly to the non deterministic agent type, this is not a fast algorithm and you should therefore start
    # by testing on very small levels, such as those found in the assignment.

    action_set = [action_library] * level.num_agents

    # Get possible goals
    possible_goals = [goal_description.get_sub_goal(index) for index in range(goal_description.num_sub_goals())]
    All_goals = DisjunctiveGoalDescription(possible_goals)

    print(possible_goals)


    # Define AND-OR graph search
    def AND_search(states,plan,visited,depth):
        new_depth = depth - 1
        
        planis = [OR_search(state,plan,visited,new_depth) for state in states]

        if all(plani != False for plani in planis) and all(plani != "loop" for plani in planis):
            for plani in planis:
                if plani != "loop":
                    plan.update(plani)
            return plan
        else:
            return False

    
    def OR_search(state,plan,visited, depth):
        # If the state is a goal state, then we return an empty plan, since we have reached a leaf node.    
        if All_goals.is_goal(state):
            #print("Goal found")
            return {}
        
        if depth == 0:
            print(state.state)
            return False
        
        # If State is in the plan, then we return False, since we have reached a loop.
        if state in plan.keys():
            return "loop"

        # If State is in the visited list, then we return False, since we have reached a loop.
        if state in visited:
            return "loop"
        
        visited.append(state)
        # If the state is not a goal state, then we continue the search.
        for action_helper in state.get_applicable_actions(action_set):

            plan_temp = AND_search(solution_graph_results(state,action_helper),plan, visited = visited,depth = depth)
            if plan_temp is not False:
                plan_temp[state] = action_helper
                return plan_temp
        
        # If we have not found a solution, then we return False.
        return False
        
    
    ## For testing EX2: ##
    actor_color = level.colors[str(actor_AGENT)]
    goal_mono = goal_description.color_filter(actor_color)
    initial_state_mono = initial_state.color_filter(actor_color)
    possible_goals = [] #List of sub-goals for the actor
    for index in range(goal_mono.num_sub_goals()):
        sub_goal = goal_mono.get_sub_goal(index)
        possible_goals.append(sub_goal)
    
    solved,solution_graph = all_optimal_plans(initial_state_mono, [action_library], possible_goals, frontier,
                          debug=True)


    Initial_GoalrecognitionNode = GoalRecognitionNode(initial_state, solution_graph, action_library[0])

    for d in range(5,6):
        print("Depth: ",d)
        policy = OR_search(Initial_GoalrecognitionNode,{},[],d)

        if policy != False:
            break


    print(action_library)


    print("")
    print(policy)

    agent_actions = [action_library[1],action_library[3],action_library[3],action_library[3],action_library[13]]


    for action in agent_actions:

        helper_action = policy[Initial_GoalrecognitionNode]
        
        print(action,helper_action)
        print("")
        Initial_GoalrecognitionNode = Initial_GoalrecognitionNode.result((action,helper_action))
        print(Initial_GoalrecognitionNode.state)
        print("")

    return True,policy, Initial_GoalrecognitionNode
    ## --------------- ##
    
    #raise NotImplementedError()



