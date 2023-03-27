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
from copy import deepcopy

    
def and_or_graph_search(initial_state, action_set, goal_description, results):

    # Here you should implement AND-OR-GRAPH-SEARCH. We are going to use a policy format, mapping from states to actions.
    # The algorithm should return a pair (worst_case_length, or_plan)
    # where the or_plan is a dictionary with states as keys and actions as values

    # deepcoppy is used to make sure that the state is not changed by the search algorithm
    # The state is changed by the result function, which is why we need to make a copy of the state before calling it.
    # The state is also changed by the is_applicable function, which is why we need to make a copy of the state before calling it.
    # The state is also changed by the get_applicable_actions function, which is why we need to make a copy of the state before calling it.

    # print(action_set,file=sys.stderr)
    def Or_search(state,path,depth=100):

        # Check if depth is reached
        if depth == 0:
            return False
        else:
            depth -= 1

        # Cheack if state is goal
        if goal_description.is_goal(state):
            return {}
        
        # Check if state is in path
        if state in path:
            return False
        
        # loop over all actions
        for action in state.get_applicable_actions(action_set):
            print(state.get_applicable_actions(action_set),file=sys.stderr)
            # Note Plan is a policy
            path_for_plan = deepcopy(path)
            path_for_plan.append(state)

            # Dont include actions which result in illegal states
            new_states = results(state,action)
            # x = []
            # for s in new_states:
            #     if s.get_applicable_actions(action_set) != []:
            #         x.append(s)
            
            # Get plan from And_search
            plan = And_search(new_states, path_for_plan, depth)

            # See if plan is succesfull
            if plan != False:

                # Add action to plan
                plan[state] = action

                # Return plan
                return plan  
        
        # If no plan is found return false
        return False


    def And_search(states,path,depth):

        plan = {}
        # loop over possible results
        for state in states:
            plani = Or_search(state,path,depth)
   
            # See if plan is succesfull
            if plani != False:
                plan.update(plani)
            
        
        # Return plan
        return plan

    # Return Policy and worst case length
    path = []
    depth = 10
    plan = Or_search(initial_state,path,depth)

    return len(plan.keys()), plan
