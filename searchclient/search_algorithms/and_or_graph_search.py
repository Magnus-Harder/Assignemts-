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
from time import time
    
def and_or_graph_search(initial_state, action_set, goal_description, results):

    # Here you should implement AND-OR-GRAPH-SEARCH. We are going to use a policy format, mapping from states to actions.
    # The algorithm should return a pair (worst_case_length, or_plan)
    # where the or_plan is a dictionary with states as keys and actions as values

    # deepcoppy is used to make sure that the state is not changed by the search algorithm
    # The state is changed by the result function, which is why we need to make a copy of the state before calling it.
    # The state is also changed by the is_applicable function, which is why we need to make a copy of the state before calling it.
    # The state is also changed by the get_applicable_actions function, which is why we need to make a copy of the state before calling it.
    
    print("ACTION SET: ", action_set,file=sys.stderr)
    def Or_search(state,path,depth):

        # Cheack if state is goal
        if goal_description.is_goal(state):
            return {}
        # Check if depth is reached
        if depth == 0:
            return False

        # Check if state is in path
        if state in path: # Loop
            if cyclic:
                return "Loop"
            else:
                return False

        # loop over all actions
        #print("APPLC: ", state.get_applicable_actions(action_set),file=sys.stderr)
        for action in state.get_applicable_actions(action_set):
            # Note Plan is a policy
            path_for_plan = deepcopy(path)
            path_for_plan.append(state)

            # Find the next states
            new_states = results(state,action)
            
            # Get plan from And_search
            plan = And_search(new_states, path_for_plan, depth)

            # See if plan is succesfull
            if plan != False:
                # Add action to plan and return
                plan[state] = action
                return plan  
        
        # If no plan is found return false
        return False


    def And_search(states,path,depth):
        new_depth = depth - 1 # Update depth for next Or-search call
        
        plan = {}
        planis = [Or_search(state,path,new_depth) for state in states]
        
        #Added cyclic case
        if cyclic:

            # Ensure that not all plans are loops
            if all(p == "Loop" or p == False for p in planis):
                return False # maybe?
            else:
                # If one plan doesnt loop or fail then all succeeding plans are added
                for p in planis:
                    # There is no reason to overwrite existing plan if in loop
                    if p != False and p != "Loop":
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

    #plan = Or_search(initial_state,path,depth)
    # # Clearer debugging
    # for k, v in plan.items():
    #     print(k,v,"\n",file=sys.stderr)
    # return len(plan.keys()), plan
    
    # Can now go to next part with propper fail return
    mulipl = 2
    start_t = time()
    ## Iterative deepening search missing from prev. implementation
    depth_budget = int(20) ## Should be inf, but don't want to waste time with unreasonable searches
    for d in range(depth_budget):
        if d%mulipl == 0:
            print(f"Searching depth of [{d}] after {int(time()-start_t)}s", file=sys.stderr)
        
        plan = Or_search(initial_state,[],d) # Find plan or fail at curr depth
        
        if plan != False:
            # Return Policy and worst case length
            return d, plan
    
    if d+1 >= depth_budget:
        print(f"Depth budget of {depth_budget} reached witout cutoff!!" ,file=sys.stderr)
        if not cyclic:
            print(f"No non-looping path to goal found" ,file=sys.stderr)
        return False
    else:
        print("This should be unreachable:&")
        raise NotImplementedError("idfk")


