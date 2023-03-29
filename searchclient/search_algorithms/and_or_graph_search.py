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
            print("<<<<<<<<<<<<<", file=sys.stderr)
            print(state,file=sys.stderr)
            print("##########", file=sys.stderr)
            new_states = results(state,action)
            for s in new_states:
                print(s, file=sys.stderr)
            print("#########", file=sys.stderr)
            print(state,file=sys.stderr)
            print("<<<<<<<<<<<<<", file=sys.stderr)
            
            # Get plan from And_search
            plan = And_search(new_states, path_for_plan, depth)

            # See if plan is succesfull
            if plan != False:

                # if state in plan.keys():
                #     print(state,file=sys.stderr)
                # Add action to plan
                plan[state] = action
                print("------\n", plan,file=sys.stderr)

                # Return plan
                return plan  
        
        # If no plan is found return false
        return False


    def And_search(states,path,depth):
        new_depth = depth - 1
        print(new_depth, file=sys.stderr)
        
        plan = {}
        planis = [Or_search(state,path,new_depth) for state in states]
        #Added cyclic case
        if cyclic:

            # Ensure that not all plans are loops
            if all(p == "Loop" or p == False for p in planis):
                return plan
            else:
                # If one plan doesnt loop or fail then all succeeding plans are added
                for p in planis:
                    # There is no reason to overwrite existing plan if in loop
                    if p != False and p != "Loop":
                        plan.update(p) 

        else:
            # Where is return failiure if any or leaf is not goal node???
            print("TTTTTTTTTTT\n", planis, file=sys.stderr)
            print(all(p != False for p in planis), file=sys.stderr)
            if all(p != False for p in planis):
                for p in planis:
                    # Add succsesfull plans
                    plan.update(p)
            else:
                return False
        
        # Return plan
        return plan

    # Return Policy and worst case length
    path = []
    depth = 2
    cyclic = False
    plan = Or_search(initial_state,path,depth)

    # Clearer debugging
    for k, v in plan.items():
        print(k,v,"\n",file=sys.stderr)
    return len(plan.keys()), plan
    
    # Can't go to next part without propper fail return
    
    ## Iterative deepening search missing from prev. implementation
    depth_budget = 1e1 ## Should be inf, but don't want to waste time with unreasonable searches
    for d in range(depth_budget):
        plan = Or_search(initial_state,[],d)
    
    if d+1 >= depth_budget:
        print(f"Depth budget of {depth_budget} reached witout cutoff!!" ,file=sys.stderr)
        return False
    else:
        print("This should be unreachable:&")
        raise NotImplementedError("idfk")


