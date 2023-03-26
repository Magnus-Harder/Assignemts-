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
from __future__ import annotations
import sys
import itertools
from utils import pos_add, pos_sub, APPROX_INFINITY

import domains.hospital.state as h_state
import domains.hospital.goal_description as h_goal_description
import domains.hospital.level as h_level

from collections import deque
from copy import deepcopy
import copy
from collections import defaultdict

class HospitalGoalCountHeuristics:

    def __init__(self):
        pass

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        self.num_box_goals = level.num_box_goals

    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return self.simple_all_subgoal_count(state,goal_description)

    def simple_all_subgoal_count(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        goal_count = 0
        sub_goal_num = goal_description.num_sub_goals()
        for index in range(sub_goal_num):
            sub_goal = goal_description.get_sub_goal(index)
            if not sub_goal.is_goal(state):
                goal_count += 1

        # IF NOT DIVIDED A* WILL NOT FIND OPTIMAL
        # -- not [enter correct term] heuristic if not divided
        return goal_count/sub_goal_num

    def box_first_subgoal_count(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        sub_goal_num = goal_description.num_sub_goals()
        goal_count = sub_goal_num

        for index in range(self.num_box_goals):
            sub_goal = goal_description.get_sub_goal(index)
            if not sub_goal.is_goal(state):
                goal_count -= 1 #THIS ONE STARTS AT MAX (and doesnt subtract the agents untill the boxes are done)
        
        if goal_count <= self.num_box_goals:
            for index in range(self.num_box_goals,sub_goal_num):
                sub_goal = goal_description.get_sub_goal(index)
                if not sub_goal.is_goal(state):
                    goal_count -= 1 #THIS ONE STARTS AT MAX (and doesnt subtract the agents untill the boxes are done)

        # IF NOT DIVIDED A* WILL NOT FIND OPTIMAL
        # -- not [enter correct term] heuristic if not divided
        return goal_count/sub_goal_num




class HospitalAdvancedHeuristics:
    def __init__(self, advanced_type = "exact_final_improved"):
        self.advanced_type = advanced_type
        #For dividing by agent and box number and not goal number
        self.fixed_divisor = False
        self.ran_best_candidate = False
        if advanced_type[-5:] == "fixed":
            self.fixed_divisor = True
            self.advanced_type = advanced_type[:-6]


    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        if self.advanced_type == "simple":
            print("=========", file=sys.stderr)
        if self.advanced_type[:5] == "exact":
            improved = False
            if self.advanced_type[-8:] == "improved":
                improved = True
                print("+++++++", file=sys.stderr)
            self.exact_dist_preprocess(level, improved=improved)
        else:
            pass

    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        if self.advanced_type == "exact":
            return self.exact_dist_lookup(state,goal_description)
        elif self.advanced_type == "exact_min":
            return self.exact_dist_lookup_min(state,goal_description)
        elif self.advanced_type == "exact_complex":
            return self.complex_lookup_min(state,goal_description)
        elif self.advanced_type == "exact_complex_improved":
            return self.complex_lookup_improved(state,goal_description)
        elif self.advanced_type == "exact_ican_improved":
            return self.ican_improved(state,goal_description)
        elif self.advanced_type == "exact_ican":
            return self.ican(state,goal_description)
        elif self.advanced_type == "exact_boxes_improved":
            return self.complex_lookup_improved(state,goal_description, only_boxes=True)
        elif self.advanced_type == "exact_final_improved":
            return self.final(state,goal_description)
        else:
            return self.simple_dist_heuristic(state,goal_description)

    def simple_dist_heuristic(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        agent_positions = state.agent_positions
        box_positions = state.box_positions
        agent_goals = goal_description.agent_goals
        box_goals = goal_description.box_goals
        total_dist = 0

        for box_goal in box_goals:
            pos_bg = box_goal[0]
            bg_name = box_goal[1]
            positive_bgoal = box_goal[2]
            for box_tuple in box_positions:
                if box_tuple[1] != bg_name:
                    continue
                x_dist = abs(box_tuple[0][0] - pos_bg[0])
                y_dist = abs(box_tuple[0][1] - pos_bg[1])

                manhattan_dist = x_dist + y_dist
                if positive_bgoal:
                    total_dist += manhattan_dist
                elif manhattan_dist < 1:
                    total_dist += 1 #Adding one for penalty just for now
        
        if total_dist >= 1:
            if self.fixed_divisor:
                return total_dist/len(box_positions)
            return total_dist/len(box_goals)
        
        #Only go to agent dist after boxes are in place
        for goal in agent_goals:
            pos_g = goal[0]
            agent_g_name = goal[1]
            positive_goal = goal[2]

            for agent_tuple in agent_positions:
                if agent_tuple[1] != agent_g_name:
                    continue
                x_dist = abs(agent_tuple[0][0] - pos_g[0])
                y_dist = abs(agent_tuple[0][1] - pos_g[1])

                manhattan_dist = x_dist + y_dist
                if positive_goal:
                    total_dist += manhattan_dist
                elif manhattan_dist < 1:
                    total_dist += 1 #Adding one for penalty just for now 

        # IF NOT DIVIDED A* WILL NOT FIND OPTIMAL
        # -- not [enter correct term] heuristic if not divided
        if self.fixed_divisor:
            return total_dist/(len(agent_positions)+len(box_positions))
        return total_dist/(len(agent_goals)+len(box_goals))
    
    def exact_dist_lookup(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        agent_positions = state.agent_positions
        agent_goals = goal_description.agent_goals

        box_positions = state.box_positions
        box_goals = goal_description.box_goals
        total_dist = 0
        
        def loop_add_to_total(positions):
            function_dist = 0
            for tuple in positions:
                name = tuple[1]
                pos = tuple[0]
                if not name in self.all_names:
                    continue
                function_dist += self.pre_calc_dists[pos[0]][pos[1]].distance_to(name)
            return function_dist
        
        total_dist += loop_add_to_total(agent_positions)
        total_dist += loop_add_to_total(box_positions)
        

        if self.fixed_divisor:
            return total_dist/(len(agent_positions)+len(box_positions))
        return total_dist/(len(agent_goals)+len(box_goals))

    def exact_dist_lookup_min(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        agent_positions = state.agent_positions
        agent_goals = goal_description.agent_goals

        box_positions = state.box_positions
        box_goals = goal_description.box_goals
        total_dist = 0
        
        goals_min = {a_goal[1]: [APPROX_INFINITY,[0,0]] for a_goal in agent_goals}
        goals_min.update({b_goal[1]: [APPROX_INFINITY,[0,0]] for b_goal in box_goals})
        def loop_add_min_to_total(positions):
            function_dist = 0
            for tuple in positions:
                name = tuple[1]
                pos = tuple[0]
                if not name in self.all_names:
                    continue
                
                curr_dist = self.pre_calc_dists[pos[0]][pos[1]].distance_to(name)
                if goals_min[name][0] > curr_dist:
                    goals_min[name] = [curr_dist,pos]
            
            for name_n in self.all_names:
                #print(name_n,goals_min,file=sys.stderr)
                new_dist = goals_min[name_n][0]
                if new_dist == APPROX_INFINITY:
                    continue
                function_dist += new_dist
            return function_dist
        
        total_dist += loop_add_min_to_total(agent_positions)
        total_dist += loop_add_min_to_total(box_positions)

        return total_dist/(len(agent_goals)+len(box_goals))
    
    def complex_lookup_min(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        agent_positions = state.agent_positions
        agent_goals = goal_description.agent_goals

        box_positions = state.box_positions
        box_goals = goal_description.box_goals
        total_dist = 0
        
        goals_min = {a_goal[1]: [APPROX_INFINITY,[0,0]] for a_goal in agent_goals}
        goals_min.update({b_goal[1]: [APPROX_INFINITY,[0,0]] for b_goal in box_goals})
        def loop_add_min_to_total(positions):
            function_dist = 0
            for tuple in positions:
                name = tuple[1]
                pos = tuple[0]
                if not name in self.all_names:
                    continue
                
                curr_dist = self.pre_calc_dists[pos[0]][pos[1]].distance_to(name)
                if goals_min[name][0] > curr_dist:
                    goals_min[name] = [curr_dist,pos]
            
            for name_n in self.all_names:
                #print(name_n,goals_min,file=sys.stderr)
                new_dist = goals_min[name_n][0]
                if new_dist == APPROX_INFINITY:
                    continue
                function_dist += new_dist
            return function_dist
        
        # total_dist += loop_add_min_to_total(agent_positions)
        total_dist += loop_add_min_to_total(box_positions)

        for goal_name, closest_dist_tuple in goals_min.items():
            if '0' <= goal_name <= '9':
                continue
            
            pos_box = closest_dist_tuple[1]
            closets_agent_dist = APPROX_INFINITY
            for agent_tuple in agent_positions:
                agent_name = agent_tuple[1]
                pos = agent_tuple[0]

                x_dist = abs(pos[0] - pos_box[0])
                y_dist = abs(pos[1] - pos_box[1])
                manhattan_dist = x_dist + y_dist
                if manhattan_dist < closets_agent_dist:
                    closets_agent_dist = manhattan_dist
            total_dist += closets_agent_dist

        return total_dist/(len(agent_positions)+len(box_goals))

    def complex_lookup_improved(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription, only_boxes = False, total= False) -> int:
        agent_positions = state.agent_positions
        agent_goals = goal_description.agent_goals

        box_positions = state.box_positions
        box_goals = goal_description.box_goals
        total_dist = 0
        
        goals_min = [[a_goal[1], APPROX_INFINITY,[0,0]] for a_goal in agent_goals]
        agent_goal_num = len(goals_min)
        goals_min.extend([[b_goal[1],APPROX_INFINITY,[0,0]] for b_goal in box_goals])

        goals_found_min = [False for i in range(len(goals_min))]
        
        def loop_add_min_to_total(positions):
            function_dist = 0
            for tuple in positions:
                name = tuple[1]
                pos = tuple[0]
                if not name in self.all_names:
                    continue
                for goal_index in range(len(goals_min)):
                    # if a clos
                    if (goals_min[goal_index][0] != name):
                        continue
                    closets_goal = [-1,"",APPROX_INFINITY]
                    curr_dist = self.pre_calc_dists[pos[0]][pos[1]].distance_to_improved(goal_index)
                    ###print("--\n",goal_index,"-", curr_dist,file=sys.stderr)
                    if total:
                        goals_min[goal_index][1] += curr_dist
                    elif goals_min[goal_index][1] > curr_dist:
                        goals_min[goal_index] = [goals_min[goal_index][0],curr_dist,pos]
            
            ###print("--\n", goals_min,file=sys.stderr)
            for goal_index in range(len(goals_min)):
                new_dist = goals_min[goal_index][1]
                if new_dist == APPROX_INFINITY:
                    continue
                function_dist += new_dist*new_dist
            return function_dist
        
        #total_dist += loop_add_min_to_total(agent_positions)
        total_dist += loop_add_min_to_total(box_positions)
        
        if only_boxes: #Ignores agent distance to boxes
            if total:
                return total_dist/(len(box_goals) + len(box_positions))
            return total_dist/(len(box_goals)) #total_dist/(len(agent_positions)+len(box_goals))

        only_closets = False
        agent_to_box_min = APPROX_INFINITY
        for closest_dist_triplet in goals_min:
            if '0' <= closest_dist_triplet[0] <= '9':
                continue
            
            pos_box = closest_dist_triplet[2]
            closets_agent_dist = APPROX_INFINITY
            for agent_tuple in agent_positions:
                agent_name = agent_tuple[1]
                pos = agent_tuple[0]

                x_dist = abs(pos[0] - pos_box[0])
                y_dist = abs(pos[1] - pos_box[1])
                manhattan_dist = x_dist + y_dist
                ###print("--\n", manhattan_dist,file=sys.stderr)
                if manhattan_dist < closets_agent_dist:
                    closets_agent_dist = manhattan_dist
            if closets_agent_dist == APPROX_INFINITY:
                continue
            if closest_dist_triplet[1] < (1 + only_closets): #If near in goal
                # < 1 would encourage the greedy to pull it out of the goal to get the discount from being close to it:/
                continue
            if not only_closets:
                total_dist += closets_agent_dist
            elif closets_agent_dist < agent_to_box_min:
                agent_to_box_min = closets_agent_dist

        if only_closets and agent_to_box_min < APPROX_INFINITY:
            total_dist += agent_to_box_min

        return total_dist/(len(agent_positions)+len(box_goals))
    
    def final(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription, agent_div = 12) -> int:
        agent_positions = state.agent_positions
        agent_goals = goal_description.agent_goals

        box_positions = state.box_positions
        box_goals = goal_description.box_goals
        total_dist = 0
        
        goals = goal_description.goals
        agent_goal_num = len(agent_goals)

        
        def loop_to_list(positions):
            goal_of_chars = {k:[] for k in self.n_surpluss.keys()}
            for obj_idx, tuple in enumerate(positions):
                curr_min = APPROX_INFINITY
                curr_min_pos = [0,0]
                name = tuple[1]
                pos = tuple[0]
                if not name in self.all_names:
                    continue
                for goal_index, goal in enumerate(goals):
                    #goal_index = (goal_index - agent_goal_num)%len(goals) # I numbered them the wrong way
                    if (goal[1] != name):
                        continue
                    curr_dist = self.pre_calc_dists[pos[0]][pos[1]].distance_to_improved(goal_index)
                    
                    if curr_min > curr_dist:
                        curr_min = curr_dist
                        curr_min_pos = pos
                goal_of_chars[name].append((curr_min_pos, curr_min))

            for name_sur, num in self.n_surpluss.items():
                if num > 0:
                    yes = goal_of_chars[name_sur]
                    #print(yes, file=sys.stderr)
                    yes.sort(key = lambda x: x[1])
                    #print(yes[:num], file=sys.stderr)
                    goal_of_chars[name_sur] = yes[:-num] #removing largest dists above num of goals
            list_out = []
            for val in goal_of_chars.values():
                list_out.extend(val)
            return list_out
        
        list_dist = loop_to_list(box_positions)

        agnets_dist = [[-1,APPROX_INFINITY]]*10
        list_dist.sort(key = lambda x: x[1], reverse=True)
        agent_queue = deque()
        for i in range(len(agent_positions)):
            agent_queue.append(i)
        while len(agent_queue) != 0:
            curr_agent = agent_queue.popleft()

            pos = agent_positions[curr_agent][0]

            for tupple_i, tuple_dist in enumerate(list_dist):
                pos_box = tuple_dist[0]
                #print(pos[0],list_dist, file=sys.stderr)
                x_dist = abs(pos[0] - pos_box[0])
                y_dist = abs(pos[1] - pos_box[1])
                manhattan_dist = x_dist + y_dist

                for j in range(len(agent_positions)):
                    if agnets_dist[j][0] == tupple_i:
                        if agnets_dist[j][1] <= manhattan_dist:
                            break
                        else:
                            agent_queue.append(j)
                else:
                    agnets_dist[curr_agent] = [tupple_i,manhattan_dist]
                    break

        #print(agnets_dist[:len(agent_positions)], file=sys.stderr)
        for tup in agnets_dist[:len(agent_positions)]:
            total_dist += tup[1]/agent_div
        for tup in list_dist:
            total_dist += tup[1]
        # if total_dist < 2:
        #     print(agnets_dist[:len(agent_positions)], file=sys.stderr)

        return total_dist/(len(agent_positions)+len(box_goals))
    
    def exact_dist_preprocess(self, level: h_level.HospitalLevel, improved = False):

        """
        - agent_goals and box_goals are lists of goals in the format (position, char, is_positive)
        """
        rows = len(level.walls)
        cols = len(level.walls[0])

        agent_goal_num = len(level.agent_goals)
        
        # Object "global" variables #

        self.box_names = set([b_g[1] for b_g in level.box_goals])
        self.agent_names = set([a_g[1] for a_g in level.agent_goals])
        self.all_names = set([b_g[1] for b_g in level.box_goals]+[a_g[1] for a_g in level.agent_goals])
        self.pre_calc_dists = [deepcopy([DistanceNode(level.agent_goals, level.box_goals) for i in range(cols)]) for j in range(rows)]
        ####

        number_of_each_goal = defaultdict(lambda: 0)
        number_of_each_obj = defaultdict(lambda: 0)
        number_of_surpluss = defaultdict(lambda: 0)
        for goal in level.agent_goals:
            number_of_each_goal[goal[1]] +=1
        for goal in level.box_goals:
            number_of_each_goal[goal[1]] +=1
        for goal in level.initial_agent_positions:
            number_of_each_obj[goal[1]] +=1
        for goal in level.initial_box_positions:
            number_of_each_obj[goal[1]] +=1
        
        for name_sur in self.all_names:
            number_of_surpluss[name_sur] = number_of_each_obj[name_sur] - number_of_each_goal[name_sur]
        
        self.n_surpluss = number_of_surpluss #For FINAL

        for i in range(1,rows-1):
            for j in range(1,cols-1):
                i_add = i+1
                j_add = j
                ###print(i_add,j_add, file=sys.stderr)
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                i_add = i-1
                j_add = j
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                i_add = i
                j_add = j+1
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                i_add = i
                j_add = j-1
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                #print(i,j,self.pre_calc_dists[i][j].neighbors, file=sys.stderr)
        
        def expand_distances(goals_in, box = False,impoved = improved):
            for goal_index, goal in enumerate(goals_in):
                pos_g = goal[0]
                agent_g_name = goal[1]

                positive_goal = goal[2] #Not yet used

                queue = deque()
                start_node = self.pre_calc_dists[pos_g[0]][pos_g[1]]

                start_node.expand_node(agent_g_name, 0)
                start_node.expand_node_improved(goal_index, 0)
                queue.append(start_node)

                agent_goal_num = len(level.agent_goals)
                while len(queue) != 0:
                    expanding_node = queue.popleft()
                    for neigh in expanding_node.neighbors:
                        if box:
                            box_index = goal_index + agent_goal_num
                            if impoved:
                                if not neigh.index_expanded[box_index]:
                                    new_distance = (expanding_node.goals_dist[box_index][1] + 1)
                                    neigh.expand_node_improved(box_index, new_distance)
                                    queue.append(neigh)
                            else:
                                if not neigh.box_expanded[agent_g_name]:
                                    new_distance = (expanding_node.box_goal_dist[agent_g_name] + 1)
                                    neigh.expand_node(agent_g_name, new_distance, box = box)
                                    queue.append(neigh)
                        else:
                            if impoved:
                                if not neigh.index_expanded[goal_index]:
                                    new_distance = (expanding_node.goals_dist[goal_index][1] + 1)
                                    neigh.expand_node_improved(goal_index, new_distance)
                                    queue.append(neigh)   
                            else:
                                if not neigh.agent_expanded[agent_g_name]:
                                    new_distance = (expanding_node.agent_goal_dist[agent_g_name] + 1)
                                    neigh.expand_node(agent_g_name, new_distance)
                                    queue.append(neigh)
        
        expand_distances(level.agent_goals)
        expand_distances(level.box_goals, box=True)        

class DistanceNode:
    def __init__(
        self,
        agent_goals,
        box_goals,
    ) -> None:
        self.agent_goal_dist = {a_goal[1]: -1 for a_goal in agent_goals}
        self.box_goal_dist = {b_goal[1]: -1 for b_goal in box_goals}

        self.agent_expanded = {a_goal[1]: False for a_goal in agent_goals}
        self.box_expanded = {b_goal[1]: False for b_goal in box_goals}

        self.goals_dist = [[a_goal[1], APPROX_INFINITY,[0,0]] for a_goal in agent_goals]
        self.agent_goal_num = len(agent_goals)
        self.goals_dist.extend([[b_goal[1],APPROX_INFINITY,[0,0]] for b_goal in box_goals])
        self.index_expanded = [False for fs in range(len(self.goals_dist))] 

        self.neighbors = []

    
    def add_neighbor(self,added_neighbor):
        self.neighbors.append(added_neighbor)
    
    def distance_to(self, name: str):
        if '0' <= name <= '9':
            return self.agent_goal_dist[name]
        if 'A' <= name <= 'Z':
            return self.box_goal_dist[name]

    #Expanding node when first calculating distances
    def expand_node(self, name, new_dist, box=False):
        if '0' <= name <= '9':
            self.agent_goal_dist[name] = new_dist
            self.agent_expanded[name] = True
        if ('A' <= name <= 'Z') or box:
            self.box_goal_dist[name] = new_dist
            self.box_expanded[name] = True

    # # MULTI GOAL METHODS eg. SAsoko levels
    def expand_node_improved(self, goal_indx, new_dist):
        self.goals_dist[goal_indx][1] = new_dist
        self.index_expanded[goal_indx] = True
    
    def distance_to_improved(self, goal_indx):
        return self.goals_dist[goal_indx][1]