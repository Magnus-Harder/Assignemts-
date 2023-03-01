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
    def __init__(self, advanced_type):
        self.advanced_type = advanced_type
        pass

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        if self.advanced_type == "simple":
            print("=========")
        if self.advanced_type[:5] == "exact":
            self.exact_dist_preprocess(level)
        else:
            pass

    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        if self.advanced_type == "exact":
            return self.exact_dist_lookup(state,goal_description)
        elif self.advanced_type == "exact_min":
            return self.exact_dist_lookup_min(state,goal_description)
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
        
        """
        for box_tuple in box_positions:
            name = box_tuple[1]
            pos = box_tuple[0]
            if not name in self.box_names:
                continue
            total_dist += self.pre_calc_dists[pos[0]][pos[1]].distance_to(name)
        """

        return total_dist/(len(agent_goals)+len(box_goals))

    def exact_dist_lookup_min(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        agent_positions = state.agent_positions
        agent_goals = goal_description.agent_goals

        box_positions = state.box_positions
        box_goals = goal_description.box_goals
        total_dist = 0
        
        goals_min = {a_goal[1]: APPROX_INFINITY for a_goal in agent_goals}
        goals_min.update({b_goal[1]: APPROX_INFINITY for b_goal in box_goals})
        def loop_add_min_to_total(positions):
            function_dist = 0
            for tuple in positions:
                name = tuple[1]
                pos = tuple[0]
                if not name in self.all_names:
                    continue
                
                curr_dist = self.pre_calc_dists[pos[0]][pos[1]].distance_to(name)
                goals_min[name] = min(goals_min[name], curr_dist)
            
            for name_n in self.all_names:
                #print(name_n,goals_min,file=sys.stderr)
                new_dist = goals_min[name_n]
                if new_dist == APPROX_INFINITY:
                    continue
                function_dist += new_dist
            return function_dist
        
        total_dist += loop_add_min_to_total(agent_positions)
        total_dist += loop_add_min_to_total(box_positions)
        
        """
        for box_tuple in box_positions:
            name = box_tuple[1]
            pos = box_tuple[0]
            if not name in self.box_names:
                continue
            total_dist += self.pre_calc_dists[pos[0]][pos[1]].distance_to(name)
        """

        return total_dist/(len(agent_goals)+len(box_goals))
    
    def exact_dist_preprocess(self, level: h_level.HospitalLevel):
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

        for i in range(1,rows-1):
            for j in range(1,cols-1):
                i_add = i+1
                j_add = j
                #print(i_add,j_add, file=sys.stderr)
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                i_add = i-1
                j_add = j
                #print(i_add,j_add, file=sys.stderr)
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                i_add = i
                j_add = j+1
                #print(i_add,j_add, file=sys.stderr)
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                i_add = i
                j_add = j-1
                #print(i_add,j_add, file=sys.stderr)
                if not level.wall_at([i_add,j_add]):
                    self.pre_calc_dists[i][j].add_neighbor(
                        self.pre_calc_dists[i_add][j_add]
                    )
                #print(i,j,self.pre_calc_dists[i][j].neighbors, file=sys.stderr)
        
        def expand_distances(goals_in, box = False):
            for goal in goals_in:
                pos_g = goal[0]
                agent_g_name = goal[1]

                positive_goal = goal[2] #Not yet used

                queue = deque()
                start_node = self.pre_calc_dists[pos_g[0]][pos_g[1]]

                start_node.expand_node(agent_g_name, 0)
                queue.append(start_node)
                
                print(goals_in,file=sys.stderr)

                while len(queue) != 0:
                    expanding_node = queue.popleft()
                    for neigh in expanding_node.neighbors:
                        if box:
                            if not neigh.box_expanded[agent_g_name]:
                                new_distance = (expanding_node.box_goal_dist[agent_g_name] + 1)
                                neigh.expand_node(agent_g_name, new_distance, box = box)
                                queue.append(neigh)
                        else:
                            if not neigh.agent_expanded[agent_g_name]:
                                new_distance = (expanding_node.agent_goal_dist[agent_g_name] + 1)
                                neigh.expand_node(agent_g_name, new_distance)
                                queue.append(neigh)
        
        expand_distances(level.agent_goals)
        expand_distances(level.box_goals, box=True)        
        # for i in range(1,rows-1):
        #     for j in range(1,cols-1):
        #         #print(i,j,self.pre_calc_dists[i][j].distance_to_agent(agent_g_name), agent_g_name,file=sys.stderr)
        #         #print(len(self.pre_calc_dists[i][j].neighbors),file=sys.stderr)
        #         pass

        for agent_index in range(agent_goal_num):
            goal = level.agent_goals[agent_index]
            pos_g = goal[0]
            agent_g_name = goal[1]
            #print(self.pre_calc_dists[3][3].distance_to_agent(agent_g_name), agent_g_name)

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