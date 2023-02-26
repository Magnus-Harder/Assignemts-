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
        pass

    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        goal_count = 0
        sub_goal_num = goal_description.num_sub_goals()
        for index in range(sub_goal_num):
            sub_goal = goal_description.get_sub_goal(index)
            if not sub_goal.is_goal(state):
                goal_count += 1

        # IF NOT DIVIDED A* WILL NOT FIND OPTIMAL
        # -- not [enter correct term] heuristic if not divided
        return goal_count/sub_goal_num


class HospitalAdvancedHeuristics:

    def __init__(self):
        pass

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        self.exact_dist_preprocess(level)


    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return self.simple_dist_heuristic(state,goal_description)

        

    def simple_dist_heuristic(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        agent_positions = state.agent_positions
        agent_goals = goal_description.agent_goals
        total_dist = 0

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
                else:
                    total_dist -= manhattan_dist

        # IF NOT DIVIDED A* WILL NOT FIND OPTIMAL
        # -- not [enter correct term] heuristic if not divided
        return total_dist/len(agent_goals)
    
    def exact_dist_lookup(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        pass
    
    def exact_dist_preprocess(self, level: h_level.HospitalLevel):
        """
        - agent_goals and box_goals are lists of goals in the format (position, char, is_positive)
        """
        rows = len(level.walls)
        cols = len(level.walls[0])
        box_goal_num = len(level.box_goals)
        agent_goal_num = len(level.agent_goals)

        self.pre_calc_dists = [deepcopy([DistanceNode(agent_goal_num,box_goal_num) for i in range(cols)]) for i in range(rows)]

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
        
        for agent_index in range(agent_goal_num):
            goal = level.agent_goals[agent_index]
            pos_g = goal[0]
            agent_g_name = goal[1]
            positive_goal = goal[2]

            queue = deque()
            start_node = self.pre_calc_dists[pos_g[0]][pos_g[1]]
            #print(pos_g,agent_g_name,file=sys.stderr)
            
            start_node.agent_expanded[agent_g_name] = True
            start_node.agent_goal_dist[agent_g_name] = 0
            queue.append(start_node)

            while len(queue) != 0:
                expanding_node = queue.popleft()
                for neigh in expanding_node.neighbors:
                    if not neigh.agent_expanded[agent_g_name]:
                        neigh.agent_goal_dist[agent_g_name] = expanding_node.distance_to_agent(agent_g_name) + 1
                        neigh.agent_expanded[agent_g_name] = True
                        queue.append(expanding_node)
        
        for i in range(1,rows-1):
            for j in range(1,cols-1):
                #print(i,j,self.pre_calc_dists[i][j].distance_to_agent(agent_g_name), agent_g_name,file=sys.stderr)
                #print(len(self.pre_calc_dists[i][j].neighbors),file=sys.stderr)
                pass
        
                


        for agent_index in range(agent_goal_num):
            goal = level.agent_goals[agent_index]
            pos_g = goal[0]
            agent_g_name = goal[1]
            #print(self.pre_calc_dists[3][3].distance_to_agent(agent_g_name), agent_g_name)

class DistanceNode:
    def __init__(
        self,
        agent_goal_num: int,
        box_goal_num: int,
    ) -> None:
        self.agent_goal_dist = {f"{i}": -1 for i in range(agent_goal_num)}
        self.box_goal_dist = [(APPROX_INFINITY,"-1")]*box_goal_num
        self.agent_expanded = {f"{i}": False for i in range(agent_goal_num)}
        self.box_expanded = [False]*box_goal_num
        self.neighbors = []
    
    def add_neighbor(self,added_neighbor):
        self.neighbors.append(added_neighbor)
    
    def distance_to_agent(self, agent_string):
        agent_dist = self.agent_goal_dist[agent_string]
        #assert agent_dist_tuple[1] == agent_string, "Agents not indexes in same order always"
        return agent_dist