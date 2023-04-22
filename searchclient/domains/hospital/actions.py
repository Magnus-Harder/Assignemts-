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

# pos_add and pos_sub are helper methods for performing element-wise addition and subtractions on positions
# Ex: Given two positions A = (1, 2) and B = (3, 4), pos_add(A, B) == (4, 6) and pos_sub(B, A) == (2,2)
from utils import pos_add, pos_sub
from typing import Union, Tuple
import domains.hospital.state as h_state
import sys

direction_deltas = {
    'N': (-1, 0),
    'S': (1, 0),
    'E': (0, 1),
    'W': (0, -1),
}

# An action class must implement three types be a valid action:
# 1) is_applicable(self, agent_index, state) which return a boolean describing whether this action is valid for
#    the agent with 'agent_index' to perform in 'state' independently of any other action performed by other agents.
# 2) result(self, agent_index, state) which modifies the 'state' to incorporate the changes caused by the agent
#    performing the action. Since we *always* call both 'is_applicable' and 'conflicts' prior to calling 'result',
#    there is no need to check for correctness.
# 3) conflicts(self, agent_index, state) which returns information regarding potential conflicts with other actions
#    performed concurrently by other agents. More specifically, conflicts can occur with regard to the following
#    two invariants:
#    A) Two objects may not have the same destination.
#       Ex: '0  A1' where agent 0 performs Move(E) and agent 1 performs Push(W,W)
#    B) Two agents may not move the same box concurrently,
#       Ex: '0A1' where agent 0 performs Pull(W,W) and agent 1 performs Pull(E,E)
#    In order to check for these, the conflict method should return two lists:
#       a) destinations which contains all newly occupied cells.
#       b) moved_boxes which contains the current position of boxes moved during the action, i.e. their position
#          prior to being moved by the action.
# Note that 'agent_index' is the index of the agent in the state.agent_positions list which is often but *not always*
# the same as the numerical value of the agent character.


Position = Tuple[int, int] # Only for type hinting

class NoOpAction:

    def __init__(self):
        self.name = "NoOp"

    def is_applicable(self, agent_index: int,  state: h_state.HospitalState) -> bool:
        # Optimization. NoOp can never change the state if we only have a single agent
        return len(state.agent_positions) > 1

    def result(self, agent_index: int, state: h_state.HospitalState):
        pass

    def conflicts(self, agent_index: int, state: h_state.HospitalState) -> tuple[list[Position], list[Position]]:
        current_agent_position, _ = state.agent_positions[agent_index]
        destinations = [current_agent_position]
        boxes_moved = []
        return destinations, boxes_moved

    def __repr__(self) -> str:
        return self.name


class MoveAction:

    def __init__(self, agent_direction):
        self.agent_delta = direction_deltas.get(agent_direction)
        self.name = "Move(%s)" % agent_direction

    def calculate_positions(self, current_agent_position: Position) -> Position:
        return pos_add(current_agent_position, self.agent_delta)

    def is_applicable(self, agent_index: int,  state: h_state.HospitalState) -> bool:
        current_agent_position, _ = state.agent_positions[agent_index]
        new_agent_position = self.calculate_positions(current_agent_position)
        return state.free_at(new_agent_position)

    def result(self, agent_index: int, state: h_state.HospitalState):
        current_agent_position, agent_char = state.agent_positions[agent_index]
        new_agent_position = self.calculate_positions(current_agent_position)
        state.agent_positions[agent_index] = (new_agent_position, agent_char)

    def conflicts(self, agent_index: int, state: h_state.HospitalState) -> tuple[list[Position], list[Position]]:
        current_agent_position, _ = state.agent_positions[agent_index]
        new_agent_position = self.calculate_positions(current_agent_position)
        # New agent position is a destination because it is unoccupied before the action and occupied after the action.
        destinations = [new_agent_position]
        # Since a Move action never moves a box, we can just return the empty value.
        boxes_moved = []
        return destinations, boxes_moved

    def __repr__(self):
        return self.name

class PushAction:

    def __init__(self, agent_direction, box_direction):
        self.agent_dir = agent_direction
        self.box_dir = box_direction
        self.agent_delta = direction_deltas.get(agent_direction)
        self.box_delta = direction_deltas.get(box_direction)
        self.name = f"Push({agent_direction},{box_direction})"

    def calculate_positions(self, current_agent_position: Position) -> Position:
        new_agent_position = pos_add(current_agent_position, self.agent_delta)
        return new_agent_position, pos_add(new_agent_position, self.box_delta)

    def is_applicable(self, agent_index: int, state: h_state.HospitalState) -> bool:
        current_agent_position, agent_char = state.agent_positions[agent_index]
        new_agent_position, new_box_position = self.calculate_positions(current_agent_position)

        # removed _, agent_char = stage.agent_at(...) since it seemed useless to not just fetch it at the top
        box_index, box_char = state.box_at(new_agent_position)
        
        # if self.agent_dir == "S" and self.box_dir == "E" and current_agent_position == (3,2):
        #     print(" ",
        #     current_agent_position, self.agent_dir, self.box_dir, "\n",
        #     state.box_at(pos_add(current_agent_position, direction_deltas.get("N"))), "\n",
        #     state.box_at(pos_add(current_agent_position, direction_deltas.get("E"))), "\n",
        #     state.box_at(pos_add(current_agent_position, direction_deltas.get("S"))), "\n",
        #     state.box_at(pos_add(current_agent_position, direction_deltas.get("W"))), "\n",
        #         file=sys.stderr)

        if box_index == -1:
            return False
        
        if state.level.colors[box_char] != state.level.colors[agent_char]:
            return False 
    
        valid_move = state.free_at(new_box_position)
              
        return valid_move

    def conflicts(self, agent_index: int, state: h_state.HospitalState) -> tuple[list[Position], list[Position]]:
        current_agent_position, _ = state.agent_positions[agent_index]
        new_agent_position, new_box_position = self.calculate_positions(current_agent_position)
        # box_index, _ = state.box_at(new_agent_position)

        # New agent position is a destination because it is occupied before the action and occupied after the action.
        destinations = [new_agent_position,new_box_position]
        # New box position is a destination because it is unoccupied before the action and occupied after the action.
        
        #boxes_moved = [box_index]
        boxes_moved =[new_agent_position] #REF 3 B b) positions prior ro being moved

        return destinations, boxes_moved
    
    def result(self, agent_index: int, state: h_state.HospitalState):

        current_agent_position, agent_char = state.agent_positions[agent_index]
        new_agent_position, new_box_position = self.calculate_positions(current_agent_position)
        box_index, box_char = state.box_at(new_agent_position)

        state.agent_positions[agent_index] = (new_agent_position, agent_char)
        state.box_positions[box_index] = (new_box_position, box_char)

    def __repr__(self):
        return self.name


class PullAction:

    def __init__(self, agent_direction, box_direction):
        self.agent_dir = agent_direction
        self.box_dir = box_direction
        self.agent_delta = direction_deltas.get(agent_direction)
        self.box_delta = direction_deltas.get(box_direction)
        self.name = f"Pull({agent_direction},{box_direction})"

    def calculate_positions(self, current_agent_position: Position) -> Position:
        # box_position = pos_sub(current_agent_position, self.box_delta)
        new_box_position = current_agent_position #pos_add(box_position, self.box_delta)
        new_agent_position = pos_add(current_agent_position, self.agent_delta)
        return new_agent_position, new_box_position

    def is_applicable(self, agent_index: int, state: h_state.HospitalState) -> bool:
        current_agent_position, agent_char = state.agent_positions[agent_index]
        new_agent_position, new_box_position = self.calculate_positions(current_agent_position)
        box_position = pos_sub(current_agent_position, self.box_delta)

        box_index, box_char = state.box_at(box_position)
        if box_index == -1:
            return False
        
        if state.level.colors[box_char] != state.level.colors[agent_char]:
            return False 

        valid_move = state.free_at(new_agent_position)
 
        return valid_move
    
    def __repr__(self):
        return self.name

    def conflicts(self, agent_index: int, state: h_state.HospitalState) -> tuple[list[Position], list[Position]]:
        current_agent_position, _ = state.agent_positions[agent_index]
        #new_box_position, new_agent_position = self.calculate_positions(current_agent_position)
        new_agent_position, new_box_position = self.calculate_positions(current_agent_position)
        #box_index, _ = state.box_at(current_agent_position)
        #box_index, _ = state.box_at(pos_sub(current_agent_position,self.box_delta))

        # New agent position is a destination because it is occupied before the action and occupied after the action.
        destinations = [new_agent_position,new_box_position]
        # New box position is a destination because it is unoccupied before the action and occupied after the action.
        
        #boxes_moved = [box_index]
        boxes_moved =[pos_sub(current_agent_position, self.box_delta)] #REF 3 B b) positions prior ro being moved

        return destinations, boxes_moved
    
    def result(self, agent_index: int, state: h_state.HospitalState):
        
        current_agent_position, agent_char = state.agent_positions[agent_index]
        new_agent_position, new_box_position = self.calculate_positions(current_agent_position) #PROBLEM WAS HERE (and same in other places, but this was the main one)
        box_position = pos_sub(current_agent_position, self.box_delta)
        box_index, box_char = state.box_at(box_position)

        state.agent_positions[agent_index] = (new_agent_position, agent_char)
        state.box_positions[box_index] = (new_box_position, box_char)
    
AnyAction = Union[NoOpAction, MoveAction, PushAction, PullAction] # Only for type hinting # added ours


# An action library for the multi agent pathfinding
DEFAULT_MAPF_ACTION_LIBRARY = [
    NoOpAction(),
    MoveAction("N"),
    MoveAction("S"),
    MoveAction("E"),
    MoveAction("W"),
]


# An action library for the full hospital domain
DEFAULT_HOSPITAL_ACTION_LIBRARY = [
    NoOpAction(),
    MoveAction("N"),
    MoveAction("S"),
    MoveAction("E"),
    MoveAction("W"),
    
    # Add Push and Pull actions here
    # Push actions (Cannot push box unto current agent position since this cell is not currently free)
    PushAction("N", "N"),
    PushAction("N", "E"),
    PushAction("N", "W"),
    PushAction("S", "S"),
    PushAction("S", "E"),
    PushAction("S", "W"),
    PushAction("E", "N"),
    PushAction("E", "S"),
    PushAction("E", "E"),
    PushAction("W", "N"),
    PushAction("W", "S"),
    PushAction("W", "W"),

    # Pull actions (Same as push, cannot pull box and move agent in opposite directions)
    PullAction("N", "N"),
    PullAction("N", "E"),
    PullAction("N", "W"),
    PullAction("S", "S"),
    PullAction("S", "E"),
    PullAction("S", "W"),
    PullAction("E", "N"),
    PullAction("E", "S"),
    PullAction("E", "E"),
    PullAction("W", "N"),
    PullAction("W", "S"),
    PullAction("W", "W"),
]

ROBOT_ACTION_LIBRARY = [
    NoOpAction(),
    MoveAction("N"),
    MoveAction("S"),
    MoveAction("E"),
    MoveAction("W"),
    
    # Add Push and Pull actions here
    # Push actions (Cannot push box unto current agent position since this cell is not currently free)
    PushAction("N", "N"),
    PushAction("S", "S"),
    PushAction("E", "E"),
    PushAction("W", "W"),
]

