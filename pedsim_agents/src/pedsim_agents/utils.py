import dataclasses
import enum
from typing import Any, Dict, List, Optional, Tuple, Type, TypeVar

import pedsim_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

# INPUT

InputMsg = pedsim_msgs.msg.PedsimAgentsDataframe

@dataclasses.dataclass
class InputData:
    header: std_msgs.msg.Header
    agents: List[pedsim_msgs.msg.AgentState]
    robots: List[pedsim_msgs.msg.RobotState]
    groups: List[pedsim_msgs.msg.AgentGroup]
    waypoints: List[pedsim_msgs.msg.Waypoint]
    line_obstacles: List[pedsim_msgs.msg.Wall] #TODO rename to walls
    obstacles: List[pedsim_msgs.msg.Obstacle]


# FEEDBACK

@dataclasses.dataclass
class FeedbackDatum:
    feedback: pedsim_msgs.msg.AgentFeedback
    TODO: None = None

FeedbackData = List[FeedbackDatum]
FeedbackMsg = pedsim_msgs.msg.AgentFeedbacks


# SEMANTIC 

@enum.unique
class SemanticAttribute(enum.Enum):
    IS_PEDESTRIAN = "pedestrian"
    IS_PEDESTRIAN_MOVING = "pedestrian_moving"
    PEDESTRIAN_VEL_X = "pedestrian_vel_x"
    PEDESTRIAN_VEL_Y = "pedestrian_vel_y"

SemanticMsg = pedsim_msgs.msg.SemanticData
SemanticData = Dict[SemanticAttribute, List[Tuple[geometry_msgs.msg.Point, float]]]

T = TypeVar("T")

def NList(l: Optional[List[T]]) -> List[T]:
    return [] if l is None else l