import dataclasses
import enum
from typing import Any, Dict, List, Optional, Tuple, Type, TypeVar

import pedsim_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

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

FeedbackMsg = pedsim_msgs.msg.AgentFeedbacks
FeedbackData = List[pedsim_msgs.msg.AgentFeedback]


class SemanticAttribute(enum.IntEnum):
    IS_PEDESTRIAN = 0
    IS_PEDESTRIAN_MOVING = 1

SemanticMsg = pedsim_msgs.msg.SemanticData
SemanticData = Dict[SemanticAttribute, List[ Tuple[geometry_msgs.msg.Point, float]]]

T = TypeVar("T")

def NList(l: Optional[List[T]]) -> List[T]:
    return [] if l is None else l