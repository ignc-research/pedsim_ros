import os

import dataclasses
from enum import Enum
from typing import Dict, List, Optional, Type, TypeVar
import pedsim_msgs.msg
import std_msgs.msg

from pedsim_agents.config import Topics
import rospy

class ForcemodelName(Enum):
    PASSTHROUGH = "passthrough"
    SPINNY = "spinny"
    PYSOCIAL = "pysocial"
    DEEPSOCIALFORCE = "deepsocialforce"
    EVACUATION = "evacuation"

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

OutputData = List[pedsim_msgs.msg.AgentFeedback]

OutputMsg = pedsim_msgs.msg.AgentFeedbacks

class Forcemodel:
    def callback(self, data: InputData) -> OutputData:
        raise NotImplementedError()
    
T = TypeVar("T")


def NList(l: Optional[List[T]]) -> List[T]:
    return [] if l is None else l


class PedsimForcemodel:

    __registry: Dict[ForcemodelName, Type[Forcemodel]] = dict()

    @classmethod
    def register(cls, name: ForcemodelName):
        def inner(force_model: Type[Forcemodel]):
            if cls.__registry.get(name) is not None:
                raise NameError(f"force model {name} is already registered")

            cls.__registry[name] = force_model
            return force_model
        return inner

    def __init__(self, forcemodel_name: ForcemodelName):

        forcemodel_class = self.__registry.get(forcemodel_name)

        if forcemodel_class is None:
            raise RuntimeError(f"Force model {forcemodel_name.value} has no registered implementation.\nImplemented force models: {[name.value for name in self.__registry.keys()]}")

        publisher = rospy.Publisher(
            name=Topics.FEEDBACK,
            data_class=OutputMsg,
            queue_size=1
        )


        while True:

            running: bool = False

            if rospy.get_param("/resetting", True) == True:
                rospy.wait_for_message("/reset_end", std_msgs.msg.Empty)

            try:
                force_model: Forcemodel = forcemodel_class()
            except Exception as e:
                rospy.signal_shutdown(f"Could not initialize force model {forcemodel_name.value}. Aborting.")
                raise RuntimeError(f"Could not initialize force model {forcemodel_name.value}. Aborting.") from e


            def callback(dataframe: InputMsg):

                if not running or dataframe.agent_states is None or len(dataframe.agent_states) == 0:
                    return

                dataframe_data = InputData(
                    header=dataframe.header,
                    agents=NList(dataframe.agent_states),
                    robots=NList(dataframe.robot_states),
                    groups=NList(dataframe.simulated_groups),
                    waypoints=NList(dataframe.simulated_waypoints),
                    line_obstacles=NList(dataframe.walls),
                    obstacles=NList(dataframe.obstacles)
                )

                agent_states_data = force_model.callback(dataframe_data)

                publisher.publish(
                    OutputMsg(
                        agents=agent_states_data
                    )
                )

            sub = rospy.Subscriber(
                name=Topics.INPUT,
                data_class=InputMsg,
                callback=callback,
                queue_size=1
            )

            running = True

            rospy.loginfo(
                f"starting pedsim_waypoint_generator with force model {type(force_model).__name__}")
            
            rospy.wait_for_message("/reset_start", std_msgs.msg.Empty)

            running = False
            sub.unregister()
            del force_model