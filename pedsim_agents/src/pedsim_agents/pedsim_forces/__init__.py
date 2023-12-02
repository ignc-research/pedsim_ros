import dataclasses
from enum import Enum
from typing import Dict, Type
import genpy



from pedsim_agents.config import Topics
from pedsim_agents.utils import InputData, FeedbackData, FeedbackMsg
import rospy

class ForcemodelName(Enum):
    PASSTHROUGH = "passthrough"
    SPINNY = "spinny"
    PYSOCIAL = "pysocial"
    DEEPSOCIALFORCE = "deepsocialforce"
    EVACUATION = "evacuation"

class Forcemodel:
    def callback(self, data: InputData) -> FeedbackData:
        raise NotImplementedError()

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

    forcemodel_name: ForcemodelName
    forcemodel_class: Type[Forcemodel]
    forcemodel: Forcemodel

    publisher: rospy.Publisher
    running: bool

    def __init__(self, name: str):

        try:
            forcemodel_name = ForcemodelName(name)
        except ValueError as e:
            raise ValueError(f"Force model {name} does not exist.\nAvailable force models: {[name.value for name in ForcemodelName]}") from e

        self.forcemodel_name = forcemodel_name

        forcemodel_class = self.__registry.get(forcemodel_name)

        if forcemodel_class is None:
            raise RuntimeError(f"Force model {forcemodel_name.value} has no registered implementation.\nImplemented force models: {[name.value for name in self.__registry.keys()]}")

        self.forcemodel_class = forcemodel_class

        self.publisher = rospy.Publisher(
            name=Topics.FEEDBACK,
            data_class=FeedbackMsg,
            queue_size=1
        )

        self.running = False


    def reset(self):
        if self.running:
            del self.forcemodel 
            self.running = False

        try:
            self.forcemodel: Forcemodel = self.forcemodel_class()
            rospy.loginfo(f"starting pedsim_agents with force model {self.forcemodel_class.__name__}")
        except Exception as e:
            rospy.signal_shutdown(f"Could not initialize force model {self.forcemodel_name.value}. Aborting.")
            raise RuntimeError(f"Could not initialize force model {self.forcemodel_name.value}. Aborting.") from e
        
        self.running = True

    def calculate(self, data: InputData) -> FeedbackData:
        if len(data.agents) == 0:
            return []
        
        return self.forcemodel.callback(data)
    
    def publish(self, stamp: genpy.Time, data: FeedbackData):
        msg = FeedbackMsg()
        msg.header.stamp = stamp
        msg.agents = data
        self.publisher.publish(msg)