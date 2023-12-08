import enum
import os
from typing import Dict, List, Tuple

import numpy as np
import genpy
import rospy

import pedsim_msgs.msg

from pedsim_agents.config import Topics
from pedsim_agents.utils import InputData, FeedbackData, FeedbackDatum, SemanticMsg, SemanticData, SemanticAttribute, PedType


class SemanticProcessor:

    publishers: Dict[SemanticAttribute, rospy.Publisher]

    def __init__(self):
        self.publishers = {attribute: rospy.Publisher(
            name = os.path.join(Topics.SEMANTIC, str(attribute.value)),
            data_class = SemanticMsg,
            queue_size = 1
        ) for attribute in SemanticAttribute}

    def calculate(self, input_data: InputData, feedback_data: FeedbackData) -> SemanticData:
        
        semantic_data: SemanticData = dict()

        for attribute in SemanticAttribute:
            semantic_data[attribute] = []

        def get_attributes(state: pedsim_msgs.msg.AgentState, feedback: FeedbackDatum) -> List[Tuple[SemanticAttribute, float]]:
            attributes: List[Tuple[SemanticAttribute, float]] = []

            attributes.append((SemanticAttribute.IS_PEDESTRIAN, 1))

            if np.linalg.norm([feedback.feedback.force.x, feedback.feedback.force.y]) > 0.05:
                attributes.append((SemanticAttribute.IS_PEDESTRIAN_MOVING, 1))

            attributes.append((SemanticAttribute.PEDESTRIAN_VEL_X, feedback.feedback.force.x))
            attributes.append((SemanticAttribute.PEDESTRIAN_VEL_Y, feedback.feedback.force.y))

            attributes.append((SemanticAttribute.PEDESTRIAN_TYPE, PedType[str(state.type)].value))

            return attributes
        
        for state, feedback in zip(input_data.agents, feedback_data):
            for attribute, intensity in get_attributes(state, feedback):
                semantic_data[attribute].append((state.pose.position, intensity))

        return semantic_data

    def publish(self, stamp: genpy.Time, data: SemanticData):
        for attribute, evidences in data.items():
            msg = SemanticMsg()

            msg.header.stamp = stamp
            msg.points = [pedsim_msgs.msg.SemanticDatum(location=location, evidence=evidence) for location, evidence in evidences]
            msg.type = attribute.value

            self.publishers[attribute].publish(msg)

    def reset(self):
        ...