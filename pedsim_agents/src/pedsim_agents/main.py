#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import std_msgs.msg

from pedsim_agents.config import Topics
from pedsim_agents.utils import NList, InputMsg, InputData

from pedsim_agents.pedsim_semantic import SemanticProcessor
from pedsim_agents.pedsim_forces.pedsim_forces import PedsimForcemodel

# Main function.
def main():
    rospy.init_node("pedsim_agents")

    #TODO automate this
    Topics.INPUT = os.path.join(rospy.get_name(), Topics.INPUT)
    Topics.FEEDBACK = os.path.join(rospy.get_name(), Topics.FEEDBACK)
    Topics.SEMANTIC = os.path.join(rospy.get_name(), Topics.SEMANTIC)

    forcemodel = PedsimForcemodel(str(rospy.get_param("~forcemodel", "")))

    semantic = SemanticProcessor()
    sub: rospy.Subscriber

    def callback(input_msg: InputMsg):

        if not running:
            return;
    
        stamp = input_msg.header.stamp

        input_data = InputData(
            header=input_msg.header,
            agents=NList(input_msg.agent_states),
            robots=NList(input_msg.robot_states),
            groups=NList(input_msg.simulated_groups),
            waypoints=NList(input_msg.simulated_waypoints),
            line_obstacles=NList(input_msg.walls),
            obstacles=NList(input_msg.obstacles)
        )

        feedback_data = forcemodel.calculate(input_data)

        semantic_data = semantic.calculate(input_data, feedback_data)

        forcemodel.publish(stamp, feedback_data)
        semantic.publish(stamp, semantic_data)
    
    while True:

        running: bool = False

        if rospy.get_param("/resetting", False) == True:
            rospy.wait_for_message("/reset_end", std_msgs.msg.Empty)

        forcemodel.reset()
        semantic.reset()

        sub = rospy.Subscriber(
            name=Topics.INPUT,
            data_class=InputMsg,
            callback=callback,
            queue_size=1
        )

        running = True
        
        rospy.wait_for_message("/reset_start", std_msgs.msg.Empty)

        running = False
        sub.unregister()

if __name__ == "__main__":
    main()