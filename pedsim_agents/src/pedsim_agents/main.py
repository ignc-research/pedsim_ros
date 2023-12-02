#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pedsim_agents.pedsim_forces.pedsim_forces import PedsimForcemodel, ForcemodelName
import rospy


# Main function.
def main():
    print("\n".join(rospy.get_param_names()))

    rospy.init_node("pedsim_agents")

    try:
        force_model: ForcemodelName = ForcemodelName(rospy.get_param("~forcemodel"))
    
    except ValueError as e:    
        raise ValueError(f"Available force models: {[name.value for name in ForcemodelName]}") from e
    
    PedsimForcemodel(forcemodel_name=force_model)
    rospy.spin()

if __name__ == "__main__":
    main()