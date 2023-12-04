from pedsim_agents.pedsim_forces import PedsimForcemodel, ForcemodelName, Forcemodel
from pedsim_agents.utils import FeedbackDatum, FeedbackData

import pedsim_msgs.msg

@PedsimForcemodel.register(ForcemodelName.PASSTHROUGH)
class Plugin_Passthrough(Forcemodel):

    def __init__(self):
        ...

    def callback(self, data) -> FeedbackData:
        return [
            FeedbackDatum(
                pedsim_msgs.msg.AgentFeedback(
                    id = agent.id,
                    force = agent.forces.force
                )
            )
        for agent
        in data.agents]