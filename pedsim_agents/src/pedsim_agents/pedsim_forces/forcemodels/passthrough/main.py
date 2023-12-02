from pedsim_agents.pedsim_forces import FeedbackData, PedsimForcemodel, InputData, ForcemodelName, Forcemodel
import pedsim_msgs.msg

@PedsimForcemodel.register(ForcemodelName.PASSTHROUGH)
class Plugin_Passthrough(Forcemodel):

    def __init__(self):
        ...

    def callback(self, data) -> FeedbackData:
        return [pedsim_msgs.msg.AgentFeedback(agent.forces.force) for agent in data.agents]