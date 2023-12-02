from pedsim_agents.pedsim_forces import OutputData, PedsimForcemodel, InputData, ForcemodelName, Forcemodel
import pedsim_msgs.msg

@PedsimForcemodel.register(ForcemodelName.PASSTHROUGH)
class Plugin_Passthrough(Forcemodel):

    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        return [pedsim_msgs.msg.AgentFeedback(unforce=True) for agent in data.agents]