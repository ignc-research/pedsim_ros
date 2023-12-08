# Message defining a tracked group
#

string      group_id        # unique identifier of the target, consistent over time

duration    age             # age of the group

geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)

string[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons