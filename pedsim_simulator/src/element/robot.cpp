#include <pedsim_simulator/element/robot.h>

Robot::Robot(pedsim::id name, std::string topicOdom, ros::NodeHandle nh)
: ScenarioElement(){
    this->state.name = name;
    this->subscriber_odom_ = nh.subscribe(
        topicOdom,
        1,
        &Robot::_callbackOdom,
        this
    );
};

Robot::~Robot() {
}

void Robot::_callbackOdom(boost::shared_ptr< ::nav_msgs::Odometry const> msg){
    this->state.pose = msg->pose.pose;
}

pedsim_msgs::RobotState Robot::getState(){
    return this->state;
}


QString Robot::toString() const {
  return tr("Robot %1").arg(this->state.name.c_str());
}
QPointF Robot::getVisiblePosition() const { return QPointF(this->state.pose.position.x, this->state.pose.position.y); }

void Robot::setVisiblePosition(const QPointF& positionIn) {
  // check and apply new position
  if (positionIn != getVisiblePosition()){
    this->state.pose.position.x = positionIn.x();
    this->state.pose.position.y = positionIn.y();
  }
}