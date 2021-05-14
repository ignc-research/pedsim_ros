//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 20012 by Christian Gloor
// Modified by Ronja Gueldenring
//

#include "ped_agent.h"
#include "ped_obstacle.h"
#include "ped_scene.h"
#include "ped_waypoint.h"
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <random>

using namespace std;

int Ped::Tagent::staticid = 1;
default_random_engine generator;

/// Default Constructor
Ped::Tagent::Tagent() {
  id = staticid++;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  v.x = 0;
  v.y = 0;
  v.z = 0;
  type = ADULT;
  scene = nullptr;
  teleop = false;

  // assign random maximal speed in m/s
  normal_distribution<double> distribution(0.6, 0.2);
  vmax = distribution(generator);
  vmaxDefault = vmax;
  forceFactorDesired = 1.0;
  forceFactorSocial = 2.1;
  forceFactorObstacle = 10.0;
  forceSigmaObstacle = 0.8;
  forceSigmaRobot = 0.3*vmax/0.4;

  agentRadius = 0.35;
  relaxationTime = 0.5;
  robotPosDiffScalingFactor = 5;
  obstacleForceRange = 2.0;

  keepDistanceForceDistanceDefault = 0.8;
  keepDistanceForceDistance = keepDistanceForceDistanceDefault;
  keepDistanceTo = Tvector(0.0, 0.0);
  ROS_INFO("created agent with id: %d", id);
}

/// Destructor
Ped::Tagent::~Tagent() {}

/// Assigns a Tscene to the agent. Tagent uses this to iterate over all
/// obstacles and other agents in a scene.
/// The scene will invoke this function when Tscene::addAgent() is called.
/// \warning Bad things will happen if the agent is not assigned to a scene. But
/// usually, Tscene takes care of that.
/// \param   *s A valid Tscene initialized earlier.
void Ped::Tagent::assignScene(Ped::Tscene* sceneIn) { scene = sceneIn; }

void Ped::Tagent::removeAgentFromNeighbors(const Ped::Tagent* agentIn) {
  // search agent in neighbors, and remove him
  set<const Ped::Tagent*>::iterator foundNeighbor = neighbors.find(agentIn);
  if (foundNeighbor != neighbors.end()) neighbors.erase(foundNeighbor);
}

/// Sets the maximum velocity of an agent (vmax). Even if pushed by other
/// agents, it will not move faster than this.
/// \param   pvmax The maximum velocity. In scene units per timestep, multiplied
/// by the simulation's precision h.
void Ped::Tagent::setVmax(double pvmax) { vmax = pvmax; }

/// Defines how much the position difference between this agent
/// and a robot is scaled: the bigger the number is, the smaller
/// the position based force contribution will be.
/// \param   scalingFactor should be positive.
void Ped::Tagent::setRobotPosDiffScalingFactor(double scalingFactor) {
  if (scalingFactor > 0) {
    robotPosDiffScalingFactor = scalingFactor;
  }
}

/// Sets the agent's position. This, and other getters returning coordinates,
/// will eventually changed to returning a
/// Tvector.
/// \param   px Position x
/// \param   py Position y
/// \param   pz Position z
void Ped::Tagent::setPosition(double px, double py, double pz) {
  p.x = px;
  p.y = py;
  p.z = pz;
}

/// Sets the factor by which the desired force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorDesired(double f) { forceFactorDesired = f; }

/// Sets the factor by which the social force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorSocial(double f) { forceFactorSocial = f; }

/// Sets the factor by which the obstacle force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorObstacle(double f) { forceFactorObstacle = f; }

double Ped::Tagent::keepDistanceForceFunction(double distance) {
  return -10 * (distance - keepDistanceForceDistance);
}

// force to keep a specific distance
Ped::Tvector Ped::Tagent::keepDistanceForce() {
  Tvector diff = p - keepDistanceTo;
  Tvector direction = diff.normalized();
  double magnitude = keepDistanceForceFunction(diff.length());
  Tvector force = direction * magnitude;
  return force;
}

/// Calculates the force between this agent and the next assigned waypoint.
/// If the waypoint has been reached, the next waypoint in the list will be
/// selected.
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::desiredForce() {
  // get destination
  Twaypoint* waypoint = getCurrentWaypoint();

  // if there is no destination, don't move
  if (waypoint == nullptr) {
    desiredDirection = Ped::Tvector();
    Tvector antiMove = -v / relaxationTime;
    return antiMove;
  }

  // compute force
  Tvector force = waypoint->getForce(*this, &desiredDirection);

  return force;
}

/// Calculates the social force between this agent and all the other agents
/// belonging to the same scene.
/// It iterates over all agents inside the scene, has therefore the complexity
/// O(N^2). A better
/// agent storing structure in Tscene would fix this. But for small (less than
/// 10000 agents) scenarios, this is just
/// fine.
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::socialForce() const {
  // define relative importance of position vs velocity vector
  // (set according to Moussaid-Helbing 2009)
  const double lambdaImportance = 2.0;

  // define speed interaction
  // (set according to Moussaid-Helbing 2009)
  const double gamma = 0.35;

  // define speed interaction
  // (set according to Moussaid-Helbing 2009)
  const double n = 2;

  // define angular interaction
  // (set according to Moussaid-Helbing 2009)
  const double n_prime = 3;

  Tvector force;
  for (const Ped::Tagent* other : neighbors) {
    // don't compute social force to yourself
    if (other->id == id) continue;

    // compute difference between both agents' positions
    Tvector diff = other->p - p;
    if(other->getType() == ROBOT) diff /= robotPosDiffScalingFactor;
    Tvector diffDirection = diff.normalized();
    // compute difference between both agents' velocity vectors
    // Note: the agent-other-order changed here
    Tvector velDiff = v - other->v;

    // compute interaction direction t_ij
    Tvector interactionVector = lambdaImportance * velDiff + diffDirection;
    double interactionLength = interactionVector.length();
    Tvector interactionDirection = interactionVector / interactionLength;


    // The robots influence is computed separetly in Ped::Tagent::robotForce()
    if(other->getType() == ROBOT){
      continue;
    }else{
      // compute angle theta (between interaction and position difference vector)
      Ped::Tangle theta = interactionDirection.angleTo(diffDirection);
      // compute model parameter B = gamma * ||D||
      double B = gamma * interactionLength;

      double thetaRad = theta.toRadian();
      double forceVelocityAmount =
          -exp(-diff.length() / B -
              (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
      double forceAngleAmount =
          -theta.sign() *
          exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

      Tvector forceVelocity = forceVelocityAmount * interactionDirection;
      Tvector forceAngle =
          forceAngleAmount * interactionDirection.leftNormalVector();
      force += forceVelocity + forceAngle;

    }

  }

  return force;
}
// Added by Ronja Gueldenring
// Robot influences agents behaviour according the robot force
Ped::Tvector Ped::Tagent::robotForce(){
  double vel = sqrt(pow(this->getvx(),2) + pow(this->getvy(),2));
  if (vel > 0.1){
    still_time = 0.0;
  }

  Tvector force;
  for (const Ped::Tagent* other : neighbors) {
    if(other->getType() == ROBOT){
      // pedestrian is influenced robot force depending on the distance to the robot.
      Tvector diff = other->p - p;
      Tvector diffDirection = diff.normalized();
      double distanceSquared = diff.lengthSquared();
      double distance = sqrt(distanceSquared) - (agentRadius + 0.7);
      double forceAmount = -1.0 * exp(-distance / forceSigmaRobot);
      Tvector robot_force = forceAmount * diff.normalized();
      force += robot_force;
      break;
    }
  }
  return force;
}

/// Calculates the force between this agent and the nearest obstacle in this
/// scene.
/// Iterates over all obstacles == O(N).
/// \return  Tvector: the calculated force
// Ped::Tvector Ped::Tagent::obstacleForce () const {
//   // obstacle which is closest only
//   Ped::Tvector minDiff;
//   double minDistanceSquared = INFINITY;

//   for (const Tobstacle* obstacle : scene->obstacles) {
//     Ped::Tvector closestPoint = obstacle->closestPoint(p);
//     Ped::Tvector diff = p - closestPoint;
//     double distanceSquared = diff.lengthSquared();  // use squared distance to
//     // avoid computing square
//     // root
//     if (distanceSquared < minDistanceSquared) {
//       minDistanceSquared = distanceSquared;
//       minDiff = diff;
//     }
//   }

//   double distance = sqrt(minDistanceSquared) - agentRadius;
//   double forceAmount = exp(-distance / forceSigmaObstacle);
//   return forceAmount * minDiff.normalized();
// }


std::vector<double> Ped::Tagent::LinearSpacedArray(double a, double b, std::size_t N)
{
  // create array with N linearly spaced values between a and b
  double h = (b - a) / static_cast<double>(N-1);
  std::vector<double> xs(N);
  std::vector<double>::iterator x;
  double val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
    *x = val;
  }
  return xs;
}

std::vector<int> Ped::Tagent::odomPosToMapIndex(Ped::Tvector pos)
{
  // translate a position in the "odom" frame to a map index
  int index_y = static_cast<int> ((pos.y - scene->map_.info.origin.position.y) / scene->map_.info.resolution);
  int index_x = static_cast<int> ((pos.x - scene->map_.info.origin.position.x) / scene->map_.info.resolution);
  std::vector<int> v{index_y, index_x};
  return v;
}

bool Ped::Tagent::isOccupied(Ped::Tvector pos)
{
  nav_msgs::OccupancyGrid map = scene->map_;
  std::vector<int> index = odomPosToMapIndex(pos);
  // check if indexes are within range
  if (
    0 <= index[0] &&
    index[0] < map.info.height &&
    0 <= index[1] &&
    index[1] < map.info.width)
  {
    // translate 2d index to 1d index in row-major order
    return map.data[index[0] * map.info.width + index[1]] > 0;
  }

  return true;
}

Ped::Tvector* Ped::Tagent::getClosestObstaclePos(std::vector<Ped::Tvector > considered_positions, Ped::Tvector pos)
{
  // find the closest position out of a list of considered positions
  Ped::Tvector closest_obstacle_pos;
  double closest_distance_squared = INFINITY;
  bool found_something = false;
  for (Ped::Tvector considered_pos : considered_positions)
  {
    // if (id == 0) { ROS_INFO("considered_pos x: %lf y: %lf %s", considered_pos.x, considered_pos.y, isOccupied(considered_pos) ? "full" : "empty");}
    if (isOccupied(considered_pos))
    {
      found_something = true;
      Ped::Tvector diff = pos - considered_pos;
      double distance_squared = diff.lengthSquared();
      if (distance_squared < closest_distance_squared)
      {
        closest_distance_squared = distance_squared;
        closest_obstacle_pos = considered_pos;
      }
    }
  }

  if (found_something)
  {
    return new Ped::Tvector(closest_obstacle_pos);
  }

  return nullptr;
}

std::vector<Ped::Tvector > Ped::Tagent::getSurroundingPositions(Ped::Tvector pos)
{
  // create a grid of positions around the given pos
  int resolution = 20;
  std::vector<Ped::Tvector > considered_positions;
  for (double value_x : LinearSpacedArray(pos.x - obstacleForceRange, pos.x + obstacleForceRange, resolution))
  {
    for (double value_y : LinearSpacedArray(pos.y - obstacleForceRange, pos.y + obstacleForceRange, resolution))
    {
      Ped::Tvector pos_temp(value_x, value_y);
      considered_positions.push_back(pos_temp);
    }
  }
  return considered_positions;
}

double Ped::Tagent::obstacleForceFunction(double distance)
{
  if (distance <= 0)
  {
    return 0.0;
  }
  return 3.0 / distance;
}

/// Calculate force between this agent and the nearest occupied cell
/// found in the map. Scans only the immediate area of the agent.
/// The size of the area is dependent on the value of obstacleForceRange.
Ped::Tvector Ped::Tagent::obstacleForce() {
  std::vector<Ped::Tvector > considered_positions = getSurroundingPositions(p);
  Ped::Tvector* closest_obstacle_pos = getClosestObstaclePos(considered_positions, p);

  if (closest_obstacle_pos == nullptr)
  {
    // return empty vector
    return Ped::Tvector();
  }

  Ped::Tvector diff = p - *closest_obstacle_pos;
  double force_magnitude = obstacleForceFunction(diff.length());
  Ped::Tvector force_direction = diff.normalized();
  Ped::Tvector force = force_direction * force_magnitude;
  // if (id == 0)
  // {
  //   ROS_INFO("agent pos: x: %lf y: %lf", p.x, p.y);
  //   ROS_INFO("closest_obstacle_pos: x: %lf y: %lf", closest_obstacle_pos->x, closest_obstacle_pos->y);
  //   ROS_INFO("diff: x: %lf y: %lf", diff.x, diff.y);
    // ROS_INFO("force: x: %lf y: %lf", force.x, force.y);
    // ROS_INFO("force_magnitude: %lf", force_magnitude);
  //   ROS_INFO("------------------");
  // }
  return force;
}

/// myForce() is a method that returns an "empty" force (all components set to
/// 0).
/// This method can be overridden in order to define own forces.
/// It is called in move() in addition to the other default forces.
/// \return  Tvector: the calculated force
/// \param   e is a vector defining the direction in which the agent wants to
/// walk to.
Ped::Tvector Ped::Tagent::myForce(Ped::Tvector e) {
  return Ped::Tvector(0.0, 0.0);
}

void Ped::Tagent::computeForces() {
  // update neighbors 
  // NOTE - have a config value for the neighbor range
  const double neighborhoodRange = 10.0;
  neighbors = scene->getNeighbors(p.x, p.y, neighborhoodRange);  

  // update forces
  desiredforce = desiredForce();
  if (forceFactorSocial > 0) socialforce = socialForce();
  if (forceFactorObstacle > 0) obstacleforce = obstacleForce();
  robotforce = robotForce();
  keepdistanceforce = keepDistanceForce();
  myforce = myForce(desiredDirection);
}

/// Does the agent dynamics stuff. Calls the methods to calculate the individual
/// forces, adds them
/// to get the total force affecting the agent. This will then be translated
/// into a velocity difference,
/// which is applied to the agents velocity, and then to its position.
/// \param   stepSizeIn This tells the simulation how far the agent should
/// proceed
void Ped::Tagent::move(double stepSizeIn) {
  still_time += stepSizeIn;

  // sum of all forces --> acceleration
  a = forceFactorDesired * desiredforce + forceFactorSocial * socialforce 
    + forceFactorObstacle * obstacleforce + myforce + keepdistanceforce;
    // ROS_INFO("desiredforce %lf,%lf,%lf, ", desiredforce.x,desiredforce.y,desiredforce.z);
    // ROS_INFO("socialforce, %lf,%lf,%lf",socialforce.x,socialforce.y,socialforce.z);
    // ROS_INFO("obstacleforce,%lf,%lf,%lf",obstacleforce.x,obstacleforce.y,obstacleforce.z);
    // ROS_INFO("myforce, %lf,%lf,%lf",myforce.x,myforce.y,myforce.z);
    // ROS_INFO("stepSizeln%lf",stepSizeIn);

  // Added by Ronja Gueldenring
  // add robot force, so that pedestrians avoid robot
  // if (this->getType() == ADULT_AVOID_ROBOT || this->getType() == ADULT_AVOID_ROBOT_REACTION_TIME){
      // a = a + forceFactorSocial * robotforce;
  // }
  
  // calculate the new velocity
  if (getTeleop() == false) {
    v = v + stepSizeIn * a;
    // ROS_WARN("update velocity %lf,%lf", v.x,v.y);
  }

  // don't exceed maximal speed
  double speed = v.length();
  if (speed > vmax) v = v.normalized() * vmax;

  // internal position update = actual move
  p += stepSizeIn * v;
    

  // notice scene of movement
  scene->moveAgent(this);
}
