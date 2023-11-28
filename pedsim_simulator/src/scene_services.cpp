/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically and add static obstacle spawning.
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 * @author Junhui Li
 * @date 24.3.2021
 **/

#include <pedsim_simulator/element/robot.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/wall.h>
#include <pedsim_simulator/scene_services.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/SetBool.h>
#include <flatland_msgs/SpawnModels.h>
#include <flatland_msgs/DeleteModels.h>
// #include <flatland_msgs/RespawnModels.h>
#include <iostream>
#include <pedsim_simulator/rng.h>
#include <ros/package.h>

int SceneServices::agents_index_ = 20;
int SceneServices::static_obstacles_index_ = 1;
std::vector<pedsim::id> SceneServices::static_obstacle_names_;

SceneServices::SceneServices()
{
  // pedsim services
  reset_service_ = nh_.advertiseService("pedsim_simulator/reset", &SceneServices::cb_reset, this);

  spawn_peds_service_ = nh_.advertiseService("pedsim_simulator/spawn_peds", &SceneServices::cb_spawnPeds, this);
  respawn_peds_service_ = nh_.advertiseService("pedsim_simulator/respawn_peds", &SceneServices::cb_respawnPeds, this);
  move_peds_service_ = nh_.advertiseService("pedsim_simulator/move_peds", &SceneServices::cb_moveAgentClustersInPedsim, this);
  reset_all_peds_service_ = nh_.advertiseService("pedsim_simulator/reset_all_peds", &SceneServices::cb_resetPeds, this);
  remove_all_peds_service_ = nh_.advertiseService("pedsim_simulator/remove_all_peds", &SceneServices::cb_removeAllPeds, this);

  add_walls_service_ = nh_.advertiseService("pedsim_simulator/add_walls", &SceneServices::cb_addWalls, this);
  clear_walls_service_ = nh_.advertiseService("pedsim_simulator/clear_walls", &SceneServices::cb_clearWalls, this);

  spawn_obstacles_service_ = nh_.advertiseService("pedsim_simulator/spawn_obstacles", &SceneServices::cb_spawnObstacles, this);
  respawn_obstacles_service_ = nh_.advertiseService("pedsim_simulator/respawn_obstacles", &SceneServices::cb_respawnObstacles, this);
  remove_all_obstacles_service_ = nh_.advertiseService("pedsim_simulator/remove_all_obstacles", &SceneServices::cb_removeAllObstacles, this);

  register_robot_service_ = nh_.advertiseService("pedsim_simulator/register_robot", &SceneServices::cb_registerRobot, this);

  // Check if flatland is the chosen simulation environment
  pedsim::id environment;
  nh_.param<pedsim::id>("/simulator", environment, "flatland");
  if (environment != "flatland")
    env_is_flatland = false;
  if (env_is_flatland)
  {
    // flatland service clients
    spawn_models_topic_ = ros::this_node::getNamespace() + "/spawn_models";
    spawn_models_client_ = nh_.serviceClient<flatland_msgs::SpawnModels>(spawn_models_topic_, true);
    // respawn_models_topic_ = ros::this_node::getNamespace() + "/respawn_models";
    // respawn_models_client_ = nh_.serviceClient<flatland_msgs::RespawnModels>(respawn_models_topic_, true);
    delete_models_topic_ = ros::this_node::getNamespace() + "/delete_models";
    delete_models_client_ = nh_.serviceClient<flatland_msgs::DeleteModels>(delete_models_topic_, true);
  }
}

bool SceneServices::cb_spawnPeds(pedsim_srvs::SpawnPeds::Request &request, pedsim_srvs::SpawnPeds::Response &response)
{
  std::vector<flatland_msgs::Model> flatland_models;

  for (auto &ped : request.peds)
  {
    std::vector<pedsim::id> new_agent_ids = generateAgentIds(ped.id, ped.number_of_peds);

    // add ped to pedsim
    addAgentClusterToPedsim(ped, new_agent_ids);

    // add flatland models to spawn_models service request
    std::vector<flatland_msgs::Model> new_models = getFlatlandModels(ped, new_agent_ids);
    flatland_models.insert(flatland_models.end(), new_models.begin(), new_models.end());
  }
  if (env_is_flatland)
  {
    bool res = spawnModelsInFlatland(flatland_models);
    response.success = res;
    return res;
  }
  return true;
}

bool SceneServices::cb_respawnPeds(pedsim_srvs::SpawnPeds::Request &request,
                                pedsim_srvs::SpawnPeds::Response &response)
{
  ROS_DEBUG("called respawnPeds() with %ld peds", request.peds.size());

  std_srvs::SetBool::Request request_;
  std_srvs::SetBool::Response response_;
  bool res = cb_removeAllPeds(request_, response_);
  if (!res)
  {
    response.success = false;
    return false;
  }

  res = cb_spawnPeds(request, response);
  if (!res)
  {
    response.success = false;
    return false;
  }

  response.success = true;
  return true;
}

bool SceneServices::cb_removeAllPeds(std_srvs::SetBool::Request &request,
                                  std_srvs::SetBool::Response &response)
{
  std::vector<pedsim::id> model_names = removePedsInPedsim();
  if (env_is_flatland)
  {
    bool res = removeModelsInFlatland(model_names);
    response.success = res;
    return res;
  }
  // if (!request.data)
  // {
  //   ROS_INFO("Deleting all interactive obstacles in removeAllPeds");
  //   removeAllInteractiveObstaclesFromPedsim();
  // }

  return true;
}


bool SceneServices::cb_respawnObstacles(pedsim_srvs::SpawnObstacles::Request &request, pedsim_srvs::SpawnObstacles::Response &response)
{
  // remove old
  std_srvs::Trigger::Request trigger_request;
  std_srvs::Trigger::Response trigger_response;

  if(!cb_removeAllObstacles(trigger_request, trigger_response)){
    response.success = false;
    return false;
  }
  // spawn new
  bool res = cb_spawnObstacles(request, response);

  response.success = res;
  return res;
}

bool SceneServices::cb_spawnObstacles(pedsim_srvs::SpawnObstacles::Request &request, pedsim_srvs::SpawnObstacles::Response &response)
{
  std::vector<flatland_msgs::Model> new_models;

  for (auto obstacle : request.obstacles)
  {

    auto q = obstacle.pose.orientation;
    // https://stackoverflow.com/a/37560411
    double yaw = atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));

    pedsim::id name = obstacle.name;
    static_obstacle_names_.push_back(name);

    // add to pedsim
    auto waypoint_pos = Ped::Tvector(obstacle.pose.position.x, obstacle.pose.position.y);
    // auto waypoint_pos = Ped::Tvector(obstacle.pose.position.x, obstacle.pose.position.y);
    auto waypoint = new AreaWaypoint(QString(name.c_str()), waypoint_pos, 0.3);
    waypoint->interactionRadius = obstacle.interaction_radius;
    if (obstacle.interaction_radius < 0.1)
    {
      // ROS_WARN("interaction_radius is smaller than 0.1. agents will not interact with this obstacle");
    }
    // waypoint->staticObstacleAngle = fmod(yaw + M_PI, 2 * M_PI);

    // // set type
    // std::string type_string = obstacle.type;
    // int type = 0;
    // // convert type string to enum value
    // auto types = SCENE.obstacle_types;
    // auto it = find(types.begin(), types.end(), type_string);
    // // If element was found
    // if (it != types.end())
    // {
    //   type = it - types.begin();
    // }
    // waypoint->setType(static_cast<Ped::Twaypoint::WaypointType>(type));

    // // set radius for obstacle force calculation
    // int radius = 1.0;
    // int radius_index = waypoint->getType();
    // if (radius_index < (int)SCENE.obstacle_radius.size())
    // {
    //   radius = SCENE.obstacle_radius[radius_index];
    // }
    // waypoint->modelRadius = radius;
    // // SCENE.removeWaypoint(name);
    // SCENE.addWaypoint(waypoint);

    // create circular obstacles for applying obstacle force to agents
    // auto circle_obstacle = Ped::Twaypoint(obstacle.pose.position.x, obstacle.pose.position.y);
    // circle_obstacle.modelRadius = radius;
    // SCENE.circleObstacles.push_back(circle_obstacle);

    // create wall obstacles along model footprint for applying obstacle force to agents

    // ROS_WARN("calling getWallsFromFlatlandModel()");
    // ROS_WARN(yaw);
    // std::cout << "yaw when calling getWallFromFlatlandModel: " << yaw << std::endl;
    std::vector<Wall *> new_walls = getWallsFromFlatlandModel(obstacle, yaw);
    // append to current list of walls

    Obstacle *sceneObstacle = new Obstacle();
    sceneObstacle->obstacle = obstacle;
    sceneObstacle->walls = new_walls;

    SCENE.addObstacle(sceneObstacle);

    // create flatland model
    flatland_msgs::Model model;
    model.name = name;
    model.ns = name;
    model.pose.x = obstacle.pose.position.x;
    model.pose.y = obstacle.pose.position.y;
    model.pose.theta = yaw;
    model.yaml_path = obstacle.yaml_path;
    new_models.push_back(model);
  }
  if (env_is_flatland)
  {
    bool res = spawnModelsInFlatland(new_models);
    response.success = res;
    return res;
  }
  response.success = true;
  return true;
}

bool SceneServices::removeAllReferencesToInteractiveObstacles()
{
  // check agents that are referencing interactive waypoints
  auto agents = SCENE.getAgents();
  for (auto agent : agents)
  {
    // check lastInteractedWithWaypoint
    if (agent->lastInteractedWithWaypoint != nullptr)
    {
      if (agent->lastInteractedWithWaypoint->isInteractive())
      {
        agent->lastInteractedWithWaypoint = nullptr;
        agent->lastInteractedWithWaypointId = -1;
      }
    }

    // check lastInteractedWithWaypointId
    if (agent->lastInteractedWithWaypointId != pedsim::id_null)
    {
      agent->lastInteractedWithWaypoint = nullptr;
      agent->lastInteractedWithWaypointId = pedsim::id_null;
    }

    // check currentDestination
    if (agent->currentDestination != nullptr)
    {
      if (agent->currentDestination->isInteractive())
      {
        agent->updateDestination();
      }
    }

    // check waypointplanner
    auto waypointplanner = agent->getWaypointPlanner();
    if (waypointplanner != nullptr)
    {
      auto waypoint = waypointplanner->getCurrentWaypoint();
      if (waypoint != nullptr)
      {
        if (waypoint->isInteractive())
        {
          // waypointplanner is pointing to an interactive obstacle
          // -> just copy destination from agent
          // we ensured above that the current agent destination is not interactive
          waypointplanner->setDestination(agent->getCurrentDestination());
        }
      }
    }
  }

  return true;
}

bool SceneServices::removeAllInteractiveObstaclesFromPedsim()
{
  removeAllReferencesToInteractiveObstacles();

  // actually remove waypoints from scene
  auto waypoints = SCENE.getWaypoints();
  std::vector<Waypoint *> to_remove;
  for (auto waypoint : waypoints.values())
  {
    if (waypoint->isInteractive())
    {
      to_remove.push_back(waypoint);
    }
  }

  for (auto waypoint : to_remove)
  {
    SCENE.removeWaypoint(waypoint);
  }

  // remove obstacles
  SCENE.removeAllObstacles();

  return true;
}

bool SceneServices::removeAllInteractiveObstaclesFromFlatland()
{
  return removeModelsInFlatland(static_obstacle_names_);
}

bool SceneServices::cb_removeAllObstacles(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  ROS_DEBUG("Removing all interactive obstacles");
  removeAllInteractiveObstaclesFromPedsim();
  if (env_is_flatland)
  {
    if(!removeAllInteractiveObstaclesFromFlatland()){
      response.success = false;
      return false;
    }
  }
  static_obstacles_index_ = 1;
  static_obstacle_names_.clear();
  response.success = true;
  response.message = "";
  return true;
}

bool SceneServices::cb_resetPeds(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  QList<Agent *> agents = SCENE.getAgents();
  for (Agent *a : agents)
  {
    a->reset();
  }

  response.success = true;
  return true;
}

int SceneServices::stringToEnumIndex(std::string str, std::vector<std::string> values)
{
  int index = 0;
  auto it = find(values.begin(), values.end(), str);
  // If element was found
  if (it != values.end())
  {
    index = it - values.begin();
  }
  return index;
}

bool SceneServices::addAgentClusterToPedsim(pedsim_msgs::Ped ped, std::vector<pedsim::id> ids)
{
  uniform_real_distribution<double> distribution_x(ped.pos.x - 1.0, ped.pos.x + 1.0);
  uniform_real_distribution<double> distribution_y(ped.pos.y - 1.0, ped.pos.y + 1.0);

  for (auto &id : ids)
  {
    Agent *a = new Agent(id);

    // randomize location if we have more than one agent
    if (ped.number_of_peds > 1)
    {
      a->setPosition(distribution_x(RNG()), distribution_y(RNG()));
    }
    else
    {
      a->setPosition(ped.pos.x, ped.pos.y);
    }

    // record initial positions used when agent is reset
    a->initialPosX = a->getx();
    a->initialPosY = a->gety();

    // set type
    a->setType(static_cast<Ped::Tagent::AgentType>(stringToEnumIndex(ped.type, SCENE.agent_types)));

    // set agent radius
    int radius = 1.0;
    int radius_index = a->getType();
    if (radius_index < (int)SCENE.agent_radius.size())
    {
      radius = SCENE.agent_radius[radius_index];
    }
    a->SetRadius(radius);

    a->setVmax(ped.vmax);
    if (a->getType() == Ped::Tagent::ELDER)
    {
      a->setVmax(ped.vmax * 0.7);
    }
    a->vmaxDefault = a->getVmax();

    // startup mode
    a->startUpMode = static_cast<Agent::StartUpMode>(stringToEnumIndex(ped.start_up_mode, SCENE.start_up_modes));
    if (a->startUpMode == Agent::StartUpMode::WAITTIMER)
    {
      a->getStateMachine()->activateState(AgentStateMachine::AgentState::StateWaitForTimer);
    }
    else if (a->startUpMode == Agent::StartUpMode::TRIGGERZONE)
    {
      a->getStateMachine()->activateState(AgentStateMachine::AgentState::StateWaitForTrigger);
    }
    else
    {
      a->getStateMachine()->activateState(AgentStateMachine::AgentState::StateNone);
    }
    // wait time
    a->waitTime = ped.wait_time;
    // trigger zone radius
    a->triggerZoneRadius = ped.trigger_zone_radius;

    a->chattingProbability = ped.chatting_probability;
    a->stateTalkingBaseTime = ped.talking_base_time;

    a->tellStoryProbability = ped.tell_story_probability;
    a->stateTellStoryBaseTime = ped.tell_story_base_time;

    a->groupTalkingProbability = ped.group_talking_probability;
    a->stateGroupTalkingBaseTime = ped.group_talking_base_time;

    a->talkingAndWalkingProbability = ped.talking_and_walking_probability;
    a->stateTalkingAndWalkingBaseTime = ped.talking_and_walking_base_time;

    a->requestingServiceProbability = ped.requesting_service_probability;
    a->stateRequestingServiceBaseTime = ped.requesting_service_base_time;
    a->stateReceivingServiceBaseTime = ped.receiving_service_base_time;

    a->requestingGuideProbability = ped.requesting_guide_probability;

    a->requestingFollowerProbability = ped.requesting_follower_probability;

    a->maxTalkingDistance = ped.max_talking_distance;
    if (
        a->getType() == Ped::Tagent::AgentType::ADULT ||
        a->getType() == Ped::Tagent::AgentType::ELDER ||
        a->getType() == Ped::Tagent::AgentType::CHILD)
    {
      if (a->maxTalkingDistance < 0.1)
      {
        ROS_WARN("maxTalkingDistance is very small. ped will probably not interact with others");
      }
    }

    a->maxServicingRadius = ped.max_servicing_radius;
    if (a->getType() == Ped::Tagent::AgentType::SERVICEROBOT)
    {
      if (a->maxServicingRadius < 0.1)
      {
        ROS_WARN("maxServicingRadius is very small. service robot will not provide service");
      }
    }

    a->waypointMode = static_cast<Agent::WaypointMode>(ped.waypoint_mode);

    // set force factors
    a->forceFactorDesired = ped.force_factor_desired;
    if (a->forceFactorDesired < 0.1)
    {
      ROS_WARN("forceFactorDesired is very small. ped will probably not move");
    }
    a->forceFactorObstacle = ped.force_factor_obstacle;
    a->forceFactorSocial = ped.force_factor_social;
    a->forceFactorRobot = ped.force_factor_robot;

    // std::cout << "SIZE " << ped.waypoints.size() << std::endl;

    // add waypoints to agentcluster and scene
    for (int i = 0; i < (int)ped.waypoints.size(); i++)
    {

      const double x = ped.waypoints[i].x;
      const double y = ped.waypoints[i].y;
      AreaWaypoint *w = new AreaWaypoint(QString(std::to_string(Ped::Twaypoint::staticid++).c_str()), x, y, 0.3);
      uniform_real_distribution<double> Distribution(0.0, 2 * M_PI);
      w->staticObstacleAngle = Distribution(RNG());
      w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
      SCENE.addWaypoint(w);
      a->addWaypoint(w);
    }

    // add agent to scene
    SCENE.addAgent(a);
  }

  return true;
}

std::vector<flatland_msgs::Model> SceneServices::getFlatlandModels(pedsim_msgs::Ped ped, std::vector<pedsim::id> ids)
{
  std::vector<flatland_msgs::Model> flatland_msg;

  for (auto &id : ids)
  {
    flatland_msgs::Model model;
    model.yaml_path = ped.yaml_file;
    model.name = id;
    model.ns = id;
    model.pose.x = ped.pos.x;
    model.pose.y = ped.pos.y;
    model.pose.theta = 0.0;
    flatland_msg.push_back(model);
  }

  return flatland_msg;
}

bool SceneServices::cb_addWalls(pedsim_srvs::SpawnWalls::Request &request,
                             pedsim_srvs::SpawnWalls::Response &response)
{
  bool success = true;
  for (auto &wall : request.walls)
  {
    auto sceneWall = new Wall(wall.start.x, wall.start.y, wall.end.x, wall.end.y, WallLayer::WORLD);
    success &= SCENE.addWall(sceneWall);
    walls.push_back(sceneWall);
  }

  response.success = success;
  return success;
}

bool SceneServices::cb_clearWalls(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  ROS_DEBUG("Clearing walls.");
  bool success = true;
  for (auto wall : walls)
  {
    success &= SCENE.removeWall(wall);
  }
  walls.clear();
  response.success = success;
  return success;
}

bool SceneServices::cb_reset(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  std_srvs::SetBool::Request req;
  std_srvs::SetBool::Response res;

  return cb_removeAllPeds(req, res) & cb_removeAllObstacles(request, response) & cb_clearWalls(request, response);
}

bool SceneServices::cb_moveAgentClustersInPedsim(pedsim_srvs::MovePeds::Request &request,
                                              pedsim_srvs::MovePeds::Response &response)
{
  SCENE.episode = request.episode;
  SCENE.moveClusters(request.episode);
  // static obstacle infos are updated every episode too
  // SCENE.removeAllWalls();
  response.success = true;
  return true;
}

bool SceneServices::cb_registerRobot(pedsim_srvs::RegisterRobot::Request &request, pedsim_srvs::RegisterRobot::Response &response)
{

  Robot *robot = new Robot(
      request.name,
      request.odom_topic,
      this->nh_);

  SCENE.addRobot(robot);

  response.success = true;
  return true;
}

std::vector<pedsim::id> SceneServices::generateAgentIds(pedsim::id base, int n = 1)
{

  if (n == 1)
    return std::vector<pedsim::id>({base});

  std::vector<pedsim::id> ids;
  for (int i = 0; i < n; i++)
  {
    ids.push_back(base + "_" + std::to_string(i));
    agents_index_++;
  }
  return ids;
}

bool SceneServices::removeModelsInFlatland(std::vector<pedsim::id> model_names)
{
  ROS_DEBUG("deleting %ld models", model_names.size());
  flatland_msgs::DeleteModels msg;
  msg.request.name = model_names;

  // check validity of client
  while (!delete_models_client_.isValid())
  {
    ROS_WARN("Reconnecting delete_models_client_-server....");
    delete_models_client_.waitForExistence(ros::Duration(5.0));
    delete_models_client_ = nh_.serviceClient<flatland_msgs::DeleteModels>(delete_models_topic_, true);
  }

  delete_models_client_.call(msg);

  if (!msg.response.success)
  {
    ROS_WARN("Failed to delete all %d models. Maybe a few were deleted. Flatland response: %s", int(msg.request.name.size()), msg.response.message.c_str());
    return false;
  }

  return true;
}

bool SceneServices::spawnModelsInFlatland(std::vector<flatland_msgs::Model> models)
{
  flatland_msgs::SpawnModels msg;
  msg.request.models = models;

  // check validity of client
  while (!spawn_models_client_.isValid())
  {
    ROS_WARN("Reconnecting spawn_models_client_-server....");
    spawn_models_client_.waitForExistence(ros::Duration(5.0));
    spawn_models_client_ = nh_.serviceClient<flatland_msgs::SpawnModels>(spawn_models_topic_, true);
  }

  spawn_models_client_.call(msg);

  if (!msg.response.success)
  {
    ROS_WARN("Failed to respawn models. Flatland response: %s", msg.response.message.c_str());
    return false;
  }

  return true;
}

// bool SceneServices::respawnModelsInFlatland(std::vector<pedsim::id> old_model_names, std::vector<flatland_msgs::Model> new_models) {
//   flatland_msgs::RespawnModels msg;
//   msg.request.old_model_names = old_model_names;
//   msg.request.new_models = new_models;

//   // check validity of client
//   while (!respawn_models_client_.isValid()) {
//     ROS_WARN("Reconnecting respawn_models_client_-server....");
//     respawn_models_client_.waitForExistence(ros::Duration(5.0));
//     respawn_models_client_ = nh_.serviceClient<flatland_msgs::RespawnModels>(respawn_models_topic_, true);
//   }

//   respawn_models_client_.call(msg);

//   if (!msg.response.success) {
//     ROS_WARN("Failed to respawn models. Flatland response: %s", msg.response.message.c_str());
//     return false;
//   }

//   return true;
// }

std::vector<Wall *> SceneServices::getWallsFromFlatlandModel(pedsim_msgs::Obstacle obstacle, double yaw)
{
  std::vector<Wall *> new_walls;
  YAML::Node model = YAML::LoadFile(obstacle.yaml_path);
  auto bodies = model["bodies"];
  if (bodies.size() < 1)
  {
    ROS_WARN("getWallsFromFlatlandModel(): No body found in yaml node.");
    return std::vector<Wall *>();
  }

  auto footprints = bodies[0]["footprints"];
  if (footprints.size() < 1)
  {
    ROS_WARN("getWallsFromFlatlandModel(): No footprint found in body.");
    return std::vector<Wall *>();
  }

  auto footprint = footprints[0];
  auto type = footprint["type"];
  if (type == NULL)
  {
    ROS_WARN("getWallsFromFlatlandModel(): No type found in footprint.");
    return std::vector<Wall *>();
  }

  auto pos = obstacle.pose.position;
  if (type.as<std::string>() == "polygon")
  {
    auto points = footprint["points"];
    auto nPoints = points.size();
    for (size_t i = 0; i < nPoints; i++)
    {
      auto start = Ped::Tvector(points[i][0].as<float>(), points[i][1].as<float>()).rotated(yaw);
      auto end = Ped::Tvector(points[(i + 1) % nPoints][0].as<float>(), points[(i + 1) % nPoints][1].as<float>()).rotated(yaw);
      Wall *wall = new Wall(
        pos.x + start.x,
        pos.y + start.y,
        pos.x + end.x,
        pos.y + end.y,
        WallLayer::OBSTACLE
      );
      new_walls.push_back(wall);
    }
  }
  else if (type.as<std::string>() == "circle")
  {
    // TODO handle circles properly
    auto center = footprint["center"];
    auto cx = center[0].as<float>();
    auto cy = center[1].as<float>();
    auto r = footprint["radius"].as<float>();

    std::vector<Ped::Tvector> corners = {
        Ped::Tvector(pos.x + cx + r, pos.y + cy + r).rotated(yaw),
        Ped::Tvector(pos.x + cx + r, pos.y + cy - r).rotated(yaw),
        Ped::Tvector(pos.x + cx - r, pos.y + cy - r).rotated(yaw),
        Ped::Tvector(pos.x + cx - r, pos.y + cy + r).rotated(yaw)};

    auto nCorners = corners.size();
    for (std::size_t i = 0; i < nCorners; i++)
    {
      auto from = corners.at(i);
      auto to = corners.at((i + 1) % nCorners);
      new_walls.push_back(
        new Wall(
          from.x,
          from.y,
          to.x,
          to.y,
          WallLayer::OBSTACLE
        )
      );
    }
  }
  return new_walls;
}




// remove all agents and return a list of their names
std::vector<pedsim::id> SceneServices::removePedsInPedsim()
{
  // Remove all waypoints
  auto waypoints = SCENE.getWaypoints();
  for (auto waypoint : waypoints.values())
  {
    if (!waypoint->isInteractive())
    {
      SCENE.removeWaypoint(waypoint);
    }
  }

  // Remove all agents
  std::vector<pedsim::id> names;
  QList<Agent *> agents = SCENE.getAgents();
  for (Agent *a : agents)
  {
    // don't remove robot
    if (a->getType() == Ped::Tagent::ROBOT)
    {
      continue;
    }
    names.push_back(a->id);
    SCENE.removeAgent(a);
  }

  // ROS_INFO("staticid in scene_services.cpp: %d", Ped::Tagent::staticid);
  agents_index_ = 1;

  return names;
}
