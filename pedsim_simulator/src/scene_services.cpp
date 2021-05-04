/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically and add static obstacle spawning. 
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 * @author Junhui Li
 * @date 24.3.2021
 **/

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/scene_services.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/SetBool.h>
#include <flatland_msgs/SpawnModels.h>
#include <flatland_msgs/DeleteModels.h>
#include <flatland_msgs/RespawnModels.h>
#include <iostream>
#include <ros/package.h>

int SceneServices::agents_index_ = 1;

SceneServices::SceneServices(){
  // pedsim services
  respawn_peds_service_ = nh_.advertiseService("pedsim_simulator/respawn_peds", &SceneServices::respawnPeds, this);
  spawn_peds_service_ = nh_.advertiseService("pedsim_simulator/spawn_peds", &SceneServices::spawnPeds, this);
  remove_all_peds_service_ = nh_.advertiseService("pedsim_simulator/remove_all_peds", &SceneServices::removeAllPeds, this);
  reset_peds_service_ = nh_.advertiseService("pedsim_simulator/reset_all_peds", &SceneServices::resetPeds, this);
  add_obstacle_service_= nh_.advertiseService("pedsim_simulator/add_obstacle", &SceneServices::addStaticObstacles, this);
  move_peds_service_= nh_.advertiseService("pedsim_simulator/move_peds", &SceneServices::moveAgentClustersInPedsim, this);
  
  //flatland service clients
  spawn_models_topic_ = ros::this_node::getNamespace() + "/spawn_models";
  spawn_models_client_ = nh_.serviceClient<flatland_msgs::SpawnModels>(spawn_models_topic_, true);
  respawn_models_topic_ = ros::this_node::getNamespace() + "/respawn_models";
  respawn_models_client_ = nh_.serviceClient<flatland_msgs::RespawnModels>(respawn_models_topic_, true);
  delete_models_topic_ = ros::this_node::getNamespace() + "/delete_models";
  delete_models_client_ = nh_.serviceClient<flatland_msgs::DeleteModels>(delete_models_topic_, true);
}

bool SceneServices::spawnPeds(pedsim_srvs::SpawnPeds::Request &request, pedsim_srvs::SpawnPeds::Response &response) {
  flatland_msgs::SpawnModels srv;

  for (int ped_i = 0; ped_i < (int) request.peds.size(); ped_i++) {
    pedsim_msgs::Ped ped = request.peds[ped_i];
    std::vector<int> new_agent_ids = generateAgentIds(ped.number_of_peds);

    // add ped to pedsim
    AgentCluster* agentCluster = addAgentClusterToPedsim(ped, new_agent_ids);

    // add flatland models to spawn_models service request
    std::vector<flatland_msgs::Model> new_models = getFlatlandModelsFromAgentCluster(agentCluster, ped.yaml_file, new_agent_ids);
    srv.request.models.insert(srv.request.models.end(), new_models.begin(), new_models.end());
  }

  // make sure client is valid
  while (!spawn_models_client_.isValid()) {
    ROS_WARN("Reconnecting to flatland spawn_models service...");
    spawn_models_client_.waitForExistence(ros::Duration(5.0));
    spawn_models_client_ = nh_.serviceClient<flatland_msgs::SpawnModels>(spawn_models_topic_, true);
  }

  // call spawn_models service
  spawn_models_client_.call(srv);
  if (srv.response.success) {
    response.success = true;
    ROS_DEBUG("Successfully called flatland spawn_models service");
  } else {
    response.success = false;
    ROS_ERROR("Failed to spawn all %ld agents", request.peds.size());
  }
  
  return true;
}

bool SceneServices::respawnPeds(pedsim_srvs::SpawnPeds::Request &request,
                                pedsim_srvs::SpawnPeds::Response &response){
  ROS_INFO("called respawnPeds() with %ld peds", request.peds.size());

  std_srvs::SetBool::Request request_;
  std_srvs::SetBool::Response response_;
  bool res = removeAllPeds(request_, response_);
  if (!res) {
    response.success = false;
    return false;
  }

  res = spawnPeds(request, response);
  if (!res) {
    response.success = false;
    return false;
  }

  response.success = true;
  return true;
}

bool SceneServices::removeAllPeds(std_srvs::SetBool::Request &request,
                                std_srvs::SetBool::Response &response){
  flatland_msgs::DeleteModels srv;
  srv.request.name = removePedsInPedsim();

  // Deleting pedestrian in flatland
  while (!delete_models_client_.isValid()) {
    ROS_WARN("Reconnecting delete_models_client_-server....");
    delete_models_client_.waitForExistence(ros::Duration(5.0));
    delete_models_client_ = nh_.serviceClient<flatland_msgs::DeleteModels>(delete_models_topic_, true);
  }

  delete_models_client_.call(srv);

  if (!srv.response.success) {
    ROS_ERROR("Failed to delete all %d agents. Maybe a few were deleted. %s", int(srv.request.name.size()), srv.response.message.c_str());
  }

  response.success = true;
  return true;
}

// remove all agents and return a list of their names
std::vector<std::string> SceneServices::removePedsInPedsim() {
  // Remove all waypoints
  auto waypoints = SCENE.getWaypoints();
  for (auto waypoint : waypoints.values()) {
    SCENE.removeWaypoint(waypoint);
  }

  // Remove all agents
  std::vector<std::string> names;
  QList<Agent*> agents = SCENE.getAgents();
  for (Agent* a : agents) {
    names.push_back(a->agentName);
    SCENE.removeAgent(a);
  }

  // reset agent counters
  Ped::Tagent::staticid = 1;
  agents_index_ = 1;

  return names;
}

bool SceneServices::resetPeds(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  if (request.data) {
    QList<Agent*> agents = SCENE.getAgents();
    for (Agent* a : agents) {
      a->reset();
    }
  }

  response.success = true;
  return true;
}

AgentCluster* SceneServices::addAgentClusterToPedsim(pedsim_msgs::Ped ped, std::vector<int> ids) {
  // create agentcluster
  const double x = ped.pos.x;
  const double y = ped.pos.y;
  const int n = ped.number_of_peds;
  AgentCluster* agentCluster = new AgentCluster(x, y, n, ids);

  // set distribution
  const double dx = 2;
  const double dy = 2;
  agentCluster->setDistribution(dx, dy);

  // set type
  const int type = ped.type;
  agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));

  // agentCluster->vmax = ped.vmax;
  agentCluster->chattingProbability = ped.chatting_probability;

  int waypoint_mode = ped.waypoint_mode;
  agentCluster->waypoint_mode = static_cast<Agent::WaypointMode>(waypoint_mode);

  // set force factors
  // agentCluster->forceFactorDesired = ped.force_factor_desired;
  // agentCluster->forceFactorObstacle = ped.force_factor_obstacle;
  // agentCluster->forceFactorSocial = ped.force_factor_social;

  // add waypoints to agentcluster and scene
  for(int i = 0; i < (int) ped.waypoints.size(); i++){
    QString id;
    id.sprintf("%d_%d", ped.id, i);
    const double x = ped.waypoints[i].x;
    const double y = ped.waypoints[i].y;
    const double r = ped.waypoints[i].z;
    AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
    w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
    SCENE.addWaypoint(w);
    agentCluster->addWaypoint(w);
  }

  // add agentcluster to scene
  SCENE.addAgentCluster(agentCluster);

  return agentCluster;
}

std::vector<flatland_msgs::Model> SceneServices::getFlatlandModelsFromAgentCluster(AgentCluster* agentCluster, std::string yaml_file, std::vector<int> ids){
  std::vector<flatland_msgs::Model> flatland_msg;

  for (int i = 0; i < agentCluster->getCount(); i++){
    flatland_msgs::Model model;
    model.yaml_path = yaml_file;
    model.name = "person_" + std::to_string(ids[i]);
    model.ns = "pedsim_agent_" + std::to_string(ids[i]);
    model.pose.x = agentCluster->getPosition().x;
    model.pose.y = agentCluster->getPosition().y;
    model.pose.theta = 0.0;
    flatland_msg.push_back(model);
  }  

  return flatland_msg;
}

bool SceneServices::addStaticObstacles(pedsim_srvs::SpawnObstacle::Request &request,
                                      pedsim_srvs::SpawnObstacle::Response &response){
  int k = (int)request.staticObstacles.obstacles.size();  
  for (int i = 0; i <= k; i++){
    pedsim_msgs::LineObstacle obstacle=request.staticObstacles.obstacles[i];
    // Obstacle obstacle=request.staticObstacles.obstacles[i];
    Obstacle *o = new Obstacle(obstacle.start.x, obstacle.start.y, obstacle.end.x, obstacle.end.y);
    SCENE.addObstacle(o);
  }

  response.finished=true;
  return true;
}

bool SceneServices::moveAgentClustersInPedsim(pedsim_srvs::MovePeds::Request &request,
                                pedsim_srvs::MovePeds::Response &response){
  SCENE.moveClusters(request.episode);
  // static obstacle infos are updated every episode too
  SCENE.removeAllObstacles();
  response.finished=true;
  return true;
}

std::vector<int> SceneServices::generateAgentIds(int n) {
  std::vector<int> ids;
  for (int i = 0; i < n; i++) {
    ids.push_back(agents_index_);
    agents_index_++;
  }
  return ids;
}
