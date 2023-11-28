/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically.
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#ifndef _scene_service_h_
#define _scene_service_h_

#include <ros/ros.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/waypointplanner.h>
#include <pedsim_srvs/SpawnPeds.h>
#include <pedsim_srvs/SpawnWalls.h>
#include <pedsim_srvs/SpawnObstacles.h>
#include <pedsim_srvs/MovePeds.h>
#include <pedsim_srvs/RegisterRobot.h>
#include <flatland_msgs/Model.h>
#include <pedsim_msgs/Ped.h>
#include <pedsim_msgs/Wall.h>
#include <pedsim_msgs/Walls.h>
#include <pedsim_msgs/Waypoint.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>


/**
 * This class provides services to spawn and remove pedestrians dynamically.
 */
class SceneServices
{
  // Constructor and Destructor
public:
  SceneServices();
  virtual ~SceneServices() = default;

  ros::ServiceServer reset_service_;

  ros::ServiceServer spawn_peds_service_;
  ros::ServiceServer respawn_peds_service_;
  ros::ServiceServer move_peds_service_;
  ros::ServiceServer remove_all_peds_service_;
  ros::ServiceServer reset_all_peds_service_;
  // ros::ServiceServer remove_all_peds_behavior_modelling_service_;
  
  ros::ServiceServer add_walls_service_;
  ros::ServiceServer clear_walls_service_;
  
  ros::ServiceServer spawn_obstacles_service_;
  ros::ServiceServer respawn_obstacles_service_;
  ros::ServiceServer remove_all_obstacles_service_;
  
  ros::ServiceServer register_robot_service_;

  bool env_is_flatland = true;
  static int agents_index_;
  static int static_obstacles_index_;
  static std::vector<pedsim::id> static_obstacle_names_;

  /**
   * @brief Spawns pedestrian in pedsim and flatland.
   */
  bool cb_spawnPeds(pedsim_srvs::SpawnPeds::Request &request, pedsim_srvs::SpawnPeds::Response &response);

  /**
   * @brief Removes all pedestrians in flatland.
   */
  bool cb_removeAllPeds(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

  /**
   * @brief Resets all pedestrians to their initial position and state
   */
  bool cb_resetPeds(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  /**
   * @brief Repawn interactive obstacles
   */
  bool cb_respawnObstacles(pedsim_srvs::SpawnObstacles::Request &request, pedsim_srvs::SpawnObstacles::Response &response);

  /**
   * @brief Spawn interactive obstacles
   */
  bool cb_spawnObstacles(pedsim_srvs::SpawnObstacles::Request &request, pedsim_srvs::SpawnObstacles::Response &response);

  /**
   * @brief Remove all interactive obstacles
   */
  bool cb_removeAllObstacles(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  /**
   * @brief Respawning means reusing objects from previous tasks.
   * It is a more efficient way to setup a task during learning.
   */
  bool cb_respawnPeds(pedsim_srvs::SpawnPeds::Request &request,
                   pedsim_srvs::SpawnPeds::Response &response);

  bool cb_moveAgentClustersInPedsim(pedsim_srvs::MovePeds::Request &request,
                                 pedsim_srvs::MovePeds::Response &response);

  /**
   * @brief Adding static obstacles to pedsim.
   */
  bool cb_addWalls(pedsim_srvs::SpawnWalls::Request &request,
                          pedsim_srvs::SpawnWalls::Response &response);

  bool cb_clearWalls(std_srvs::Trigger::Request &request,
                          std_srvs::Trigger::Response &response);

  bool cb_reset(std_srvs::Trigger::Request &request,
                          std_srvs::Trigger::Response &response);

  bool cb_registerRobot(pedsim_srvs::RegisterRobot::Request &request,
                          pedsim_srvs::RegisterRobot::Response &response);

protected:
  ros::NodeHandle nh_;

private:
  std::vector<pedsim::id> removePedsInPedsim();
  bool addAgentClusterToPedsim(pedsim_msgs::Ped ped, std::vector<pedsim::id> ids);
  std::vector<flatland_msgs::Model> getFlatlandModels(pedsim_msgs::Ped ped, std::vector<pedsim::id> ids);
  std::vector<pedsim::id> generateAgentIds(pedsim::id base, int n);
  bool removeModelsInFlatland(std::vector<pedsim::id> model_names);
  bool spawnModelsInFlatland(std::vector<flatland_msgs::Model> models);
  // bool respawnModelsInFlatland(std::vector<pedsim::id> old_model_names, std::vector<flatland_msgs::Model> new_models);
  bool removeAllReferencesToInteractiveObstacles();
  bool removeAllInteractiveObstaclesFromPedsim();
  bool removeAllInteractiveObstaclesFromFlatland();
  std::vector<Wall *> getWallsFromFlatlandModel(pedsim_msgs::Obstacle obstacle, double yaw);
  int stringToEnumIndex(pedsim::id str, std::vector<pedsim::id> values);

  std::string spawn_models_topic_;
  ros::ServiceClient spawn_models_client_;

  // pedsim::id respawn_models_topic_;
  // ros::ServiceClient respawn_models_client_;

  std::string delete_models_topic_;
  ros::ServiceClient delete_models_client_;

  std::vector<Wall *> walls;
};

#endif /* _scene_service_h_ */
