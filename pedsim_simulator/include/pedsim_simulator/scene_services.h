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
#include <pedsim_srvs/SpawnPeds.h>
#include <flatland_msgs/Model.h>
#include <pedsim_msgs/Ped.h>
#include <std_srvs/SetBool.h>

  /**
   * This class provides services to spawn and remove pedestrians dynamically.
   */
class SceneServices {
  // Constructor and Destructor
 public:
  SceneServices();
  virtual ~SceneServices() = default;

 protected:
   ros::NodeHandle nh_;

 private:
  ros::NodeHandle nh;
  int last_id_;                           //Keeping track of cluster id, that increases in pedsim automatically
  std::string flatland_path_;             //path to flatland package

    /**
    * @brief Removing all pedestrians in pedsim.
    * @return corresponding flatland namespaces of pedestrians
    */
  std::vector<std::string> removePedsInPedsim();

    /**
    * @brief Adding pedestrian to pedsim.
    * @return corresponding flatland model-message
    */
  std::vector<flatland_msgs::Model> addAgentClusterToPedsim(pedsim_msgs::Ped ped);

   /**
    * spawn_ped_service_ + spawnPed
    * @brief Spawns pedestrian in pedsim and flatland.
    */
  ros::ServiceServer spawn_ped_service_;                
  bool spawnPed(pedsim_srvs::SpawnPeds::Request &request,
                                pedsim_srvs::SpawnPeds::Response &response);

   /**
    * remove_all_peds_service_ + removeAllPeds
    * @brief Removes all pedestrians in flatland.
    */
  ros::ServiceServer remove_all_peds_service_;          
  bool removeAllPeds(std_srvs::SetBool::Request &request,
                                std_srvs::SetBool::Response &response);

   /**
    * respawn_peds_service_ + respawnPeds
    * @brief Respawning means reusing objects from previous tasks
    * It is a more efficient way to setup a task during learning.
    */
  ros::ServiceServer respawn_peds_service_;          
  bool respawnPeds(pedsim_srvs::SpawnPeds::Request &request,
                                pedsim_srvs::SpawnPeds::Response &response);

  std::string spawn_model_topic;
  ros::ServiceClient spawn_agents_;             //Service client to spawn agent in flatland
  std::string respawn_model_topic;
  ros::ServiceClient respawn_agents_;            //Service client to spawn agent in flatland
  std::string delete_model_topic;
  ros::ServiceClient delete_agents_;            // Service client to remove agent in flatland.
};

#endif /* _scene_service_h_ */
