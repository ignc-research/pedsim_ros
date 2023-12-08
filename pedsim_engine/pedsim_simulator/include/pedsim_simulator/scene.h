/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#ifndef _scene_h_
#define _scene_h_

#include <pedsim/ped_scene.h>
#include <pedsim/ped_vector.h>
#include <QMap>
#include <QObject>
#include <QRectF>

#include <pedsim_simulator/utilities.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <pedsim_simulator/element/obstacle.h>

// Forward Declarations
class QGraphicsScene;
class Agent;
class Wall;
class Waypoint;
class AttractionArea;
class AgentCluster;
class AgentGroup;
class WaitingQueue;
class Robot;

struct SpawnArea {
  double x, y;
  int n;
  int dx, dy;
  std::vector<QString> waypoints;

  SpawnArea(const double xx, const double yy, const int nn, const int dxi,
            const int dyi)
      : x{xx}, y{yy}, n{nn}, dx{dxi}, dy{dyi} {}
};

class Scene : public QObject, protected Ped::Tscene {
  Q_OBJECT

 public:
  Scene(QObject* parent = 0);
  virtual ~Scene();

// Singleton Design Pattern
#define SCENE Scene::getInstance()
 protected:
  static Scene* instance;
  ros::NodeHandle nh_;

 public:
  static Scene& getInstance();
  void setTimeStepSize(float t);

  // Signals
 signals:
  void aboutToStart();
  void aboutToMoveAgents();
  void movedAgents();
  void sceneTimeChanged(double time);

  // → added/removed elements
  void agentAdded(pedsim::id id);
  void agentRemoved(pedsim::id id);
  void wallAdded(pedsim::id id);
  void wallRemoved(pedsim::id id);
  void waypointAdded(pedsim::id id);
  void waypointRemoved(pedsim::id id);
  void agentClusterAdded(pedsim::id id);
  void agentClusterRemoved(pedsim::id id);
  void waitingQueueAdded(QString name);
  void waitingQueueRemoved(QString name);
  void attractionAdded(QString name);
  void attractionRemoved(QString name);

  // Slots
 public slots:
  void moveAllAgents();
 protected slots:
  void cleanupScene();

  // Methods
 public:
  void clear();

  QRectF itemsBoundingRect() const;

  // → elements
  const QList<Robot*>& getRobots() const;
  const QList<Agent*>& getAgents() const;
  Agent* getAgent(pedsim::id id) const;
  Agent* getAgentById(pedsim::id idIn) const;
  QList<AgentGroup*> getGroups();
  QMap<QString, AttractionArea*> getAttractions();
  const QList<Wall*>& getWalls() const;
  const QList<Obstacle*>& getObstacles() const;
  const QMap<QString, Waypoint*>& getWaypoints() const;
  const QMap<QString, AttractionArea*>& getAttractions() const;
  Waypoint* getWaypointById(pedsim::id idIn) const;
  Waypoint* getWaypointByName(const QString& nameIn) const;
  WaitingQueue* getWaitingQueueByName(const QString& nameIn) const;
  const QList<AgentCluster*>& getAgentClusters() const;
  AttractionArea* getAttractionByName(const QString& nameIn) const;
  AttractionArea* getClosestAttraction(const Ped::Tvector& positionIn,
                                       double* distanceOut = nullptr) const;
  std::vector<SpawnArea*> getSpawnAreas() const { return spawn_areas; }
  void addSpawnArea(SpawnArea* sa) { spawn_areas.emplace_back(sa); }

  // → simulation time
  double getTime() const;
  bool hasStarted() const;

 protected:
  void dissolveClusters();

 public:
  virtual bool addAgent(Agent* agent);
  virtual bool addWall(Wall* wall);
  virtual bool addObstacle(Obstacle* obstacle);
  virtual bool addWaypoint(Waypoint* waypoint);
  virtual bool addAgentCluster(AgentCluster* clusterIn);
  virtual bool addWaitingQueue(WaitingQueue* queueIn);
  virtual bool addAttraction(AttractionArea* attractionIn);
  virtual bool addRobot(Robot* agent);
  virtual bool removeAgent(Agent* agent);
  virtual bool removeWall(Wall* wall);
  virtual bool removeObstacle(Obstacle* obstacle);
  bool removeWaypoint(QString name);
  virtual bool removeWaypoint(Waypoint* waypoint);
  virtual bool removeAgentCluster(AgentCluster* clusterIn);
  virtual bool removeWaitingQueue(WaitingQueue* queueIn);
  virtual bool removeAttraction(AttractionArea* attractionInIn);
  virtual bool moveClusters(int i);
  virtual bool removeAllObstacles();

  virtual std::set<const Ped::Tagent*> getNeighbors(double x, double y,
                                                    double maxDist);

  bool getClosestWall(Ped::Tvector pos_in, Ped::Twaypoint* closest);
  void arenaGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  std::vector<int> odomPosToMapIndex(Ped::Tvector pos);
  bool isOccupied(Ped::Tvector pos);

  // wall cell locations
  std::vector<Location> wall_cells_;

  std::vector<std::string> agent_types {"adult", "child", "elder", "forklift", "servicerobot", "robot"};
  std::vector<float> agent_radius {0.5, 0.5, 0.5, 1.8, 1.0, 1.0};  // used for wall force calculation
  std::vector<std::string> wall_types {"areawaypoint", "pointwaypoint", "shelf"};
  std::vector<float> wall_radius {1.0, 1.0, 1.0};  // used for wall force calculation
  std::vector<std::string> start_up_modes {"default", "wait_timer", "trigger_zone"};

  Agent* robot;
  int episode;
  bool guideActive;
  bool followerActive;
  bool serviceRobotExists;

  Ped::Tvector* arenaGoal;
  ros::Subscriber arenaGoalSub;

  // Attributes
 protected:
  QList<Agent*> agents;
  QList<Robot*> robots;
  QList<Wall*> walls;
  QList<Obstacle*> obstacles;
  QMap<QString, Waypoint*> waypoints;
  QMap<QString, AttractionArea*> attractions;
  QList<AgentCluster*> agentClusters;
  QList<AgentGroup*> agentGroups;

  std::vector<SpawnArea*> spawn_areas;

  // → simulated time
  double sceneTime;
};

#endif
