#ifndef BURGER_LOCAL_PLANNER_BURGER_TRAJECTORY_FINDER_H_
#define BURGER_LOCAL_PLANNER_BURGER_TRAJECTORY_FINDER_H_

#include <vector>
#include <Eigen/Core>

#include <burger_local_planner/BurgerPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>
#include <pcl_ros/publisher.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
// #include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
// #include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
// #include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
// #include <base_local_planner/twirling_cost_function.h>
// #include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace burger_local_planner
{

const float L = 0.15;              ///< distance between middle of wheels
const float SPEED_DELTA_MAX = 0.1; ///< Max speed variation between two rounds (only when increasing)
const float SPEED_MAX = 1.0;       ///< Max linear speed
const float SPEED_MIN = 0.05;      ///< Min linear speed
const float OMEGA_MAX = 10.0;

const float WHEEL_MAX_SPEED = 50.0;

const float Kp = 2.0;
const float Ki = 1.0;
const float Kd = 0.0;

class BurgerTrajectoryFinder
{
public:
  base_local_planner::Trajectory findBestPath(Eigen::Vector3f goal, Eigen::Vector3f pose, float deltaTime);
  Eigen::Vector3f computeNewPositions(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, double dt);
  void initialize();
  void initialize(Eigen::Vector3f initialPose);

private:
  float xPos_[2], yPos_[2], thetaPos_[2];
  float xGoal_, yGoal_, thetaGoal_;
  float linearSpeed_[2], omega_[2];
  float speedRight_, speedLeft_;

  float omegai_[3], omegap_[3], omegad_[3];
}
}

#endif
