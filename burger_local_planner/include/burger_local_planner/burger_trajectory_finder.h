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

const float L = 0.160;              ///< distance between middle of wheels
const float SPEED_DELTA_MAX = 0.01; ///< Max speed variation between two rounds (only when increasing)
const float SPEED_MAX = 0.18;       ///< Max linear speed (m/s)
const float SPEED_MIN = 0.05;       ///< Min linear speed
const float OMEGA_MAX = 2.840;      ///< Mas angular velocit (rad/s)
const float PI = 3.1415926535;
const float WHEEL_MAX_SPEED = SPEED_MAX * 0.9;
const float SPEED_FACTOR = 0.15;

const float Kp = 0.5;
const float Ki = 0.1;
const float Kd = 0.00;

class BurgerTrajectoryFinder
{
public:
  BurgerTrajectoryFinder();

  base_local_planner::Trajectory findBestPath(Eigen::Vector3f goal, Eigen::Vector3f pose, float deltaTime, bool isLastGoal, bool allowReverse);
  Eigen::Vector3f computeNewPositions(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, double dt);
  void initialize();
  void initialize(Eigen::Vector3f initialPose);

private:
  float xPos_[2], yPos_[2], thetaPos_[2];
  float xGoal_, yGoal_, thetaGoal_;
  float linearSpeed_[2], omega_;
  float speedRight_, speedLeft_;

  float omegai_[3], omegap_[3], omegad_[3];
};
}

#endif
