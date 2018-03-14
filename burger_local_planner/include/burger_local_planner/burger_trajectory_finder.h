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
const float PI = 3.1415926535;

/**
 * \brief Internal engine for trajectories
 */
class BurgerTrajectoryFinder
{
public:
  /**
   * \brief distance between middle of wheels
   */
  float _L_;
  /**
   * \brief Max speed variation between two rounds (only when increasing)
   */
  float _ACC_MAX_;
  /**
   * \brief Max linear speed (m/s)
   */
  float _SPEED_MAX_;
  /**
   * \brief Min linear speed
   */
  float _SPEED_MIN_;
  /**
   * \brief Max angular velocity (rad/s)
   */
  float _OMEGA_MAX_;
  /**
   * \brief max linear speed of one wheel
   */
  float _WHEEL_MAX_SPEED_;
  /**
   * \brief Multiplicative factor applied to computed output speed
   */
  float _SPEED_FACTOR_;
  /**
   * \brief PID param : proportionnal
   */
  float _Kp_;
  /**
   * \brief PID param : integrator
   */
  float _Ki_;
  /**
   * \brief PID param : derivative
   */
  float _Kd_;


  BurgerTrajectoryFinder();

  base_local_planner::Trajectory findBestPath(Eigen::Vector3f goal, Eigen::Vector3f pose, float deltaTime, bool isLastGoal, bool allowReverse);
  Eigen::Vector3f computeNewPositions(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, double dt);
  void initialize();
  void initialize(Eigen::Vector3f initialPose);
  /**
   * \brief sets up internal params
   * Allows configuration from external sources : files, rosparam
   */
  bool setInternalParams(float L, float ACC_MAX, float SPEED_MAX, float SPEED_MIN, float OMEGA_MAX, float SPEED_FACTOR, float pid_Kp, float pid_Ki, float pid_Kd);

private:
  float xPos_[2], yPos_[2], thetaPos_[2];
  float xGoal_, yGoal_, thetaGoal_;
  float linearSpeed_[2], omega_;
  float speedRight_, speedLeft_;

  float omegai_[3], omegap_[3], omegad_[3];
  float lastRequestedSpeed_;
};
}

#endif
