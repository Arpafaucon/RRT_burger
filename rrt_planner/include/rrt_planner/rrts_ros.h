/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <array>
#include <list>
#include <iostream>	
#include <fstream>

#include "system_ros.h"
#include "rrts.hpp"

#ifndef RRTS_ROS_H
#define RRTS_ROS_H

#define SPACE_DIM 2

using std::string;

typedef RRTstar::Planner<Burger2D::State2, Burger2D::Trajectory, Burger2D::System> planner_t;
typedef RRTstar::Vertex<Burger2D::State2, Burger2D::Trajectory, Burger2D::System> vertex_t;
typedef std::array<double, SPACE_DIM> point_t;
typedef std::array<double, 2 * SPACE_DIM> surface_t;
typedef std::pair<planner_t *, Burger2D::System *> experience_t;
typedef Burger2D::State2 state_t;


namespace rrts_burger
{

class RRTPlanner : public nav_core::BaseGlobalPlanner
{
  public:
	RRTPlanner();

	/** overridden classes from interface nav_core::BaseGlobalPlanner **/
	void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped &start,
				  const geometry_msgs::PoseStamped &goal,
				  std::vector<geometry_msgs::PoseStamped> &plan);
	void saveExpToFile(std::ofstream& out);

  private:
	costmap_2d::Costmap2D *costmap_;
	string frame_id_;
	bool initialized_;
	Burger2D::System* system_ = NULL;
	planner_t* planner_ = NULL;

	void clearRobotCell(const unsigned int mx, const unsigned int my);
};
};
#endif
