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

// these are the default values
#define WAYPOINT_DISTANCE 0.1
#define MAX_ITERATION 5000
#define PUBLISH_MARKERS true
#define GOAL_BIAS 0.1
#define GOAL_SIZE 0.1
#define RRTS_GAMMA 0.5
#define VERBOSE true
#define ROBOT_RADIUS 0.05
#define DISCRETIZATION_STEP 0.01

using std::string;

typedef RRTstar::Planner<Burger2D::State2, Burger2D::Trajectory, Burger2D::System> planner_t;
typedef RRTstar::Vertex<Burger2D::State2, Burger2D::Trajectory, Burger2D::System> vertex_t;
typedef std::array<double, SPACE_DIM> point_t;
typedef std::array<double, 2 * SPACE_DIM> surface_t;
typedef std::pair<planner_t *, Burger2D::System *> experience_t;
typedef Burger2D::State2 state_t;

namespace rrts_burger
{

/** 
 * \brief RRTS GlobalPlanner implementation
 */
class RRTPlanner : public nav_core::BaseGlobalPlanner
{
  public:
	/**
	 * \brief constructor
	 */
	RRTPlanner();

	/**
	 * \brief sets up the planner
	 * 
	 * GlobalPlanner interface.
	 * Builds necessary objects, and register the two marker topics
	 * 
	 * \param name name of the planner
	 * \param costmap_ros costmap to use
	 */
	void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
	/**
	 * \brief Computes a path
	 * 
	 * Calls the RRTS engine
	 * 
	 * \param start starting point
	 * \param goal destination point
	 * \param plan list of waypoints (to fill)
	 */
	bool makePlan(const geometry_msgs::PoseStamped &start,
				  const geometry_msgs::PoseStamped &goal,
				  std::vector<geometry_msgs::PoseStamped> &plan);
	
	/**
	 * \brief writes waypoints and tree to stream
	 * 
	 * Used to write .sol files
	 * 
	 * \param out the pre-opened and filled output stream to the file
	 */
	void saveExpToFile(std::ofstream &out);

	/**
	 * \brief publish waypoints and RRTS tree as markers for RVIZ
	 * 
	 * \param stateList list of waypoints as returned by the planner
	 * \param verticesList list of vertices. If NULL, no tree is published
	 */
	bool publishStatesRviz(std::list<double *> &stateList, std::list<vertex_t *> *verticesList);

  private:
	/**
	 * \brief internal costmap
	 */
	costmap_2d::Costmap2D *costmap_;
	/**
	 * \brief the world tf frame
	 */
	string frame_id_;
	/**
	 * \brief internal state of the planner
	 */
	bool initialized_;
	/**
	 * \brief the last system used
	 * 
	 * Helpful in peripheral functions (publishStateRviz & saveExpToFile)
	 */
	Burger2D::System *system_ = NULL;
	/**
	 * \brief the last planner used
	 * 
	 * Helpful in peripheral functions (publishStateRviz & saveExpToFile)
	 */
	
	planner_t *planner_ = NULL;

	/**
	 * \brief enable plugin verbose output
	 */
	bool verbose_;
	/**
	 * \brief enable pusblishing visual markers for Rviz
	 */
	bool publishMarkers_;
	/**
	 * \brief path publisher (rviz markers)
	 */
	ros::Publisher markerPub_;
	/**
	 * \brief tree pusblisher (rviz markers)
	 */
	ros::Publisher treeMarkerPub_;
	/**
	 * \brief unused
	 * Could be useful to ensure uniqueness of msgs
	 */
	unsigned int msgIndex_ = 0;
	
	/**
	 * \brief goal bias factor in RRTS
	 * 
	 * This setting is transmitted to system_ros
	 */
	double goalBias_;
	/**
	 * \brief max iteration number
	 */
	int maxIteration_;
	/**
	 * \brief max length between two states
	 * Transmitted to system_ros
	 */
	double waypointDistance_;

	/**
	 * \brief gamma parameter of the planner
	 */
	double rrtsGamma_;

	/**
	 * \brief tolerancy around given goal state
	 * To be given in costmap cells. goalBoxSize = goalSize_*CostmapResolution
	 */
	double goalSize_;

	/**
	 * \brief Robot radius (in m)
	 * Transmitted to system_ros
	 */
	double robotRadius_;

	/**
	 * \brief Discrete length of integration when computing next state on a path
	 * TRansmitted to system_ros
	 */
	double discretizationStep_;

	/**
	 * \brief Save of last asked goal
	 */
	geometry_msgs::PoseStamped lastGoal_;

	/**
	 * \brief clears a cell of the costmap
	 */
	void clearRobotCell(const unsigned int mx, const unsigned int my);

	/**
	 * \brief builds the final plan given to move_base
	 */
	bool formatPlan(std::list<double *> stateListIn, std::vector<geometry_msgs::PoseStamped> &planOut);


};
};
#endif
