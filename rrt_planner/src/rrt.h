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

using std::string;

#ifndef RRT_PLANNER_CPP
#define RRT_PLANNER_CPP

namespace rrt_planner
{

class RRTPlanner : public nav_core::BaseGlobalPlanner
{
  public:
	RRTPlanner();
	RRTPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

	/** overridden classes from interface nav_core::BaseGlobalPlanner **/
	void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped &start,
				  const geometry_msgs::PoseStamped &goal,
				  std::vector<geometry_msgs::PoseStamped> &plan);

  private:
	costmap_2d::Costmap2DROS *costmap_ros_;
	double step_size_, min_dist_from_robot_;
	costmap_2d::Costmap2D *costmap_;
	base_local_planner::WorldModel *world_model_; ///< @brief The world model that the controller will use

	/**
       	* @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       	* @param x_i The x position of the robot 
       	* @param y_i The y position of the robot 
       	* @param theta_i The orientation of the robot
       	* @return 
       	*/
	double footprintCost(double x_i, double y_i, double theta_i);

	bool initialized_;

	inline tf::Pose myPoseStampedMsgToTF(const geometry_msgs::PoseStamped &msg)
	{
		return tf::Transform(tf::Quaternion(msg.pose.orientation.x,
											msg.pose.orientation.y,
											msg.pose.orientation.z,
											msg.pose.orientation.w),
							 tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
	}
};
};
#endif
