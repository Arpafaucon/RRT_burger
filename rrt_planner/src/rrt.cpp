#include <pluginlib/class_list_macros.h>
#include "rrt.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

 	using namespace std;

 	//Default Constructor
 	namespace rrt_planner 
 	{
 		RRTPlanner::RRTPlanner ()
 		{
 		}


 		RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
 		{
   			initialize(name, costmap_ros);
 		}


 		void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

 		}


 		bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
 										                              std::vector<geometry_msgs::PoseStamped>& plan )
		{

			plan.push_back(start);
	   		for (int i=0; i<20; i++)
	   		{
		 		geometry_msgs::PoseStamped new_goal = goal;
		 		tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

			  	new_goal.pose.position.x = -2.5+(0.05*i);
			  	new_goal.pose.position.y = -3.5+(0.05*i);

			  	new_goal.pose.orientation.x = goal_quat.x();
			  	new_goal.pose.orientation.y = goal_quat.y();
			  	new_goal.pose.orientation.z = goal_quat.z();
			  	new_goal.pose.orientation.w = goal_quat.w();

	   			plan.push_back(new_goal);
	   		}
	   		plan.push_back(goal);
	  		return true;
	 	}
	 	
	 	  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  		double RRTPlanner::footprintCost(double x_i, double y_i, double theta_i)
  		{
    		if(!initialized_)
    		{
			  	ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			  	return -1.0;
			}

			std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
			//if we have no footprint... do nothing
			if(footprint.size() < 3)
			  	return -1.0;

			//check if the footprint is legal
			double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
			return footprint_cost;
		}
		
 	};
