#include <pluginlib/class_list_macros.h>
#include "rrt_planner/carrot.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace tf;

//Default Constructor
namespace rrt_planner
{
RRTPlanner::RRTPlanner()
{
	//initialize("Whohohohohoh", costmap_2d::Costmap2DROS:: ;
}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
	initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
	if (!initialized_)
	{
		ROS_INFO("Begin RRT Initialization");
		costmap_ros_ = costmap_ros;			   //initialize the costmap_ros_ attribute to the parameter.
		costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_

		// initialize other planner parameters
		ros::NodeHandle private_nh("~/" + name);
		private_nh.param("step_size", step_size_, costmap_->getResolution());
		private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
		world_model_ = new base_local_planner::CostmapModel(*costmap_);

		initialized_ = true;
		ROS_INFO("End RRT Initialization");
	}
	else
		ROS_WARN("This planner has already been initialized... doing nothing");
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
						  std::vector<geometry_msgs::PoseStamped> &plan)
{
	ROS_INFO("BurgerRRT is making a plan");
	plan.push_back(start);
	Pose startPose = myPoseStampedMsgToTF(start);
	Pose goalPose = myPoseStampedMsgToTF(goal);

	Transform difference = startPose.inverseTimes(goalPose);

	float diffX = goal.pose.position.x - start.pose.position.x;
	float diffY = goal.pose.position.y - start.pose.position.y;

	tf::Quaternion quat = difference.getRotation();

	int numberOfPoint = 5;

	for (int i = 0; i < numberOfPoint; i++)
	{
		//ROS_INFO("Whoholo");
		geometry_msgs::PoseStamped inter_goal = goal;

		inter_goal.pose.orientation = tf::createQuaternionMsgFromYaw(quat.getY());

		inter_goal.pose.position.x = start.pose.position.x + (i * (float)(1.0 / (float)numberOfPoint)) * diffX;
		//inter_goal.pose.position.y = start.pose.position.y + (i * (float)(1.0 / numberOfPoint)) * diffY;
		inter_goal.pose.position.y = start.pose.position.y;
		plan.push_back(inter_goal);
	}

	geometry_msgs::PoseStamped Xgoal = plan.back();

	for (int i = 0; i < numberOfPoint; i++)
	{
		//ROS_INFO("Whoholo");
		geometry_msgs::PoseStamped inter_goal = goal;

		inter_goal.pose.orientation = tf::createQuaternionMsgFromYaw(quat.getY());

		//inter_goal.pose.position.x = start.pose.position.x + (i * (float)(1.0 / numberOfPoint)) * diffX;
		inter_goal.pose.position.y = start.pose.position.y + (i * (float)(1.0 / (float)numberOfPoint)) * diffY;
		inter_goal.pose.position.x = Xgoal.pose.position.x;
		plan.push_back(inter_goal);
	}

	plan.push_back(goal);
	ROS_INFO("Plan finished");
	return true;
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double RRTPlanner::footprintCost(double x_i, double y_i, double theta_i)
{
	ROS_INFO("Footprint called");
	if (!initialized_)
	{
		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
		return -1.0;
	}

	std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
	//if we have no footprint... do nothing
	if (footprint.size() < 3)
		return -1.0;

	//check if the footprint is legal
	double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
	return footprint_cost;
}
};
