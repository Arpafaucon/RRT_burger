#include <pluginlib/class_list_macros.h>
#include <unistd.h>
#include "rrt_planner/rrts_ros.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrts_burger::RRTPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace tf;

//Default Constructor
namespace rrts_burger
{

void spinForDebug()
{
	volatile int i = 0;
    char hostname[256];
    gethostname(hostname, sizeof(hostname));
    printf("PID %d on %s ready for attach\n", getpid(), hostname);
    fflush(stdout);
    while (0 == i)
        sleep(5);
}

RRTPlanner::RRTPlanner()
{
	//initialize("Whohohohohoh", costmap_2d::Costmap2DROS:: ;
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
	if (!initialized_)
	{
		ROS_INFO("Begin RRT Initialization");
		costmap_ = costmap_ros->getCostmap(); //get the costmap_ from costmap_ros_
		frame_id_ = costmap_ros->getGlobalFrameID();

		initialized_ = true;
		ROS_INFO("End RRT Initialization");
	}
	else
		ROS_WARN("This planner has already been initialized... doing nothing");
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
						  std::vector<geometry_msgs::PoseStamped> &plan)
{
	static const int goalSize = 2; // in cells
	double wx, wy;
	volatile int i = 0;
	unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
	double start_x, start_y, goal_x, goal_y;
	ROS_INFO("RRTS Star called");
	Burger2D::System burgerSystem(costmap_);
	planner_t rrts = planner_t();
	ROS_INFO("BurgerRRT is making a plan");


    // char hostname[256];
    // gethostname(hostname, sizeof(hostname));
    // ROS_INFO("PID %d on %s ready for attach\n", getpid(), hostname);
    // fflush(stdout);
    // while (0 == i)
    //     sleep(5);

	wx = goal.pose.position.x;
	wy = goal.pose.position.y;
	if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
	{
		ROS_WARN(
			"The robot's goal position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
		return false;
	}	
	ROS_INFO("RRTS Star got goal");

	burgerSystem.worldToMap(wx, wy, goal_x, goal_y);
	Burger2D::region2 goalRegion;
	goalRegion.center[0] = goal_x;
	goalRegion.center[1] = goal_y;
	goalRegion.size[0] = goalSize;
	goalRegion.size[1] = goalSize;
	burgerSystem.regionGoal_ = goalRegion;
	ROS_INFO("RRTS Star initialized goal");

	rrts.setSystem(burgerSystem);
	ROS_INFO("RRTS Star set sytem");

	// parsing start position
	wx = start.pose.position.x;
	wy = start.pose.position.y;
	if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
	{
		ROS_WARN(
			"The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
		return false;
	}	
	ROS_INFO("RRTS Star got start");
	burgerSystem.worldToMap(wx, wy, start_x, start_y);
	ROS_INFO("RRTS Star 2b");
	Burger2D::State2 &rootState = rrts.getRootVertex().getState();
	ROS_INFO("RRTS Star got root vertex");
	rootState[0] = start_x;
	rootState[1] = start_y;

	//clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
	clearRobotCell(start_x_i, start_y_i);
	ROS_INFO("RRTS Star cleared robot cell");

	 // Initialize the planner
    rrts.initialize();
    // This parameter should be larger than 1.5 for asymptotic
    //   optimality. Larger values will weigh on optimization
    //   rather than exploration in the RRT* algorithm. Lower
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma(1.5);

	ROS_INFO("RRT Star completerly initialized");

	clock_t startTime = clock();

    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < 2000; i++)
        rrts.iteration();

    clock_t finishTime = clock();

	list<double *> stateList;
    rrts.getBestTrajectory(stateList);

	ROS_INFO("RRT Start finished running");

	ros::Time plan_time = ros::Time::now();

	int stateIndex = 0;
    for (list<double *>::iterator iter = stateList.begin(); iter != stateList.end(); iter++)
    {
        double *stateRef = *iter;
		double world_x, world_y;
		burgerSystem.mapToWorld(stateRef[0], stateRef[1], world_x, world_y);
		geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
		plan.push_back(pose);
        // bool coll = system.IsInCollision(stateRef);

        //small verif to be sure
        // cout << "reaching" << system. << endl;

        // resultFile << "W " << stateRef[0] << " " << stateRef[1] << " " << (coll ? "!" : "") << endl;

        // delete[] stateRef;
        // stateIndex++;
    }

	ROS_INFO("RRT Star finished filling plan (%lu entries)", plan.size());


    // cout << "Time : " << (static_cast<double>(finishTime - startTime)) / CLOCKS_PER_SEC << endl;


	// plan.push_back(start);
	// Pose startPose = myPoseStampedMsgToTF(start);
	// Pose goalPose = myPoseStampedMsgToTF(goal);

	// Transform difference = startPose.inverseTimes(goalPose);

	// float diffX = goal.pose.position.x - start.pose.position.x;
	// float diffY = goal.pose.position.y - start.pose.position.y;

	// tf::Quaternion quat = difference.getRotation();

	// int numberOfPoint = 5;

	// for (int i = 0; i < numberOfPoint; i++)
	// {
	// 	//ROS_INFO("Whoholo");
	// 	geometry_msgs::PoseStamped inter_goal = goal;

	// 	inter_goal.pose.orientation = tf::createQuaternionMsgFromYaw(quat.getY());

	// 	inter_goal.pose.position.x = start.pose.position.x + (i * (float)(1.0 / (float)numberOfPoint)) * diffX;
	// 	//inter_goal.pose.position.y = start.pose.position.y + (i * (float)(1.0 / numberOfPoint)) * diffY;
	// 	inter_goal.pose.position.y = start.pose.position.y;
	// 	plan.push_back(inter_goal);
	// }

	// geometry_msgs::PoseStamped Xgoal = plan.back();

	// for (int i = 0; i < numberOfPoint; i++)
	// {
	// 	//ROS_INFO("Whoholo");
	// 	geometry_msgs::PoseStamped inter_goal = goal;

	// 	inter_goal.pose.orientation = tf::createQuaternionMsgFromYaw(quat.getY());

	// 	//inter_goal.pose.position.x = start.pose.position.x + (i * (float)(1.0 / numberOfPoint)) * diffX;
	// 	inter_goal.pose.position.y = start.pose.position.y + (i * (float)(1.0 / (float)numberOfPoint)) * diffY;
	// 	inter_goal.pose.position.x = Xgoal.pose.position.x;
	// 	plan.push_back(inter_goal);
	// }

	// plan.push_back(goal);
	ROS_INFO("Plan finished");
	return true;
}

void RRTPlanner::clearRobotCell(unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

};