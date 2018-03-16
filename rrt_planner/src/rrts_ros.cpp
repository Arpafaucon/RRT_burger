#include <pluginlib/class_list_macros.h>
#include <unistd.h>
#include "rrt_planner/rrts_ros.h"
#include "rrt_planner/system_ros.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <math.h>

#define PI 3.14159265

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
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
	if (!initialized_)
	{
		ROS_INFO("Begin Burger RRT Initialization");
		costmap_ = costmap_ros->getCostmap(); //get the costmap_ from costmap_ros_
		frame_id_ = costmap_ros->getGlobalFrameID();

		ros::NodeHandle nodeHandle("~/" + name);
		// getting node params
		nodeHandle.setParam("rrtsros_dummyparam", true);
		nodeHandle.param("goal_bias", goalBias_, GOAL_BIAS);
		nodeHandle.param("publish_markers", publishMarkers_, PUBLISH_MARKERS);
		nodeHandle.param("max_iteration", maxIteration_, MAX_ITERATION);
		nodeHandle.param("waypoint_distance", waypointDistance_, WAYPOINT_DISTANCE);
		nodeHandle.param("rrts_gamma", rrtsGamma_, RRTS_GAMMA);
		nodeHandle.param("goal_size", goalSize_, GOAL_SIZE);
		nodeHandle.param("verbose", verbose_, VERBOSE);
		nodeHandle.param("robot_radius", robotRadius_, ROBOT_RADIUS);
		nodeHandle.param("discretization_step", discretizationStep_, DISCRETIZATION_STEP);

		ROS_WARN_STREAM("got node params : " << goalBias_ << "," << publishMarkers_ << "," << maxIteration_ << "," << waypointDistance_ << "," << rrtsGamma_ << "," << goalSize_ << "," << verbose_ << "," << robotRadius_);

		if (publishMarkers_)
		{
			markerPub_ = nodeHandle.advertise<visualization_msgs::Marker>("rrts_ros_path", 1, true);
			treeMarkerPub_ = nodeHandle.advertise<visualization_msgs::Marker>("rrts_ros_tree", 1, true);
		}

		initialized_ = true;
		ROS_INFO("End  Burger RRT Initialization");
	}
	else
	{
		ROS_WARN("This planner has already been initialized... doing nothing");
	}
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
						  std::vector<geometry_msgs::PoseStamped> &plan)
{
	if (!initialized_)
	{
		ROS_ERROR("Planner not initialized. Exiting");
		return false;
	}
	double wx, wy, rwx, rwy;
	unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
	double start_x, start_y, goal_x, goal_y;
	ROS_INFO("RRTS Star called");
	if (planner_ != NULL)
	{
		delete planner_;
	}
	if (system_ != NULL)
	{
		delete system_;
	}
	Burger2D::System *burgerSystem = new Burger2D::System(costmap_);
	// transmitting node params to system_ros
	burgerSystem->robotRadiusCells_= burgerSystem->convertDistance(robotRadius_);
	burgerSystem->goalBias_ = goalBias_;
	burgerSystem->waypointDistance_ = burgerSystem->convertDistance(waypointDistance_);
	double goalSizeCells = burgerSystem->convertDistance(goalSize_);
	burgerSystem->discretizationStep_ = burgerSystem->convertDistance(discretizationStep_);

	planner_t *rrts = new planner_t();
	planner_ = rrts;
	system_ = burgerSystem;
	ROS_INFO_COND(verbose_, "BurgerRRT is making a plan");

	wx = goal.pose.position.x;
	wy = goal.pose.position.y;
	if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
	{
		ROS_WARN(
			"The robot's goal position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized? %d %d", goal_x_i, goal_y_i);
		return false;
	}

	burgerSystem->worldToMap(wx, wy, goal_x, goal_y);
	burgerSystem->mapToWorld(goal_x, goal_y, rwx, rwy);
	ROS_INFO_COND(verbose_, "RRTS Star got goal (%lf, %lf) [in world %lf %lf | rev=(%lf, %lf)]", goal_x, goal_y, wx, wy, rwx, rwy);

	Burger2D::region2 goalRegion;
	goalRegion.center[0] = goal_x;
	goalRegion.center[1] = goal_y;
	goalRegion.size[0] = goalSizeCells;
	goalRegion.size[1] = goalSizeCells;
	burgerSystem->regionGoal_ = goalRegion;

	Burger2D::region2 operatingRegion;
	int sizeX = costmap_->getSizeInCellsX();
	int sizeY = costmap_->getSizeInCellsY();
	operatingRegion.center[0] = sizeX / 2.;
	operatingRegion.center[1] = sizeY / 2.;
	operatingRegion.size[0] = sizeX;
	operatingRegion.size[1] = sizeY;
	burgerSystem->regionOperating_ = operatingRegion;
	ROS_INFO_COND(verbose_, "Read costmap dimension %d %d", sizeX, sizeY);

	rrts->setSystem(*burgerSystem);
	ROS_INFO_COND(verbose_, "RRTS Star set sytem");

	// parsing start position
	wx = start.pose.position.x;
	wy = start.pose.position.y;
	if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
	{
		ROS_WARN(
			"The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized? %d %d", start_x_i, start_y_i);
		return false;
	}
	ROS_INFO_COND(verbose_, "RRTS Star got start");
	burgerSystem->worldToMap(wx, wy, start_x, start_y);

	burgerSystem->mapToWorld(start_x, start_y, rwx, rwy);
	ROS_INFO_COND(verbose_, "RRTS Star got start (%lf, %lf) [in world %lf %lf | rev %lf %lf]", start_x, start_y, wx, wy, rwx, rwy);
	ROS_INFO_COND(verbose_, "RRTS Star 2b");
	Burger2D::State2 &rootState = rrts->getRootVertex().getState();
	// ROS_INFO_COND(verbose_,"rrts.getRootVertex.getstate donne %lf, %lf", rootState[0], rootState[1]);
	ROS_INFO_COND(verbose_, "RRTS Star got root vertex");
	rootState[0] = start_x;
	rootState[1] = start_y;

	//clear the starting cell within the costmap because we know it can't be an obstacle
	tf::Stamped<tf::Pose> start_pose;
	tf::poseStampedMsgToTF(start, start_pose);
	clearRobotCell(start_x_i, start_y_i);
	ROS_INFO_COND(verbose_, "RRTS Star cleared robot cell");

	// Initialize the planner
	if (rrts->initialize() == 0)
	{
		ROS_WARN("RRTS inititialize failed!");
	}
	// This parameter should be larger than 1.5 for asymptotic
	//   optimality. Larger values will weigh on optimization
	//   rather than exploration in the RRT* algorithm. Lower
	//   values, such as 0.1, should recover the RRT.
	rrts->setGamma(rrtsGamma_);

	ROS_INFO_COND(verbose_, "RRT Star completerly initialized");
	// spinForDebug();

	clock_t startTime = clock();

	// Run the algorithm for 10000 iterations
	for (int i = 0; i < maxIteration_; i++)
	{
		int error = rrts->iteration();
		if (error < 0)
		{
			ROS_WARN_THROTTLE(1, "rrts.iteration failed error %d", error);
		}
		// ROS_INFO_COND(verbose_,"number of vertice : %d", rrts.numVertices);
		ROS_INFO_THROTTLE(2, "burger iterating");
	}

	clock_t finishTime = clock();

	list<double *> stateList;
	if (rrts->getBestTrajectory(stateList) == 0)
	{
		ROS_WARN("rrts.getBestTrajectory failed");
	}
	ROS_INFO_COND(verbose_, "stateList length = %lu", stateList.size());

	ROS_INFO("RRT Start finished running  in %lf s", ((double)(finishTime - startTime)) / CLOCKS_PER_SEC);

	ros::Time plan_time = ros::Time::now();

	// int stateIndex = 0;

	for (list<double *>::iterator iter = stateList.begin(); iter != stateList.end(); iter++)
	{
		ROS_INFO_THROTTLE(1, "added vertex in plan");
		double *stateRef = *iter;
		double world_x, world_y;
		burgerSystem->mapToWorld(stateRef[0], stateRef[1], world_x, world_y);
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
	}

	ROS_INFO_STREAM("RRT Star finished filling plan (entries: " << plan.size() << " )");

	ROS_INFO_COND(verbose_, "Publishing to rviz...");
	if (publishMarkers_)
	{
		publishStatesRviz(stateList, &(rrts->listVertices));
	}

	ROS_INFO("Plan finished");
	return true;
}

void RRTPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
	if (!initialized_)
	{
		ROS_ERROR(
			"This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//set the associated costs in the cost map to be free
	costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

void RRTPlanner::saveExpToFile(std::ofstream &out)
{
	ROS_INFO("saving to stream\n");
	list<double *> stateList;
	// Burger2D::System burgerSystem = rrts->getSystem();
	planner_->getBestTrajectory(stateList);
	for (list<double *>::iterator iter = stateList.begin(); iter != stateList.end(); iter++)
	{
		// ROS_INFO("added vertex in plan");
		double *stateRef = *iter;
		double world_x, world_y;
		// system_->mapToWorld(stateRef[0], stateRef[1], world_x, world_y);
		world_x = stateRef[0];
		world_y = stateRef[1];
		ROS_INFO_THROTTLE(1, "state: %lf,%lf\n", world_x, world_y);
		out << "W " << world_x << " " << world_y << endl;
	}
	ROS_INFO_COND(verbose_, "filling tree : %d entries", planner_->numVertices);
	for (std::list<vertex_t *>::iterator iter = planner_->listVertices.begin(); iter != planner_->listVertices.end(); iter++)
	{
		double wcx, wcy, wpx, wpy;
		vertex_t &vertexCurr = **iter;

		vertex_t &vertexParent = vertexCurr.getParent();

		if (&vertexParent == NULL)
			continue;

		state_t &stateCurr = vertexCurr.getState();
		state_t &stateParent = vertexParent.getState();
		wcx = stateCurr[0];
		wcy = stateCurr[1];
		wpx = stateParent[0];
		wpy = stateParent[1];

		out << "T "
			<< wpx << " " << wpy
			<< " " << wcx << " " << wcy << endl;
	}
}

bool RRTPlanner::publishStatesRviz(list<double *> &stateList, list<vertex_t *> *verticesList)
{
	if (!initialized_ || system_ == NULL)
	{
		ROS_ERROR("Invalid configuration. Unable to publish");
		return false;
	}
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id_;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "state_list";
	marker.id = 0;
	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::LINE_LIST;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.01;
	marker.pose.orientation.w = 1.0;

	marker.lifetime = ros::Duration();

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.8;
	int count = 0;
	for (list<double *>::iterator iter = stateList.begin(); iter != stateList.end(); iter++)
	{

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		double *stateRef = *iter;
		double world_x, world_y;
		system_->mapToWorld(stateRef[0], stateRef[1], world_x, world_y);
		geometry_msgs::Point point, pur, pul, pbr, pbl;
		// for this RRT, squares have the size of the robot
		double sql = robotRadius_;
		point.x = world_x;
		point.y = world_y;
		pur.x = world_x + sql;
		pur.y = world_y + +sql;
		pul.x = world_x - sql;
		pul.y = world_y + +sql;
		pbr.x = world_x + sql;
		pbr.y = world_y - +sql;
		pbl.x = world_x - sql;
		pbl.y = world_y - +sql;

		std_msgs::ColorRGBA color;
		color.r = 0.6 + 0.3 * cos(PI * count / 5);
		color.a = 0.9;

		marker.points.push_back(pul);
		marker.points.push_back(pur);
		marker.points.push_back(pur);
		marker.points.push_back(pbr);
		marker.points.push_back(pbr);
		marker.points.push_back(pbl);
		marker.points.push_back(pbl);
		marker.points.push_back(pul);

		for (int i = 0; i < 8; i++)
		{
			marker.colors.push_back(color);
		}
		count++;
	}
	ROS_INFO_COND(verbose_, "attempt to publish message with %d markers %lu", count, marker.points.size());
	markerPub_.publish(marker);
	ROS_INFO_COND(verbose_, "published shape");

	if (verticesList != NULL)
	{
		visualization_msgs::Marker markerTree;
		markerTree.header.frame_id = frame_id_;
		markerTree.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		markerTree.ns = "tree_wp_list";
		markerTree.id = 0;
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		markerTree.type = visualization_msgs::Marker::LINE_LIST;

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		markerTree.action = visualization_msgs::Marker::ADD;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		markerTree.scale.x = 0.005;

		markerTree.lifetime = ros::Duration();

		// Set the color -- be sure to set alpha to something non-zero!
		markerTree.color.r = 0.0f;
		markerTree.color.g = 0.8f;
		markerTree.color.b = 0.8f;
		markerTree.color.a = 0.6;
		for (list<vertex_t *>::iterator iter = verticesList->begin(); iter != verticesList->end(); iter++)
		{
			vertex_t &vertexCurr = **iter;
			vertex_t &vertexParent = vertexCurr.getParent();

			if (&vertexParent == NULL)
				continue;

			state_t &stateCurr = vertexCurr.getState();
			state_t &stateParent = vertexParent.getState();

			double cx, cy, px, py;
			system_->mapToWorld(stateCurr[0], stateCurr[1], cx, cy);
			system_->mapToWorld(stateParent[0], stateParent[1], px, py);

			geometry_msgs::Point ppar, pcur;
			ppar.x = px;
			ppar.y = py;
			pcur.x = px;
			pcur.y = py;
			markerTree.points.push_back(ppar);
			markerTree.points.push_back(pcur);
		}
		treeMarkerPub_.publish(markerTree);
		ROS_INFO_COND(verbose_, "published tree (%lu entries)", verticesList->size());
	}
	return true;
}
};