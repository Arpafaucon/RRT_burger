#include "ros/ros.h"
#include "std_msgs/String.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <list>
#include <sstream>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <ctime>
#include <fstream>
#include <sstream>

#include "rrts_ros.h"

#define COSTMAP_RESOLUTION
#define TITLE_SIZE 200

#define CENTRE(x, y) ((x) + (y) / 2)

typedef RRTstar::Planner<Burger2D::State2, Burger2D::Trajectory, Burger2D::System> planner_t;
typedef RRTstar::Vertex<Burger2D::State2, Burger2D::Trajectory, Burger2D::System> vertex_t;
typedef std::array<double, SPACE_DIM> point_t;
typedef std::array<double, 2 * SPACE_DIM> surface_t;
typedef std::pair<planner_t *, Burger2D::System *> experience_t;
typedef Burger2D::State2 state_t;

costmap_2d::Costmap2D configCostmap(string fileNameIn,
									double resolution,
									geometry_msgs::PoseStamped &poseStart,
									geometry_msgs::PoseStamped &poseEnd,
									string &expTitle);
int writeRes(rrts_burger::RRTPlanner planner, std::string &outputDir, const char *fileNameIn, const char *expTitle);
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "planner_tester");
	/**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n("~");
	std::string expFileName;
	std::string outputDir;
	double res;
	n.getParam("f", expFileName);
	n.getParam("odir", outputDir);
	n.getParam("res", res);
	ROS_INFO_STREAM("got fileName " << expFileName);
	ROS_INFO_STREAM("got odir " << outputDir);

	/**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	/**
   *  Create the CostMap *
   * costmap_2d::Costmap2D::Costmap2D 	( 	unsigned int  	cells_size_x,
		unsigned int  	cells_size_y,
		double  	resolution,
		double  	origin_x,
		double  	origin_y,
		unsigned char  	default_value = 0 
	) 	
   */
	//   tf::TransformListener tfl(&n);
	tf::TransformListener tfl;
	ROS_INFO("Created TransformListener");
	costmap_2d::Costmap2DROS costmap_ros(std::string("testWrapper"), tfl);
	ROS_INFO("Created CostMap2DROS");
	geometry_msgs::PoseStamped start;
	geometry_msgs::PoseStamped end;
	ROS_INFO("Created CostMap2D");
	std::string expTitle;
	costmap_2d::Costmap2D costmap = configCostmap(expFileName, res, start, end, expTitle);
	ROS_INFO("Configured CostMap2D");

	*(costmap_ros.getCostmap()) = costmap;

	rrts_burger::RRTPlanner planner;
	planner.initialize(std::string("testPlanner"), &costmap_ros);
	ROS_INFO("Initialized");

	// start.pose.position.x = 0;
	// start.pose.position.y = 0;
	// end.pose.position.x = 1;
	// end.pose.position.y = 2;

	std::vector<geometry_msgs::PoseStamped> plan;

	planner.makePlan(start, end, plan);

	writeRes(planner, outputDir, expFileName.c_str(), expTitle.c_str());

	//   while (ros::ok())
	//   {
	//     /**
	//      * This is a message object. You stuff it with data, and then publish it.
	//      */
	//     std_msgs::String msg;

	//     std::stringstream ss;
	//     ss << "hello world " << count;
	//     msg.data = ss.str();

	//     ROS_INFO("%s", msg.data.c_str());

	//     /**
	//      * The publish() function is how you send messages. The parameter
	//      * is the message object. The type of this object must agree with the type
	//      * given as a template parameter to the advertise<>() call, as was done
	//      * in the constructor above.
	//      */
	//     chatter_pub.publish(msg);

	//     ros::spinOnce();

	//     loop_rate.sleep();
	//     ++count;
	//   }

	return 0;
}

int writeRes(rrts_burger::RRTPlanner planner, std::string &outputDir, const char *fileNameIn, const char *expTitle)
{
	char fileNameOut[TITLE_SIZE];
	snprintf(fileNameOut, TITLE_SIZE, "%s/%s-ros.sol", outputDir.c_str(), expTitle);
	std::ofstream resultFile(fileNameOut, std::ios::trunc);

	resultFile << expTitle << std::endl;
	resultFile << fileNameIn << std::endl;
	planner.saveExpToFile(resultFile);
	ROS_INFO("saved Exp to file");
	return 0;
}

costmap_2d::Costmap2D configCostmap(string fileNameIn,
									double resolution,
									geometry_msgs::PoseStamped &poseStart,
									geometry_msgs::PoseStamped &poseEnd,
									string &expTitleOut)
{
	string title;
	char expTitle[TITLE_SIZE];
	point_t start;
	surface_t dim, goal;
	std::vector<surface_t> obstacleList;
	string line;
	std::ifstream infile(fileNameIn);
	if (!infile.is_open())
	{
		ROS_ERROR("Couldn't open file. Exiting");
		exit(EXIT_FAILURE);
	}
	/*******************************
     * FILE PARSING 
     ******************************/

	std::getline(infile, title);
	std::cout << "title : " << title << std::endl
			  << "reading params" << std::endl;
	snprintf(expTitle, TITLE_SIZE, "%s", title.c_str());
	expTitleOut.assign(expTitle);
	while (std::getline(infile, line))
	{
		std::istringstream iss(line);
		// cout << "line read : " << line << endl;
		char command;
		double x, y, w, h;
		bool secondGetFailed = false;
		surface_t obs;
		if (!(iss >> command >> x >> y))
		{
			std::cout << "error" << std::endl;
			continue;
		} // error
		if (!(iss >> w >> h))
		{
			secondGetFailed = true;
		} // error only if additionnal params are needed

		switch (command)
		{
		case 'D':
			if (secondGetFailed)
			{
				std::cout << "error" << std::endl;
				continue;
			}
			//got dimensions
			std::cout << "dim (x, y, w, h)=" << x << "," << y << "," << w << "," << h << std::endl;
			dim = {{x, y, w, h}};
			// dim_w = x;
			// dim_h = y;
			break;
		case 'S':
			//got start
			std::cout << "start (x, y)=" << x << "," << y << std::endl;
			start = {{x, y}};
			// start_x = x;
			// start_y = y;
			break;
		case 'G':
			//got goal
			if (secondGetFailed)
			{
				std::cout << "error" << std::endl;
				continue;
			}
			std::cout << "goal (x, y, w, h)=" << x << "," << y << "," << w << "," << h << std::endl;
			goal = {{x, y, w, h}};
			// goal_x = x;
			// goal_y = y;
			// goal_w = w;
			// goal_h = h;
			break;
		case 'O':
			//got dimensions
			if (secondGetFailed)
			{
				std::cout << "error" << std::endl;
				continue;
			}
			std::cout << "obstacle (x, y, w, h)=" << x << "," << y << "," << w << "," << h << std::endl;
			obs = {{x, y, w, h}};
			// goal_x = x;
			// goal_y = y;
			// goal_w = w;
			// goal_h = h;
			obstacleList.push_back(obs);
			break;

		default:
			std::cout << "error command unrecognized = " << command << std::endl;
			break;
			// process pair (x,y)
		}
	}

	/**retranscription */
	int ox = dim[2];
	int oy = dim[3];
	double offx = dim[0] / resolution;
	double offy = dim[1] / resolution;
	costmap_2d::Costmap2D costmap(ox, oy, resolution, offx, offy);
	ROS_INFO("created costmap w;h;RES;offx;offy : %u ; %u ; %lf ; %lf ; %lf", ox, oy, resolution, offx, offy);
	poseStart.pose.position.x = start[0] * resolution;
	poseStart.pose.position.y = start[1] * resolution;
	ROS_INFO_STREAM("Start is " << poseStart.pose.position.x << ", " << poseStart.pose.position.y);
	poseEnd.pose.position.x = CENTRE(goal[0], goal[2]) * resolution;
	poseEnd.pose.position.y = CENTRE(goal[1], goal[3]) * resolution;
	ROS_INFO_STREAM("Goal is " << poseEnd.pose.position.x << ", " << poseEnd.pose.position.y);

	for (std::vector<surface_t>::iterator iter = obstacleList.begin(); iter != obstacleList.end(); iter++)
	{
		for (int i = 0; i < (*iter)[2]; i++)
		{
			for (int j = 0; j < (*iter)[3]; j++)
			{
				double cellx = (*iter)[0] + i;
				double celly = (*iter)[1] + j;
				costmap.setCost((unsigned int)cellx, (unsigned int)celly, 255);
				// ROS_INFO("filling cell %lf %lf as an obst", cellx, celly);
			}
		}
	}

	//no obstacles for now

	infile.close();
	return costmap;
}