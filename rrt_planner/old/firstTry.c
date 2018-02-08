#include <iostream>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <list>

// #include <bot_core/bot_core.h>

// #include <lcm/lcm.h>

// #include <lcmtypes/lcmtypes.h>

#include "rrts.hpp"
#include "system_single_integrator.h"

#define TITLE_SIZE 40
#define DPRINT printf
#define SPACE_DIM 2
#define X 0
#define Y 1
#define W 2
#define H 3

using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;

typedef Planner<State, Trajectory, System> planner_t;
typedef Vertex<State, Trajectory, System> vertex_t;
typedef array<double, SPACE_DIM> point_t;
typedef array<double, 2 * SPACE_DIM> surface_t;
typedef pair<planner_t *, System *> experience_t;

/**
 * \brief loads x experience file into the RRT planner
 * 
 * The file will be scanned according to the format specified into viz/viz.py for the moment
 * 
 * \param fileName name of file to use 
 * \param titleOut set title of the read experience for further use
 * */
experience_t createExperienceFromFile(const char *fileName, char *titleOut)
{
    cout << "filename provided : " << fileName << endl;

    // //System representation
    string title;
    point_t start;
    surface_t dim, goal;
    vector<surface_t> obstacleList;
    // double dim_w, dim_h;
    // double start_x, start_y;
    // double goal_x, goal_y, goal_w, goal_h;
    // vector<array<double, 4>> obstacles;

    // file parsing
    string line;
    std::ifstream infile(fileName);
    std::getline(infile, title);
    cout << "title : " << title << endl
         << "reading params" << endl;
    snprintf(titleOut, TITLE_SIZE, "%s", title.c_str());
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        cout << "line read : " << line << endl;
        char command;
        double x, y, w, h;
        bool secondGetFailed = false;
        surface_t obs;
        if (!(iss >> command >> x >> y))
        {
            cout << "error" << endl;
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
                cout << "error" << endl;
                continue;
            }
            //got dimensions
            cout << "dim (x, y, w, h)=" << x << "," << y << "," << w << "," << h << endl;
            dim = {{x, y, w, h}};
            // dim_w = x;
            // dim_h = y;
            break;
        case 'S':
            //got start
            cout << "start (x, y)=" << x << "," << y << endl;
            start = {{x, y}};
            // start_x = x;
            // start_y = y;
            break;
        case 'G':
            //got goal
            if (secondGetFailed)
            {
                cout << "error" << endl;
                continue;
            }
            cout << "goal (x, y, w, h)=" << x << "," << y << "," << w << "," << h << endl;
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
                cout << "error" << endl;
                continue;
            }
            cout << "obstacle (x, y, w, h)=" << x << "," << y << "," << w << "," << h << endl;
            obs = {{x, y, w, h}};
            // goal_x = x;
            // goal_y = y;
            // goal_w = w;
            // goal_h = h;
            obstacleList.push_back(obs);
            break;

        default:
            cout << "error command unrecognized = " << command << endl;
            break;
            // process pair (x,y)
        }
    }

    infile.close();

    return configExp(dim, start, goal, obstacleList);
}

experience_t configExp(const surface_t dim, const point_t start, const surface_t goal, vector<surface_t> obstacleList)
{

    planner_t *rrts = new planner_t();
    System *system = new System();

    cout << "RRTstar is alive" << endl;

    // Get lcm
    // lcm_t *lcm = bot_lcm_get_global(NULL);
    // Create the dynamical system

    // Three dimensional configuration space
    system->setNumDimensions(SPACE_DIM);

    // Define the operating region
    system->regionOperating.setNumDimensions(SPACE_DIM);
    system->regionOperating.center[0] = sfCenterX(dim);
    system->regionOperating.center[1] = sfCenterY(dim);
    // system.regionOperating.center[2] = 0.0;
    system->regionOperating.size[0] = dim[W];
    system->regionOperating.size[1] = dim[H];
    // system.regionOperating.size[2] = 20.0;

    // Define the goal region
    system->regionGoal.setNumDimensions(SPACE_DIM);
    system->regionGoal.center[0] = sfCenterX(goal);
    system->regionGoal.center[1] = sfCenterY(goal);
    // system.regionGoal.center[2] = 2.0;
    system->regionGoal.size[0] = goal[W];
    system->regionGoal.size[1] = goal[H];
    // system.regionGoal.size[2] = 2.0;

    // Define the obstacle region

    region *obstacle;

    // int a = obstacleList.size();

    for (std::vector<surface_t>::iterator obsIt = obstacleList.begin(); obsIt != obstacleList.end(); ++obsIt)
    {
        obstacle = new region;
        obstacle->setNumDimensions(SPACE_DIM);
        obstacle->center[0] = sfCenterX(*obsIt);
        obstacle->center[1] = sfCenterY(*obsIt);
        // obstacle->center[2] = 6;
        obstacle->size[0] = (*obsIt)[W];
        obstacle->size[1] = (*obsIt)[H];
        // obstacle->size[2] = 8;

        system->obstacles.push_back(obstacle); // Add the obstacle to the list
    }

    // Add the system to the planner
    rrts->setSystem(*system);

    // Set up the root vertex
    vertex_t &root = rrts->getRootVertex();
    State &rootState = root.getState();
    rootState[0] = start[X];
    rootState[1] = start[Y];
    // rootState[2] = 0.0;

    return make_pair(rrts, system);
}