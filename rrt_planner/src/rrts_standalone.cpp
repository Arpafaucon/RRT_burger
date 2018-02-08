/**
 * \file rrts_main 
 * \brief Launcher class
 * \author arpad
 * 
 * Be careful, there are many lost adresses when config returns (the object is FAR from being standalone)
 * So Everything should be created on a reliable portion of the stack
 */

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

experience_t createExperienceFromFile(const char *fileName, char *titleOut);
experience_t configExp(const surface_t dim, const point_t start, const surface_t goal, vector<surface_t> obstacleList);
double sfCenterX(surface_t s);
double sfCenterY(surface_t s);
void fileExperience(char *fileNameIn);

// int publishTree (lcm_t *lcm, planner_t& planner, System& system);
// int publishTraj (lcm_t *lcm, planner_t& planner, System& system);
// int publishEnvironment (lcm_t *lcm, region& regionOperating, region& regionGoal, list<region*>& obstacleList);

int main(int argc, char **argv)
{
    char defFileName[] = "exp/exp.rsc";
    char *fileName;

    cout << "=================" << endl;

    if (argc < 2)
    {
        fileName = defFileName;
        cout << "no filename" << endl;
    }
    else
    {
        fileName = argv[1];
        std::cout << "file : " << argv[1] << endl;
    }

    fileExperience(fileName);

    return 0;
}

/**
 * \brief complete file-based experience
 * 
 * Sorry, erything is much too long...
 * This is because the current system allocates everything on the stack 
 * This first version respects this design...at the cost of conciseness
 * 
 * \param fileNameIn 
 */
void fileExperience(char *fileNameIn)
{
    char expTitle[TITLE_SIZE];
    char fileNameOut[TITLE_SIZE];
    // //System representation
    string title;
    point_t start;
    surface_t dim, goal;
    vector<surface_t> obstacleList;
    cout << "filename provided : " << fileNameIn << endl;

    planner_t rrts = planner_t();
    System system = System();

    /*******************************
     * FILE PARSING 
     ******************************/
    string line;
    std::ifstream infile(fileNameIn);
    std::getline(infile, title);
    cout << "title : " << title << endl
         << "reading params" << endl;
    snprintf(expTitle, TITLE_SIZE, "%s", title.c_str());
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

    /*************************************
     * CONFIGURATION OF THE SYSTEM AND RRT
     ************************************/

    cout << "RRTstar is alive" << endl;

    // Three dimensional configuration space
    system.setNumDimensions(SPACE_DIM);

    // Define the operating region
    system.regionOperating.setNumDimensions(SPACE_DIM);
    system.regionOperating.center[0] = sfCenterX(dim);
    system.regionOperating.center[1] = sfCenterY(dim);
    system.regionOperating.size[0] = dim[W];
    system.regionOperating.size[1] = dim[H];

    // Define the goal region
    system.regionGoal.setNumDimensions(SPACE_DIM);
    system.regionGoal.center[0] = sfCenterX(goal);
    system.regionGoal.center[1] = sfCenterY(goal);
    // system.regionGoal.center[2] = 2.0;
    system.regionGoal.size[0] = goal[W];
    system.regionGoal.size[1] = goal[H];
    // system.regionGoal.size[2] = 2.0;

    // Define the obstacle region
    region *obstacle;

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

        cout << "O cx cy wx wy:" << obstacle->center[0] << " " << obstacle->center[1] << " "
             << obstacle->size[0] << " " << obstacle->size[1] << endl;

        system.obstacles.push_back(obstacle); // Add the obstacle to the list
    }

    // Add the system to the planner
    rrts.setSystem(system);

    // Set up the root vertex
    vertex_t &root = rrts.getRootVertex();
    State &rootState = root.getState();
    rootState[0] = start[X];
    rootState[1] = start[Y];
    // rootState[2] = 0.0;

    cout << "RRT star initialized" << endl;

    /**************************************
     * RUNNING THE SYSTEM
     *************************************/

    // Initialize the planner
    rrts.initialize();

    // This parameter should be larger than 1.5 for asymptotic
    //   optimality. Larger values will weigh on optimization
    //   rather than exploration in the RRT* algorithm. Lower
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma(1.5);

    clock_t startTime = clock();

    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < 2000; i++)
        rrts.iteration();

    clock_t finishTime = clock();
    cout << "Time : " << (static_cast<double>(finishTime - startTime)) / CLOCKS_PER_SEC << endl;

    /********************************************
     * WRITING HE RESULTS IN A FILE
     *******************************************/

    snprintf(fileNameOut, TITLE_SIZE, "exp/%s.sol", expTitle);
    ofstream resultFile(fileNameOut, ios::trunc);

    resultFile << expTitle << endl;
    resultFile << fileNameIn << endl;

    list<double *> stateList;

    rrts.getBestTrajectory(stateList);

    int stateIndex = 0;
    for (list<double *>::iterator iter = stateList.begin(); iter != stateList.end(); iter++)
    {

        double *stateRef = *iter;
        bool coll = system.IsInCollision(stateRef);

        //small verif to be sure
        // cout << "reaching" << system. << endl;

        resultFile << "W " << stateRef[0] << " " << stateRef[1] << " " << (coll ? "!" : "") << endl;

        delete[] stateRef;
        // stateIndex++;
    }

    for (list<vertex_t *>::iterator iter = rrts.listVertices.begin(); iter != rrts.listVertices.end(); iter++)
    {

        vertex_t &vertexCurr = **iter;

        vertex_t &vertexParent = vertexCurr.getParent();

        if (&vertexParent == NULL)
            continue;

        State &stateCurr = vertexCurr.getState();
        State &stateParent = vertexParent.getState();

        resultFile << "T "
                   << stateParent[0] << " " << stateParent[1]
                   << " " << stateCurr[0] << " " << stateCurr[1] << endl;
    }

    resultFile.close();
}

double sfCenterX(surface_t s)
{
    return s[X] + 0.5 * s[W];
}

double sfCenterY(surface_t s)
{
    return s[Y] + 0.5 * s[H];
}

// bool testStateValidity(double stateIn, list<region> obstacles)
// {
//     for (list<region>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++)
//     {
//         if()
//     }
// }
