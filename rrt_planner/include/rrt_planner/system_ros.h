/*! 
 * \file system_single_integrator.h 
 */

#ifndef RRTS_SYSTEM_ROS
#define RRTS_SYSTEM_ROS

#include <list>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#define DISCRETIZATION_STEP 0.01
#define ROBOT_RADIUS 0.05
namespace Burger2D
{

/**
 * \brief Region class
 *
 * A region is a set of states. 
 * In 2D, it is a surface, and is implemented as a couple of lists (center, size)
 */
class region2
{
  /**
  * The dimension space is fixed
  */
  static const int numDimensions = 2;

public:
  /**
   * \brief Cartesian coordinates of the center of the region
   */
  double center[numDimensions] = {0};

  /**
   * \brief Size of the region in cartesian coordinates
   */
  double size[numDimensions] = {0};

  /*!
  * \brief region constructor
  */
  region2();

  /**
   * \brief box constructor
   */ 
  region2(double xleft, double xright, double ybottom, double yup);

  /*!
  * \brief region destructor
  */
  ~region2();
};

/*!
* \brief State2 Class.
*
* Defines a possible state of the robot
*/
class State2
{
private:
  /**
   * \brief fixed to 2D
   */
  static const int numDimensions = 2;
  /**
   * \brief Cartesian coordinates
   */
  double x[numDimensions] = {0};

public:
  /*!
         * \brief State constructor
         */
  State2();

  /*!
         * \brief State desctructor
         */
  ~State2();

  /*!
         * \brief State copy constructor
         */
  State2(const State2 &stateIn);

  /*!
         * \brief State assignment operator
         */
  State2 &operator=(const State2 &stateIn);

  /*!
         * \brief State bracket operator
         */
  double &operator[](const int i) { return x[i]; }

  friend class System;
  friend class Trajectory;
};

/*!
     * \brief Trajectory Class.
     *
     * A Trajectory is a path between two states.
     * The path is a right line.
     */
class Trajectory
{

  State2 *endState;
  double totalVariation;

public:
  /*!
         * \brief Trajectory constructor
         */
  Trajectory();

  /*!
         * \brief Trajectory destructor
         */
  ~Trajectory();

  /*!
         * \brief Trajectory copy constructor
         *
         * \param trajectoryIn The trajectory to be copied.
         *
         */
  Trajectory(const Trajectory &trajectoryIn);

  /*!
         * \brief Trajectory assignment constructor
         *
         * \param trajectoryIn the trajectory to be copied.
         *
         */
  Trajectory &operator=(const Trajectory &trajectoryIn);

  /*!
         * \brief Returns a reference to the end state of this trajectory.
         */
  State2 &getEndState() { return *endState; }

  /*!
         * \brief Returns a reference to the end state of this trajectory (constant).
         */
  State2 &getEndState() const { return *endState; }

  /*!
         * \brief Returns the cost of this trajectory.
         */
  double evaluateCost();

  friend class System;
};

/*!
     * \brief System Class.
     *
     * This system class uses a ROS::Costmap2D to get all the needed informations
     * 
     * A more elaborate description of the State class
     */
class System
{
private:
/**
 * \brief fixed to 2D
 */
  static const int numDimensions_ = 2;
  /**
   * \brief the internal costmap
   */
  costmap_2d::Costmap2D *costmap_;
  /**
   * \brief The start point
   */
  State2 rootState_;
  /**
   * \brief status of the system
   */
  bool initalized_;

  /**
   * Used in 'mapToworld' & 'worldToMap': 
   * if 0.0 -> world coordinates are the bottom left corner
   * if 0.5 -> center of the cell
   */
  double convert_offset_ = 0.5;

public:
  /**
  * \brief collision check
  * 
  * Checks if the given state is in conflict with an obstacle of the costmap
  */
  bool IsInCollision(double *stateIn);
  /**
   * \brief collision check
   * 
   * Checks if the given region intersects an obstacle $
   */
  bool IsInCollision(region2 region);

  /*!
         * \brief The goal region
         *
         */
  region2 regionGoal_;

  /*!
         * \brief The operating region
         *
         */
  region2 regionOperating_;


  /*!
         * \brief System constructor
         *
         */
  System(costmap_2d::Costmap2D *costmap);

  /*!
         * \brief System destructor
         *
         */
  ~System();

  //   int setNumDimensions(int numDimensionsIn);

  /*!
         * \brief Returns the dimensionality of the Euclidean space.
         * 
         * Always 2 here
         *
         */
  int getNumDimensions() { return numDimensions_; }

  /**
   * \brief Coordinates converter
   * 
   * From map (=grid) to world coordinates
   * 
   * \param mx map coordinate x
   * \param my map coordinate y
   * \param wx world coordinate x
   * \param wy world coordinate y
   */
  void mapToWorld(double mx, double my, double &wx, double &wy);
  /**
   * \brief Coordinates converter
   * 
   * From world to map(=grid) coordinates
   * 
   * \param wx world coordinate x
   * \param wy world coordinate y
   * \param mx map coordinate x
   * \param my map coordinate y
   */
  bool worldToMap(double wx, double wy, double &mx, double &my);

  /*!
         * \brief Returns a reference to the root state.
         *
         */
  State2 &getRootState() { return rootState_; }

  /*!
         * \brief Returns the statekey for the given state.
         *
         *
         * \param stateIn the given state
         * \param stateKey the key to the state. An array of dimension getNumDimensions()
         *
         */
  int getStateKey(State2 &stateIn, double *stateKey);

  /*!
         * \brief Returns true if the given state reaches the target.
         *
         */
  bool isReachingTarget(State2 &stateIn);

  /*!
         * \brief Returns a sample state.
         *
         * \param randomStateOut randomState
         *
         */
  int sampleState(State2 &randomStateOut);

  /*!
         * \brief Returns a the cost of the trajectory that connects stateFromIn and
         *        stateTowardsIn. The trajectory is also returned in trajectoryOut.
         *
         * 
         * \param stateFromIn Initial state
         * \param stateTowardsIn Final state
         * \param trajectoryOut Trajectory that starts the from the initial state and 
         *                      reaches near the final state.
         * \param exactConnectionOut Set to true if the initial and the final states
         *                           can be connected exactly.
         *
         */
  int extendTo(State2 &stateFromIn, State2 &stateTowardsIn,
               Trajectory &trajectoryOut, bool &exactConnectionOut);

  /*!
         * \brief Returns the cost of the trajectory that connects stateFromIn and StateTowardsIn.
         *
         *
         * \param stateFromIn Initial state
         * \param stateTowardsIn Final state
         * \param exactConnectionOut Set to true if the initial and the final states
         *                           can be connected exactly.
         *
         */
  double evaluateExtensionCost(State2 &stateFromIn, State2 &stateTowardsIn, bool &exactConnectionOut);

  /*!
         * \brief Returns a lower bound on the cost to go starting from stateIn
         *
         *
         * \param stateIn Starting state
         *
         */
  double evaluateCostToGo(State2 &stateIn);

  /*!
         * \brief Returns the trajectory as a list of double arrays, each with dimension getNumDimensions.
         *
         *
         * \param stateFromIn Initial state
         * \param stateToIn Final state
         * \param trajectoryOut The list of double arrays that represent the trajectory
         *
         */
  int getTrajectory(State2 &stateFromIn, State2 &stateToIn, std::list<double *> &trajectoryOut);
};
}

#endif
