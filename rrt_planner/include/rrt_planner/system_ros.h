/*! 
 * \file system_single_integrator.h 
 */

#ifndef RRTS_SYSTEM_ROS
#define RRTS_SYSTEM_ROS

#include <list>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#define DISCRETIZATION_STEP 0.01

namespace Burger2D
{

/*!
     * \brief region class
     *
     * More elaborate description
     */
class region2
{

  static const int numDimensions = 2;

public:
  /*!
         * \brief Cartesian coordinates of the center of the region
         *
         * More elaborate description
         */
  double center[numDimensions] = {0};

  /*!
         * \brief Size of the region in cartesian coordinates
         *
         * More elaborate description
         */
  double size[numDimensions] = {0};

  /*!
         * \brief region constructor
         *
         * More elaborate description
         */
  region2();

  region2(double xleft, double xright, double ybottom, double yup);

  /*!
         * \brief region destructor
         *
         * More elaborate description
         */
  ~region2();

  /*!
         * \brief Sets the dimensionality of the region
         *
         * More elaborate description
         *
         * \param numDimensionsIn New number of dimensions.
         *
         */
  //   int setNumDimensions(int numDimensionsIn);
};

/*!
     * \brief State2 Class.
     *
     * A more elaborate description of the State2 class
     */
class State2
{
private:
  static const int numDimensions = 2;
  double x[numDimensions] = {0};

public:
  /*!
         * \brief State constructor
         *
         * More elaborate description
         */
  State2();

  /*!
         * \brief State desctructor
         *
         * More elaborate description
         */
  ~State2();

  /*!
         * \brief State copy constructor
         *
         * More elaborate description
         */
  State2(const State2 &stateIn);

  /*!
         * \brief State assignment operator
         *
         * More elaborate description
         */
  State2 &operator=(const State2 &stateIn);

  /*!
         * \brief State bracket operator
         *
         * More elaborate description
         */
  double &operator[](const int i) { return x[i]; }

  friend class System;
  friend class Trajectory;
};

/*!
     * \brief Trajectory Class.
     *
     * A more elaborate description of the State class
     */
class Trajectory
{

  State2 *endState;
  double totalVariation;

public:
  /*!
         * \brief Trajectory constructor
         *
         * More elaborate description
         */
  Trajectory();

  /*!
         * \brief Trajectory destructor
         *
         * More elaborate description
         */
  ~Trajectory();

  /*!
         * \brief Trajectory copy constructor
         *
         * More elaborate description
         *
         * \param trajectoryIn The trajectory to be copied.
         *
         */
  Trajectory(const Trajectory &trajectoryIn);

  /*!
         * \brief Trajectory assignment constructor
         *
         * More elaborate description
         *
         * \param trajectoryIn the trajectory to be copied.
         *
         */
  Trajectory &operator=(const Trajectory &trajectoryIn);

  /*!
         * \brief Returns a reference to the end state of this trajectory.
         *
         * More elaborate description
         */
  State2 &getEndState() { return *endState; }

  /*!
         * \brief Returns a reference to the end state of this trajectory (constant).
         *
         * More elaborate description
         */
  State2 &getEndState() const { return *endState; }

  /*!
         * \brief Returns the cost of this trajectory.
         *
         * More elaborate description
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

  static const int numDimensions_ = 2;
  // costmap2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  State2 rootState_;
  bool initalized_;

  /**
   * used in mapto world and world to map: 
   * if 0.0 -> world cooridnates are the bottom left corner
   * if 0.5 -> center of the cell
   */
  double convert_offset_ = 0.5;

public:
  bool IsInCollision(double *stateIn);

  /*!
         * \brief The goal region
         *
         * More elaborate description
         */
  region2 regionGoal_;

  /*!
         * \brief The operating region
         *
         * More elaborate description
         */
  region2 regionOperating_;

  /*!
         * \brief The list of all obstacles
         *
         * More elaborate description
         */
  // std::list<region2 *> obstacles;

  /*!
         * \brief System constructor
         *
         * More elaborate description
         */
  System(costmap_2d::Costmap2D *costmap);

  /*!
         * \brief System destructor
         *
         * More elaborate description
         */
  ~System();

  //   int setNumDimensions(int numDimensionsIn);

  /*!
         * \brief Returns the dimensionality of the Euclidean space.
         *
         * A more elaborate description.
         */
  int getNumDimensions() { return numDimensions_; }

  void mapToWorld(double mx, double my, double &wx, double &wy);
  bool worldToMap(double wx, double wy, double &mx, double &my);

  /*!
         * \brief Returns a reference to the root state.
         *t
         * A more elaborate description.
         */
  State2 &getRootState() { return rootState_; }

  /*!
         * \brief Returns the statekey for the given state.
         *
         * A more elaborate description.
         *
         * \param stateIn the given state
         * \param stateKey the key to the state. An array of dimension getNumDimensions()
         *
         */
  int getStateKey(State2 &stateIn, double *stateKey);

  /*!
         * \brief Returns true if the given state reaches the target.
         *
         * A more elaborate description.
         */
  bool isReachingTarget(State2 &stateIn);

  /*!
         * \brief Returns a sample state.
         *
         * A more elaborate description.
         *
         * \param randomStateOut randomState
         *
         */
  int sampleState(State2 &randomStateOut);

  /*!
         * \brief Returns a the cost of the trajectory that connects stateFromIn and
         *        stateTowardsIn. The trajectory is also returned in trajectoryOut.
         *
         * A more elaborate description.
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
         * A more elaborate description.
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
         * A more elaborate description.
         *
         * \param stateIn Starting state
         *
         */
  double evaluateCostToGo(State2 &stateIn);

  /*!
         * \brief Returns the trajectory as a list of double arrays, each with dimension getNumDimensions.
         *
         * A more elaborate description.
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
