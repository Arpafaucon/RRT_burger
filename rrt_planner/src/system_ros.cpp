#include "system_ros.h"
#include <cmath>
#include <cstdlib>
#include <costmap_2d/costmap_2d.h>

#include <iostream>

using namespace std;
using namespace Burger2D;

region2::region2()
{
}

region2::region2(double xleft, double xright, double ybottom, double yup)
{
	center[0] = 0.5 * (xleft + xright);
	center[1] = 0.5 * (ybottom + yup);
	size[0] = xright - xleft;
	size[1] = yup - ybottom;
}

region2::~region2()
{

	// if (center)
	//     delete [] center;
	// if (size)
	//     delete [] size;
}

// int region1::setNumDimensions (int numDimensionsIn) {

//     numDimensions = numDimensionsIn;

//     if (center)
//         delete [] center;
//     center = new double[numDimensions];

//     if (size)
//         delete [] size;
//     size = new double[numDimensions];

//     return 1;

// }

State2::State2()
{
}

State2::~State2()
{
	//nothing to do since tabs are on stack
}

State2::State2(const State2 &stateIn)
{
	// numDimensions = stateIn.numDimensions;
	for (int i = 0; i < numDimensions; i++)
		x[i] = stateIn.x[i];
}

State2 &State2::operator=(const State2 &stateIn)
{

	if (this == &stateIn)
		return *this;

	// if (numDimensions != stateIn.numDimensions)
	// {
	// 	if (x)
	// 		delete[] x;
	// 	numDimensions = stateIn.numDimensions;
	// 	if (numDimensions > 0)
	// 		x = new double[numDimensions];
	// }

	for (int i = 0; i < numDimensions; i++)
		x[i] = stateIn.x[i];

	return *this;
}

// int State::setNumDimensions(int numDimensionsIn)
// {

// 	if (x)
// 		delete[] x;

// 	if (numDimensions < 0)
// 		return 0;

// 	numDimensions = numDimensionsIn;

// 	if (numDimensions > 0)
// 		x = new double[numDimensions];

// 	return 1;
// }

Trajectory::Trajectory()
{

	endState = NULL;
}

Trajectory::~Trajectory()
{

	if (endState)
		delete endState;
}

Trajectory::Trajectory(const Trajectory &trajectoryIn)
{

	endState = new State2(trajectoryIn.getEndState());
}

Trajectory &Trajectory::operator=(const Trajectory &trajectoryIn)
{

	if (this == &trajectoryIn)
		return *this;

	if (endState)
		delete endState;

	endState = new State2(trajectoryIn.getEndState());

	totalVariation = trajectoryIn.totalVariation;

	return *this;
}

double Trajectory::evaluateCost()
{

	return totalVariation;
}

System::System(costmap_2d::Costmap2D *costmap)
{
	costmap_ = costmap;
	// numDimensions = 0;
}

System::~System()
{
	// cout << "System destroyed";
}

// int System::setNumDimensions(int numDimensionsIn)
// {

// 	if (numDimensions < 0)
// 		return 0;

// 	numDimensions = numDimensionsIn;

// 	rootState.setNumDimensions(numDimensions);

// 	return 1;
// }

double System::distance(State2 &s1, State2 &s2)
{
	double *dists = new double[numDimensions_];
	for (int i = 0; i < numDimensions_; i++)
		dists[i] = s1.x[i] - s2.x[i];

	double distTotal = 0.0;
	for (int i = 0; i < numDimensions_; i++)
		distTotal += dists[i] * dists[i];
	distTotal = sqrt(distTotal);
	return distTotal;
}

double System::distance(double *s1, double *s2)
{
	double *dists = new double[numDimensions_];
	for (int i = 0; i < numDimensions_; i++)
		dists[i] = s1[i] - s2[i];

	double distTotal = 0.0;
	for (int i = 0; i < numDimensions_; i++)
		distTotal += dists[i] * dists[i];
	distTotal = sqrt(distTotal);
	return distTotal;
}

int System::getStateKey(State2 &stateIn, double *stateKey)
{

	for (int i = 0; i < numDimensions_; i++)
		stateKey[i] = stateIn.x[i] / regionOperating_.size[i];
	return 1;
}

bool System::isReachingTarget(State2 &stateIn)
{

	for (int i = 0; i < numDimensions_; i++)
	{
		ROS_INFO_THROTTLE(2, "regienGoal_.center[i] = %lf and statein.x[i] = %lf", regionGoal_.center[i], stateIn.x[i]);
		// punctual version
		// if (fabs(stateIn.x[i] - regionGoal_.center[i]) > regionGoal_.size[i] / 2.0)
		// 	return false;
		// solid version
		if (fabs(stateIn.x[i] - regionGoal_.center[i]) > (regionGoal_.size[i] / 2.0) - robotRadiusCells_)
			return false;

	}

	return true;
}

bool System::IsInCollision(double *stateIn)
{
	unsigned int cx = (unsigned int)stateIn[0];
	unsigned int cy = (unsigned int)stateIn[1];
	int cost = costmap_->getCost(cx, cy);
	if (cost >= costmap_2d::LETHAL_OBSTACLE)
	{
		return true;
	}

	region2 footprint;
	footprint.center[0] = stateIn[0];
	footprint.center[1] = stateIn[1];
	footprint.size[0] = robotRadiusCells_;
	footprint.size[1] = robotRadiusCells_;
	return IsInCollision(footprint);
	// for (list<region2 *>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++)
	// {

	// 	region2 *obstacleCurr = *iter;
	// 	bool collisionFound = true;

	// 	for (int i = 0; i < numDimensions; i++)
	// 		if (fabs(obstacleCurr->center[i] - stateIn[i]) > obstacleCurr->size[i] / 2.0)
	// 		{
	// 			collisionFound = false;
	// 			break;
	// 		}

	// 	if (collisionFound)
	// 	{
	// 		return true;
	// 	}
	// }

	return false;
}

bool System::IsInCollision(region2 region)
{
	for (unsigned int cx = region.center[0] - region.size[0]/2.; cx <= region.center[0] + region.size[0]/2.; cx++)
	{
		for (unsigned int cy = region.center[1] - region.size[1]/2.; cy <= region.center[1]/2. + region.size[1]; cy++)
		{
			int cost = costmap_->getCost(cx, cy);
			if (cost >= costmap_2d::LETHAL_OBSTACLE)
			{
				return true;
			}
		}
	}
	return false;
}

int System::sampleState(State2 &randomStateOut)
{

	// randomStateOut.setNumDimensions(numDimensions);
	// ROS_INFO_THROTTLE(2, "regionOperating_ : size = %lf, %lf ; center %lf,%lf", regionOperating_.size[0], regionOperating_.size[1], regionOperating_.center[0], regionOperating_.center[0]);
	bool goalBiased = (rand() < RAND_MAX * goalBias_);
	if (goalBiased)
	{
		for (int i = 0; i < numDimensions_; i++)
		{
			randomStateOut.x[i] = regionGoal_.center[i];
		}
	}
	else
	{
		for (int i = 0; i < numDimensions_; i++)
		{

			randomStateOut.x[i] = (double)rand() / (RAND_MAX + 1.0) * regionOperating_.size[i] - regionOperating_.size[i] / 2.0 + regionOperating_.center[i];
		}
	}

	if (IsInCollision(randomStateOut.x))
		return 0;

	return 1;
}

double System::setRobotRadius(double radius)
{
	return robotRadiusCells_ = radius / (costmap_->getResolution());
}

int System::extendTo(State2 &stateFromIn, State2 &stateTowardsIn, Trajectory &trajectoryOut, bool &exactConnectionOut)
{
	double *dists = new double[numDimensions_];
	for (int i = 0; i < numDimensions_; i++)
		dists[i] = stateTowardsIn.x[i] - stateFromIn.x[i];

	double distTotal = 0.0;
	for (int i = 0; i < numDimensions_; i++)
		distTotal += dists[i] * dists[i];
	distTotal = sqrt(distTotal);

	// static const int DIST_MAX_BT_POINTS = 5.0;

	State2 newTowardState = stateTowardsIn;
	if (distTotal >= waypointDistance_)
	{
		for (int i = 0; i < numDimensions_; i++)
			newTowardState.x[i] = dists[i] * waypointDistance_ / distTotal + stateFromIn.x[i];

		distTotal = waypointDistance_;
	}

	double incrementTotal = distTotal / discretizationStep_;

	// normalize the distance according to the disretization step
	for (int i = 0; i < numDimensions_; i++)
		dists[i] /= incrementTotal;

	int numSegments = (int)floor(incrementTotal);

	double stateCurr[numDimensions_];
	for (int i = 0; i < numDimensions_; i++)
		stateCurr[i] = stateFromIn.x[i];

	for (int i = 0; i < numSegments; i++)
	{

		if (IsInCollision(stateCurr))
			return 0;

		for (int i = 0; i < numDimensions_; i++)
			stateCurr[i] += dists[i];
	}

	if (IsInCollision(newTowardState.x))
		return 0;

	ROS_DEBUG_THROTTLE(30, "extend from state %lf,%lf to %lf,%lf", stateFromIn.x[0], stateFromIn.x[1], stateTowardsIn.x[0], stateTowardsIn.x[1]);
	trajectoryOut.endState = new State2(newTowardState);
	trajectoryOut.totalVariation = distTotal;

	delete[] dists;
	// delete[] stateCurr;

	exactConnectionOut = true;

	return 1;
}

double System::evaluateExtensionCost(State2 &stateFromIn, State2 &stateTowardsIn, bool &exactConnectionOut)
{

	exactConnectionOut = true;

	double distTotal = 0.0;
	for (int i = 0; i < numDimensions_; i++)
	{
		double distCurr = stateTowardsIn.x[i] - stateFromIn.x[i];
		distTotal += distCurr * distCurr;
	}

	return sqrt(distTotal);
}

int System::getTrajectory(State2 &stateFromIn, State2 &stateToIn, list<double *> &trajectoryOut)
{
	double *stateArr = new double[numDimensions_];
	for (int i = 0; i < numDimensions_; i++)
		stateArr[i] = stateToIn[i];
	trajectoryOut.push_front(stateArr);

	return 1;
}

double System::evaluateCostToGo(State2 &stateIn)
{

	double radius = 0.0;
	for (int i = 0; i < numDimensions_; i++)
		radius += regionGoal_.size[i] * regionGoal_.size[i];
	radius = sqrt(radius);

	double dist = 0.0;
	for (int i = 0; i < numDimensions_; i++)
		dist += (stateIn[i] - regionGoal_.center[i]) * (stateIn[0] - regionGoal_.center[i]);
	dist = sqrt(dist);

	return dist - radius;
}

void System::mapToWorld(double mx, double my, double &wx, double &wy)
{
	wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
	wy = costmap_->getOriginY() + (my + convert_offset_) * costmap_->getResolution();
}

bool System::worldToMap(double wx, double wy, double &mx, double &my)
{
	double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
	double resolution = costmap_->getResolution();

	if (wx < origin_x || wy < origin_y)
		return false;

	mx = (wx - origin_x) / resolution - convert_offset_;
	my = (wy - origin_y) / resolution - convert_offset_;

	if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
		return true;

	return false;
}

double System::convertDistance(double distanceWorld)
{
	return distanceWorld / costmap_->getResolution();
}

bool System::splineStateList(std::list<double *> &stateListIn, std::list<double *> &splinedListOut)
{
	for (list<double *>::iterator iter = stateListIn.begin(); iter != stateListIn.end(); iter++)
	{
		double *stateRef = *iter;
		// in all cases, adding the initial state
		splinedListOut.push_back(stateRef);

		// computing iterator on next element
		list<double *>::iterator iterNext = iter;
		iterNext++;
		if (iterNext != stateListIn.end())
		{
			// will test for need of intermediate points
			double *stateNext = *(iterNext);

			double dist = distance(stateRef, stateNext);
			if (dist > waypointDistance_)
			{
				int nInterp = 1 + dist / waypointDistance_;
				ROS_INFO("Would have needed interpolation d=%lf n=%d", dist, nInterp);
				double distX = (stateNext[0] - stateRef[0]) / nInterp;
				double distY = (stateNext[1] - stateRef[1]) / nInterp;
				for (int i = 1; i < nInterp; i++)
				{
					double *state = new double[2];
					state[0] = stateRef[0] + i * distX;
					state[1] = stateRef[1] + i * distY;
					splinedListOut.push_back(state);
				}
			}
			else
			{
				ROS_INFO("no interp : d=%lf refD=%lf", dist, waypointDistance_);
			}
		}
	}
	return true;
}