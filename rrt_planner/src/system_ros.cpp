#include "system_ros.h"
#include <cmath>
#include <cstdlib>

#include <iostream>

using namespace std;
using namespace Burger2D;

#define DISCRETIZATION_STEP 0.01

region2::region2()
{
}

region2::region2(int xleft, int xright, int ybottom, int yup)
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

System::System()
{

	// numDimensions = 0;
}

System::~System()
{
	cout << "System destroyed";
}

// int System::setNumDimensions(int numDimensionsIn)
// {

// 	if (numDimensions < 0)
// 		return 0;

// 	numDimensions = numDimensionsIn;

// 	rootState.setNumDimensions(numDimensions);

// 	return 1;
// }

int System::getStateKey(State2 &stateIn, double *stateKey)
{

	for (int i = 0; i < numDimensions; i++)
		stateKey[i] = stateIn.x[i] / regionOperating.size[i];

	return 1;
}

bool System::isReachingTarget(State2 &stateIn)
{

	for (int i = 0; i < numDimensions; i++)
	{

		if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i] / 2.0)
			return false;
	}

	return true;
}

bool System::IsInCollision(int *stateIn)
{

	for (list<region2 *>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++)
	{

		region2 *obstacleCurr = *iter;
		bool collisionFound = true;

		for (int i = 0; i < numDimensions; i++)
			if (fabs(obstacleCurr->center[i] - stateIn[i]) > obstacleCurr->size[i] / 2.0)
			{
				collisionFound = false;
				break;
			}

		if (collisionFound)
		{
			return true;
		}
	}

	return false;
}

int System::sampleState(State2 &randomStateOut)
{

	// randomStateOut.setNumDimensions(numDimensions);

	for (int i = 0; i < numDimensions; i++)
	{

		randomStateOut.x[i] = (double)rand() / (RAND_MAX + 1.0) * regionOperating.size[i] - regionOperating.size[i] / 2.0 + regionOperating.center[i];
	}

	if (IsInCollision(randomStateOut.x))
		return 0;

	return 1;
}

int System::extendTo(State2 &stateFromIn, State2 &stateTowardsIn, Trajectory &trajectoryOut, bool &exactConnectionOut)
{

	double *dists = new double[numDimensions];
	for (int i = 0; i < numDimensions; i++)
		dists[i] = stateTowardsIn.x[i] - stateFromIn.x[i];

	double distTotal = 0.0;
	for (int i = 0; i < numDimensions; i++)
		distTotal += dists[i] * dists[i];
	distTotal = sqrt(distTotal);

	double incrementTotal = distTotal / DISCRETIZATION_STEP;

	// normalize the distance according to the disretization step
	for (int i = 0; i < numDimensions; i++)
		dists[i] /= incrementTotal;

	int numSegments = (int)floor(incrementTotal);

	int stateCurr[numDimensions];
	for (int i = 0; i < numDimensions; i++)
		stateCurr[i] = stateFromIn.x[i];

	for (int i = 0; i < numSegments; i++)
	{

		if (IsInCollision(stateCurr))
			return 0;

		for (int i = 0; i < numDimensions; i++)
			stateCurr[i] += dists[i];
	}

	if (IsInCollision(stateTowardsIn.x))
		return 0;

	trajectoryOut.endState = new State2(stateTowardsIn);
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
	for (int i = 0; i < numDimensions; i++)
	{
		double distCurr = stateTowardsIn.x[i] - stateFromIn.x[i];
		distTotal += distCurr * distCurr;
	}

	return sqrt(distTotal);
}

int System::getTrajectory(State2 &stateFromIn, State2 &stateToIn, list<double*> &trajectoryOut)
{
	double *stateArr = new double[numDimensions];
	for (int i = 0; i < numDimensions; i++)
		stateArr[i] = stateToIn[i];
	trajectoryOut.push_front(stateArr);

	return 1;
}

double System::evaluateCostToGo(State2 &stateIn)
{

	double radius = 0.0;
	for (int i = 0; i < numDimensions; i++)
		radius += regionGoal.size[i] * regionGoal.size[i];
	radius = sqrt(radius);

	double dist = 0.0;
	for (int i = 0; i < numDimensions; i++)
		dist += (stateIn[i] - regionGoal.center[i]) * (stateIn[0] - regionGoal.center[i]);
	dist = sqrt(dist);

	return dist - radius;
}
