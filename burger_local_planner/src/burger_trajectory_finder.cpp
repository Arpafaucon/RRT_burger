
#include <burger_local_planner/burger_trajectory_finder.h>

namespace burger_local_planner
{

BurgerTrajectoryFinder::BurgerTrajectoryFinder()
{
}

void BurgerTrajectoryFinder::initialize()
{
    xPos_[0] = 0;
    yPos_[0] = 0;
    thetaPos_[0] = 0;
    omega_[0] = 0;
    omega_[1] = 0;
    omegad_[0] = 0;
    omegad_[1] = 0;
    omegai_[0] = 0;
    omegai_[1] = 0;
    omegap_[0] = 0;
    omegap_[1] = 0;
    linearSpeed_[0] = 0;
}

void BurgerTrajectoryFinder::initialize(Eigen::Vector3f initialPose)
{
    xPos_[0] = initialPose[0];
    yPos_[0] = initialPose[1];
    thetaPos_[0] = initialPose[2];
    omega_[0] = 0;
    omega_[1] = 0;
    omegad_[0] = 0;
    omegad_[1] = 0;
    omegai_[0] = 0;
    omegai_[1] = 0;
    omegap_[0] = 0;
    omegap_[1] = 0;
    linearSpeed_[0] = 0;
}

base_local_planner::Trajectory BurgerTrajectoryFinder::findBestPath(Eigen::Vector3f goal, Eigen::Vector3f pose, float deltaTime)
{
    base_local_planner::Trajectory traj;
    xPos_[1] = pose(0);
    yPos_[1] = pose(1);
    thetaPos_[1] = pose(2);

    xGoal_ = goal(0);
    yGoal_ = goal(1);
    // thetaGoal_ = goal(2);

    linearSpeed_[1] = sqrt(pow((xGoal_ - xPos_[1]) / deltaTime, 2) + pow((yGoal_ - yPos_[1]) / deltaTime, 2));
    if (linearSpeed_[1] - linearSpeed_[0] >= SPEED_DELTA_MAX)
    {
        linearSpeed_[1] = linearSpeed_[0] + SPEED_DELTA_MAX;
    }

    if (linearSpeed_[1] > SPEED_MAX)
    {
        linearSpeed_[1] = SPEED_MAX;
    }

    if (linearSpeed_[1] < SPEED_MIN)
    {
        linearSpeed_[1] = SPEED_MIN;
    }

    float thetaDiffNow = atan((yGoal_ - yPos_[1]) / (xGoal_ - xPos_[1]));
    float thetaDiffPrev = atan((yGoal_ - yPos_[0]) / (xGoal_ - xPos_[0]));

    omegap_[2] = Kp * (thetaDiffNow - thetaPos_[1]);
    omegai_[2] = omegai_[0] + Ki * (deltaTime / 2.0) * (thetaDiffNow - thetaPos_[1] + thetaDiffPrev - thetaPos_[0]);
    omegad_[2] = -omegad_[0] + Kd * (2.0 / deltaTime) * ((thetaDiffNow - thetaPos_[1]) - (thetaDiffPrev - thetaPos_[0]));
    omega_[2] = omegad_[2] + omegai_[2] + omegap_[2];

    if (omega_[2] > OMEGA_MAX)
    {
        omega_[2] = OMEGA_MAX;
    }

    speedRight_ = linearSpeed_[1] + (omega_[2] * L) / 2.0;
    speedLeft_ = linearSpeed_[1] - (omega_[2] * L) / 2.0;

    if (abs(speedLeft_) > WHEEL_MAX_SPEED)
    {
        speedLeft_ *= WHEEL_MAX_SPEED / abs(speedLeft_);
        speedRight_ *= WHEEL_MAX_SPEED / abs(speedLeft_);
        linearSpeed_[1] = speedLeft_ + (omega_[2] * L) / 2.0;
    }

    if (abs(speedRight_) > WHEEL_MAX_SPEED)
    {
        speedLeft_ *= WHEEL_MAX_SPEED / abs(speedRight_);
        speedRight_ *= WHEEL_MAX_SPEED / abs(speedRight_);
        linearSpeed_[1] = speedLeft_ + (omega_[2] * L) / 2.0;
    }

    traj.xv_ = linearSpeed_[1];
    traj.xy_ = 0.0;
    traj.thetav_ = omega_[2];
    traj.cost_ = 1;

    Eigen::Vector3f velocity;
    velocity[0] = linearSpeed_[1];
    velocity[1] = 0.0;
    velocity[2] = omega_[2];

    int numSteps = 10;
    float dt = deltaTime / (float)numSteps;

    for (int i = 0; i < numSteps; i++)
    {
        traj.addPoint(pose[0], pose[1], pose[2]);
        pose = computeNewPositions(pose, velocity, dt);
    }

    // TO DO : add score trajectory with obstacles!!

    // Now, we should increase one step in our memory:
    xPos_[0] = xPos_[1];
    yPos_[0] = yPos_[1];
    thetaPos_[0] = thetaPos_[1];
    linearSpeed_[0] = linearSpeed_[1];
    omega_[0], omega_[1] = omega_[1], omega_[2];
    omegad_[0], omegad_[1] = omegad_[1], omegad_[2];
    omegai_[0], omegai_[1] = omegai_[1], omegai_[2];
    omegap_[0], omegap_[1] = omegap_[1], omegap_[2];

    return traj;
}

Eigen::Vector3f BurgerTrajectoryFinder::computeNewPositions(const Eigen::Vector3f &pos,
                                                            const Eigen::Vector3f &vel, double dt)
{
    Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
    new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
    new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
    new_pos[2] = pos[2] + vel[2] * dt;
    return new_pos;
}
{