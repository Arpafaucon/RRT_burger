
#include <burger_local_planner/burger_trajectory_finder.h>
#include <math.h>

namespace burger_local_planner
{

BurgerTrajectoryFinder::BurgerTrajectoryFinder()
{
}

void BurgerTrajectoryFinder::initialize()
{
    xPos_[0] = 0.0;
    yPos_[0] = 0.0;
    thetaPos_[0] = 0.0;
    omega_ = 0.0;
    omegad_[0] = 0.0;
    omegad_[1] = 0.0;
    omegai_[0] = 0.0;
    omegai_[1] = 0.0;
    omegap_[0] = 0.0;
    omegap_[1] = 0.0;
    linearSpeed_[0] = 0.0;
}

void BurgerTrajectoryFinder::initialize(Eigen::Vector3f initialPose)
{
    xPos_[0] = initialPose[0];
    yPos_[0] = initialPose[1];
    thetaPos_[0] = initialPose[2];
    omega_ = 0.0;
    omegad_[0] = 0.0;
    omegad_[1] = 0.0;
    omegai_[0] = 0.0;
    omegai_[1] = 0.0;
    omegap_[0] = 0.0;
    omegap_[1] = 0.0;
    linearSpeed_[0] = 0.0;
}

base_local_planner::Trajectory BurgerTrajectoryFinder::findBestPath(Eigen::Vector3f goal, Eigen::Vector3f pose, float deltaTime, bool isLastGoal, bool allowReverse)
{
    base_local_planner::Trajectory traj;
    xPos_[1] = pose[0];
    yPos_[1] = pose[1];
    thetaPos_[1] = pose[2];

    if (goal[0] != xGoal_ || goal[1] != yGoal_)
    {
        xGoal_ = goal[0];
        yGoal_ = goal[1];
        ROS_INFO("trajectory finder got new goal");
    }

    // thetaGoal_ = goal(2);
    float requested_speed = SPEED_FACTOR * sqrt(pow((xGoal_ - xPos_[1]) / deltaTime, 2) + pow((yGoal_ - yPos_[1]) / deltaTime, 2));

    if (isLastGoal)
    {
        linearSpeed_[1] = requested_speed;
        ROS_INFO("Is last goal!");
    }
    else
    {
        // When it is not the true goal, no need to deccelerate when getting closer to the intermediary goal
        if (requested_speed < abs(linearSpeed_[0]))
        {
            linearSpeed_[1] = abs(linearSpeed_[0]);
        }
        else
        {
            linearSpeed_[1] = requested_speed;
        }
    }

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

    float thetaToGoalNow = atan((yGoal_ - yPos_[1]) / (xGoal_ - xPos_[1]));
    if ((xGoal_ - xPos_[1]) < 0)
    {
        if ((yGoal_ - yPos_[1]) > 0)
        {
            thetaToGoalNow += PI;
        }
        else
        {
            thetaToGoalNow -= PI;
        }
    }

    float thetaToGoalPrev = atan((yGoal_ - yPos_[0]) / (xGoal_ - xPos_[0]));
    if ((xGoal_ - xPos_[0]) < 0)
    {
        if ((yGoal_ - yPos_[0]) > 0)
        {
            thetaToGoalPrev += PI;
        }
        else
        {
            thetaToGoalPrev -= PI;
        }
    }

    float thetaDiffNow = (thetaToGoalNow - thetaPos_[1]);
    if (thetaDiffNow > PI)
    {
        thetaDiffNow -= 2 * PI;
    }
    if (thetaDiffNow < -PI)
    {
        thetaDiffNow += 2 * PI;
    }

    float thetaDiffPrev = (thetaToGoalPrev - thetaPos_[0]);
    if (thetaDiffPrev > PI)
    {
        thetaDiffPrev -= 2 * PI;
    }
    if (thetaDiffPrev < -PI)
    {
        thetaDiffPrev += 2 * PI;
    }

    if (allowReverse)
    {
        if (abs(thetaDiffNow) > PI / 2.0)
        {
            ROS_INFO("Going reverse mode");
            linearSpeed_[1] *= -1.0;
            if (thetaDiffNow > PI / 2.0)
            {
                thetaDiffNow -= PI;
            }
            if (thetaDiffNow < -PI / 2.0)
            {
                thetaDiffNow += PI;
            }
            if (thetaDiffPrev > PI / 2.0)
            {
                thetaDiffPrev -= PI;
            }
            if (thetaDiffPrev < -PI / 2.0)
            {
                thetaDiffPrev += PI;
            }
        }
    }

    linearSpeed_[1] /= (1.0 + 8 * abs(thetaDiffNow));

    // ROS_INFO("pose = (%f, %f, %f), goal =(%f, %f)", xPos_[1], yPos_[1], thetaPos_[1], xGoal_, yGoal_);

    // ROS_INFO("thetaToGoalNow = %f, thetaNow = %f", thetaToGoalNow, thetaPos_[1]);

    omegap_[2] = Kp * thetaDiffNow;
    omegai_[2] = omegai_[0] + Ki * (deltaTime / 2.0) * (thetaDiffNow + thetaDiffPrev);
    omegad_[2] = -omegad_[0] + Kd * (2.0 / deltaTime) * (thetaDiffNow - thetaDiffPrev);
    omega_ = omegad_[2] + omegai_[2] + omegap_[2];

    if (omega_ > OMEGA_MAX)
    {
        omega_ = OMEGA_MAX;
    }

    if (omega_ < -OMEGA_MAX)
    {
        omega_ = -OMEGA_MAX;
    }

    speedRight_ = linearSpeed_[1] + (omega_ * L) / 2.0;
    speedLeft_ = linearSpeed_[1] - (omega_ * L) / 2.0;

    if (abs(speedLeft_) > WHEEL_MAX_SPEED)
    {
        speedLeft_ *= WHEEL_MAX_SPEED / abs(speedLeft_);
        speedRight_ *= WHEEL_MAX_SPEED / abs(speedLeft_);
        linearSpeed_[1] = speedLeft_ + (omega_ * L) / 2.0;
    }

    if (abs(speedRight_) > WHEEL_MAX_SPEED)
    {
        speedLeft_ *= WHEEL_MAX_SPEED / abs(speedRight_);
        speedRight_ *= WHEEL_MAX_SPEED / abs(speedRight_);
        linearSpeed_[1] = speedLeft_ + (omega_ * L) / 2.0;
    }

    traj.xv_ = linearSpeed_[1];
    traj.yv_ = 0.0;
    traj.thetav_ = omega_;
    traj.cost_ = 1;

    // ROS_INFO("Trajectory is V = %lf \n     Omega = %lf \n", linearSpeed_[1], omega_[2]);

    Eigen::Vector3f velocity;
    velocity[0] = linearSpeed_[1];
    velocity[1] = 0.0;
    velocity[2] = omega_;

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
}