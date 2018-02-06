//
//
//  PIRobot : Unicycle robot model + PID
//  main.cpp
//
//
//  Created by Adina Marlena Panchea on 5/22/17.
//  Copyright © 2017 Adina M. Panchea. All rights reserved.
//
//

#include <iostream>
#include <vector>
#include "math.h"

// typedef std::vector<double> vd;

int main()
{

    const float L = 0.102, // distance between middle of wheels
        // R = 0.032, // radius of a wheel
        f = 30,     // frequency Hz
        dt = 1 / f, // sampling time
        Te = dt,
                time = 10, // samples
        xi = 0,
                yi = 0,
                thetai = 0.01,
                xg = 4,
                yg = 5,
                //thetag = 0.01,
        vmin = -0.63,
                vmax = 0.63, // calculated empirically
        // PID coeff determined with Ziegler-Nichols(see ZNPIDtuning.m)
        Kcritic = 30.4,
                Tcritic = 0.2;
    //
    // %P%%%%%%%%%%%%%%%%%%%%
    // % Kp = 0.5*Kcritic;
    // % Ki = 0;% attention ... the way you write the pid eq
    // % Kd = 0;
    // %PI%%%%%%%%%%%%%%%%%%%
    float Kp = 0.45 * Kcritic;
    float Ki = (1 / 1.2) * Tcritic;
    float Kd = 0;
    // %PID%%%%%%%%%%%%%%%%%%
    // % Kp = 0.6*Kcritic;
    // % Ki = 0.5*Tcritic;
    // % Kd = 0.125*Tcritic;
    //
    float omegamax = 2 * vmax / L;
    float omegamin = 2 * vmin / L; // rad/s (considering that both wheels have the same max speed)

    // std::cout << "Hello there 1 !\n";

    float t[int(round(time / Te))];   // time
    int N = sizeof(t) / sizeof(t[0]); //sampling time

    // std::cout << "Hello there 2 !\n";

    // calculate time t = 0 : Te : time
    t[0] = 0;
    for (int ij = 1; ij < N; ij++)
    {
        t[ij] = t[ij - 1] + Te;
    }

    // std::cout << "Hello there 3 !\n";

    float x[N];
    float y[N];
    float theta[N];
    x[0] = xi;
    y[0] = yi;
    theta[0] = thetai;
    x[1] = xi;
    y[1] = yi;
    theta[1] = thetai; // the PID needs the first two states
    float v[N];
    float omega[N];
    //for PID controller p i and d
    float omegap[N];
    float omegai[N];
    float omegad[N];
    float vr[N];
    float vl[N];

    // std::cout << "Hello there 4 !\n";

    //float thetad[N];
    //for (int ij = 0; ij < N; ij ++){
    //    thetad[ij] = atan((yg-y[ij])/(xg-x[ij]));
    //}

    // std::cout << "Hello there 5 !\n";

    float thetad1 = 0;
    float thetad2 = 0;

    v[0] = 0;
    v[1] = 0;
    omega[0] = 0;
    omega[1] = 0;
    omegap[0] = 0;
    omegap[1] = 0;
    omegai[0] = 0;
    omegai[1] = 0;
    omegad[0] = 0;
    omegad[1] = 0;
    vr[0] = 0;
    vr[1] = 0;
    vl[0] = 0;
    vl[1] = 0;

    for (int ij = 1; ij < N - 1; ij++)
    {
        x[ij + 1] = x[ij] + Te * ((vr[ij] + vl[ij]) / 2) * cos(theta[ij]);
        y[ij + 1] = y[ij] + Te * ((vr[ij] + vl[ij]) / 2) * sin(theta[ij]);
        theta[ij + 1] = theta[ij] + Te * ((vr[ij] - vl[ij]) / L);

        // std::cout << "Hello there 6 !\n";

        v[ij + 1] = sqrt(pow((xg - x[ij]) / dt, 2) + pow((yg - y[ij]) / dt, 2));
        if (v[ij + 1] > vmax)
        {
            v[ij + 1] = vmax;
        }
        else if (v[ij + 1] < vmin)
        {
            v[ij + 1] = vmin;
        }

        //std::cout << "Hello there 7 !\n";

        thetad1 = atan((yg - y[ij]) / (xg - x[ij]));
        thetad2 = atan((yg - y[ij - 1]) / (xg - x[ij - 1]));

        omegap[ij + 1] = Kp * (thetad1 - theta[ij]);
        omegai[ij + 1] = omegai[ij - 1] + Ki * (Te / 2) * (thetad1 - theta[ij] + thetad2 - theta[ij - 1]);
        omegad[ij + 1] = -omegad[ij - 1] + ((2 * Kd) / Te) * ((thetad1 - theta[ij]) - (thetad2 - theta[ij - 1]));
        omega[ij + 1] = omegap[ij + 1] + omegai[ij + 1] + omegad[ij + 1];
        if (omega[ij + 1] > omegamax)
        {
            omega[ij + 1] = omegamax;
        }
        else if (omega[ij + 1] < omegamin)
        {
            omega[ij + 1] = omegamin;
        }

        //std::cout << "Hello there 8 !\n";

        vr[ij + 1] = v[ij + 1] + (omega[ij + 1] * L) / 2;
        vl[ij + 1] = v[ij + 1] - (omega[ij + 1] * L) / 2;
    }

    std::cout << "position on x!\n";
    for (int ij = 0; ij < N; ij++)
    {
        // printf("%f \n", vr[ij]);
        //printf("%f \n", vl[ij]);
        printf("%f \n", x[ij]);
        //printf("%f \n", y[ij]);
    }

    std::cout << "position on y!\n";
    for (int ij = 0; ij < N; ij++)
    {
        // printf("%f \n", vr[ij]);
        //printf("%f \n", vl[ij]);
        //  printf("%f \n", x[ij]);
        printf("%f \n", y[ij]);
    }

    return 0;
}
