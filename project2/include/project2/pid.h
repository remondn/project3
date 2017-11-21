#pragma once

#include <cmath>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/traj.h>
#endif
#include "project2/functions.h"

// We got new message at frequency 10Hz => 1/10 seconds
#define DELTA_T 1

class PID{
public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    float get_control(point car_pose, traj prev_goal, traj cur_goal);
    double set_speed(double min_speed, double max_speed, point car_pose, traj cur_goal, double rad_to_turn);
    void initErrorSum();
    void setWeight(point car, point goal);
private:
    float error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
    bool iWasNegative;
    float wg;
    float wd;
};
