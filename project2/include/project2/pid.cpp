#include <project2/pid.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

#define PI 3.14159265358979323846

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

    error = 0;
    error_sum = 0;
    error_diff = 0;
    wg = 1.0;
    wd = 0.0;
    Kp = 1.0;
    Ki = 0;
    Kd = 0;
    iWasNegative = false;
}

void PID::initErrorSum() { error_sum = 0; }

float PID::get_control(point car_pose, traj prev_goal, traj cur_goal) {
    //TODO

     if(car_pose.th < -M_PI)
     {
      car_pose.th += 2 * M_PI;  // Equivalent
     }
     if(car_pose.th > M_PI)
     {
        car_pose.th -= 2 * M_PI;  // Equivalent
     }

    // 'ctrl' is the return value.
    // 'newError' being used for 'pidError', and ctrl is -(pidError)
    float newError = 0, pidError = 0, ctrl = 0;

    // pidError = pt + it + dt
    float pt = 0, it = 0, dt = 0;

    // The value of precise direction to the next goal.
    point goal;
    goal.x = cur_goal.x;
    goal.y = cur_goal.y;
    goal.th = cur_goal.th;
    float preciseDirection = getDirection(car_pose, goal);

    std::cout << "car_pose.th=" << car_pose.th << ", actually " << preciseDirection << std::endl;

    // Compute e(t) from the point of the car and the goal point
    // this->setWeight(car_pose, goal);
    // newError = car_pose.th - wg * preciseDirection - wd * goal.th;
    newError = car_pose.th - preciseDirection;

    if(fabs(newError) > M_PI) {
        if (car_pose.th * preciseDirection < 0)
        {
            if(car_pose.th > 0)
            {
                newError = (car_pose.th - M_PI) + (-M_PI - preciseDirection);
            }
            else
            {
                newError = (car_pose.th - -M_PI) + (M_PI - preciseDirection);
            }
        }
    }

    // Put 0 as the sum of error in case of changing the sign
    if ((newError > 0.0 && iWasNegative) || (newError < 0.0 && !iWasNegative))
    {
        error_sum = 0;
        iWasNegative = !iWasNegative;
    }

    // Compute the proportionnal term
    pt = Kp * newError;

    // Compute the Integral term
    it = Ki * DELTA_T * error_sum;

    // Compute Derivative term
    dt = (Kd / DELTA_T) * (newError - error);

    // Recursion form
    // ctrl = -error;
    pidError = pt + it + dt;
    ctrl = -pidError;

    // We will concern getting the limit of max_turn developed.
    float max_turn = 60.0*M_PI/180.0;
    ctrl = fmin(max_turn, ctrl);
    ctrl = fmax(-max_turn, ctrl);

    // Update PID
    error = newError;
    error_sum += newError;

    if(ctrl < -M_PI)
    {
        ctrl += 2 * M_PI;  // Equivalent
    }
    if(ctrl > M_PI)
    {
        ctrl -= 2 * M_PI;  // Equivalent
    }

    return ctrl;
}

double PID::set_speed(double min_speed, double max_speed, point car_pose, traj cur_goal, double rad_to_turn)
{
    double dist = sqrt(pow(car_pose.x - cur_goal.x, 2.0) + pow(car_pose.y - cur_goal.y, 2.0));

    return fmin(min_speed + dist/(1 + 8*fabs(rad_to_turn)), max_speed);
}

void PID::setWeight(point car, point goal)
{
    if(distance(car, goal) >= 1.0)
    {
        wg = 1.0;
        wd = 0.0;
        return;
    }

    wg = distance(car, goal);
    wd = 1.0 - wg;
}
