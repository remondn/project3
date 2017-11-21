#include "project2/functions.h"
#include <math.h>

float distance(point a, point b)
{
    return sqrt(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0));
}

float getDirection(point car_pose, point goal_pose)
{
    float dx = 0.0, dy = 0.0;
    float angle = car_pose.th;

    dx = goal_pose.x - car_pose.x;
    dy = goal_pose.y - car_pose.y;

    if (dx == 0) {return angle;}

    angle = atan(dy / dx);

    if (dx >= 0 && dy >=0) {
        angle = angle;
    }
    else if (dx < 0 && dy >= 0) {
        //angle = M_PI + angle;
        angle = M_PI + angle;
    }
    else if (dx < 0 && dy < 0) {
        // angle = -(M_PI - angle);
        angle = -(M_PI - angle);
    }
    else {
        angle = angle;
    }

    // if(angle < -M_PI)
    // {
    //     angle += 2 * M_PI;  // Equivalent
    // }
    // if(angle > M_PI)
    // {
    //     angle -= 2 * M_PI;  // Equivalent
    // }

    return angle;
}
