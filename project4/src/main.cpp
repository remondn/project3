//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

// [FROM PROJECT 3]
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include "project2/functions.h"
#define MAX_SPEED 2.0

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
// int margin = 5;
// int K = 500;
// double MaxStep = 2;
//int waypoint_margin = 23;
int waypoint_margin = 23;

// TEST
int cc = 0;

//parameters [FROM PROJECT 3]
int margin = 3;
int K = 1500;
double MaxStep = 2;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;
std::vector<traj>::iterator currentGoal;    // [FROM PROJECT 3]
traj prevGoal;                              // [FROM PROJECT 3]   

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
PID pid_ctrl;       // [FROM PROJECT 3]

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

int main(int argc, char** argv){
    // Seed Random [FROM PROJECT 3]
    srand (static_cast <unsigned> (time(0)));

    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
            
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/project4/src/slam_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
            res = 0.05;
            printf("Load map\n");

            // Correct values of MAP [FROM PROJECT 3]
            for(int k = 0; k < map.cols; k++)
            {
                for(int l = 0; l < map.rows; l++)
                {
                    if(map.at<uchar>(l, k) == 250)
                    {
                        map.at<uchar>(l, k) = 255;
                    }
                }
            }

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;

            // TEST
            //state = RUNNING;
        } break;

        case PATH_PLANNING:{
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            // Initialise the iterator to access the right point of the path [FROM PROJECT 3]
            currentGoal = path_RRT.begin();
            prevGoal = *currentGoal;

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;
        } break;

        case RUNNING: {
            // TODO 1

            // TEST
            // setcmdvel(0.1, 60.0*3.14/180.0);
            // cmd_vel_pub.publish(cmd);
            // ros::spinOnce();
            // control_rate.sleep();
            // break;

            // PART IMPORTED [FROM PROJECT 3]

            // Step 1 : Update the steering angle with PID algorithm
            double turn = pid_ctrl.get_control(robot_pose, prevGoal, *currentGoal);
            double speed = pid_ctrl.set_speed(0.75, MAX_SPEED, robot_pose, *currentGoal, turn);
            //double speed = pid_ctrl.set_speed(0.45, MAX_SPEED, robot_pose, *currentGoal, turn);

            // DEBUG
            std::cout << "Turn <" << turn << \
                " (" << robot_pose.x << "," << robot_pose.y << ")>>(" << (*currentGoal).x << "," << (*currentGoal).y << ") " \
                "with Speed " << speed << \
                std::endl;
            setcmdvel(speed, turn);
            // DEBUG
            // setcmdvel(1.0, 60.0*3.14/180.0);

            // Step 2 : Publish the new data
            cmd_vel_pub.publish(cmd);

            // Step 3 & 4 : Check if we reached the point
            point goal;
            goal.x = currentGoal->x;
            goal.y = currentGoal->y;
            goal.th = currentGoal->th;

            if(distance(robot_pose, goal) < 0.2)
            {
                //DEBUG
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                std::cout << "Point Reached !" << std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

                // If the distance is lower than 0.2 meters, then we change the goal
                pid_ctrl.initErrorSum();
                prevGoal = *currentGoal;
                currentGoal++;
            }

            //Step 5 : Check if we reached our final destination
            if(currentGoal == path_RRT.end())
            {
                // We loved you dear robot, but it's time to say goodbye, maybe forever...
                state = FINISH;

                //DEBUG
                std::cout << "___________________________________" << std::endl;
                std::cout << "END OF PATH !" << std::endl;
                std::cout << "___________________________________" << std::endl;
            }

            ros::spinOnce();
            control_rate.sleep();
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;
        default: {
        } break;
        }
    }
    return 0;
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    // [TA CODE]
    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
                    for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
                        if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    // TODO 2
    // Make your own code to select waypoints.
    // You can randomly sample some points from the map.
    // Also, the car should follow the track in clockwise.

    // Divide the map into 4 corners (clockwise).
    int jStart[4] = {(jSize / 2) + 1, (jSize / 2) + 1, 0, 0};   // Column starting point of the corners
    int jEnd[4] = {jSize, jSize, jSize / 2, jSize / 2};         // Column ending point of the corners
    int iStart[4] = {0, (iSize / 2) + 1, (iSize / 2) + 1, 0};   // Row starting point of the corners
    int iEnd[4] = {iSize / 2, iSize, iSize, iSize / 2};         // Row ending point of the corners

    // Find to which corner the start point belong
    int corner = -1;
    for(int i = 0; i < 4; i++)
    {
        int si = ((int)floor(waypoint_candid[0].x / res)) + map_origin_x;
        int sj = ((int)floor(waypoint_candid[0].y / res)) + map_origin_y;
        if(si >= iStart[i] && si <= iEnd[i] && sj >= jStart[i] && sj <= jEnd[i])
        {
            // It's this corner !
            corner = i;
        }
    }
    // DEBUG
    // std::cout << "CORNER = " << corner << std::endl;
    // std::cout << ((int)floor(waypoint_candid[0].x / res)) + map_origin_x << " , " << ((int)floor(waypoint_candid[0].y / res)) + map_origin_y << std::endl;

    // We have 3 points to assign : 0 and 4 is already assigned (start & goal)
    // We need to assign them is the right corner
    corner = (corner + 1) % 4;    // Turn clockwise
    int randi;
    int randj;
    for(int p = 1; p < 4; p++)
    {
        // Generate randomly a valid point inside the corner
        do
        { 
            // Randomly generate a point
            randj = rand() % (jEnd[corner] - jStart[corner] + 2) + jStart[corner];
            randi = rand() % (iEnd[corner] - iStart[corner] + 2) + iStart[corner];

            // DEBUG
            // std::cout << "Map<" << randi << ", " << randj << "> = " << (int)map_margin.at<uchar>(randi, randj) << std::endl;
        }while((int)map_margin.at<uchar>(randi, randj) != 255);

        // Once we have a valid point, save it
        waypoint_candid[p].x = res * (randi - map_origin_x);
        waypoint_candid[p].y = res * (randj - map_origin_y);

        // Update the corner for next point
        corner = (corner + 1) % 4;    // Turn clockwise
    }

    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 12.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    // First turn
    waypoints.clear();
    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }

    // // Second turn (Do not add the start point, because it's the same as the last goal point)
    // for(int i = 1; i < order_size; i++){
    //     if(i == order_size - 1)
    //     {
    //         waypoint_candid[order_size - 1].x = -3.5;
    //         waypoint_candid[order_size - 1].y = 12.0;
    //     }
    //     waypoints.push_back(waypoint_candid[order[i]]);
    // }
}

void generate_path_RRT()
{
    // TODO 1

    // PART IMPORTED [FROM PROJECT 3]

    bool crash = true;
    while(crash)
    {
        crash = false;
        path_RRT.clear();

        // Iterate through all way point
        std::vector<point>::iterator it = waypoints.begin();
        point lastPoint = *it;
        point previousLastPoint = lastPoint;
        int c = 1;

        for(it = waypoints.begin() + 1; it != waypoints.end(); it++)
        {
            int isRRTValid;
            rrtTree t;
            // DEBUG
            std::cout << "Generation of path for waypoint " << c << "->" << c+1 << std::endl;

            // Create the tree for the current waypoint
            t = rrtTree(lastPoint, *it, map, map_origin_x, map_origin_y, res, margin);
        
            // Generate the RRT Tree
            isRRTValid = t.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
            c++;

            if(isRRTValid == -1)
            {
                // DEBUG
                std::cout << "Crashed" << std::endl;

                crash = true;
                break;
            }
        
            // Get the backtracking path
            std::vector<traj> pathI = t.backtracking_traj();
    
            // Optimize it (straight line)
            pathI = t.optimizePath(pathI);
    
            //DEBUG
            //t.visualizeTree(pathI);
            //std::cin.get();
            // t.optimizeTree();
    
            // Add it to the total path
            path_RRT.insert(path_RRT.end(), pathI.begin(), pathI.end());
    
            //DEBUG
            //t.visualizeTree(path_RRT);
            //std::cin.get();
    
            // Update for the next loop
            previousLastPoint = lastPoint;
            lastPoint.x = path_RRT.back().x;
            lastPoint.y = path_RRT.back().y;
            lastPoint.th = path_RRT.back().th;
        }

        // Copy the path a second time for 2 turn
        path_RRT.insert(path_RRT.end(), path_RRT.begin(), path_RRT.end());

        // Test if a full path is produced
        if(path_RRT.size() <= 20 && !crash)
        {
            // Not enough point generated, try again
            std::cout << "Crashed : " << path_RRT.size() << std::endl;
            crash = true;
        }
        if(crash)
        {
            set_waypoints();
        }
    }
    return;

    //OLD

    // // Iterate through all way point
    // std::vector<point>::iterator it = waypoints.begin();
    // point lastPoint = *it;
    // point previousLastPoint = lastPoint;
    // int c = 1;

    // for(it = waypoints.begin() + 1; it != waypoints.end(); it++)
    // {
    //     // DEBUG
    //     std::cout << "Generation of path for waypoint " << c << "->" << c+1 << std::endl;
    //     c++;

    //     // Create the tree for the current waypoint
    //     rrtTree t = rrtTree(lastPoint, *it, map, map_origin_x, map_origin_y, res, margin);
    
    //     // Generate the RRT Tree
    //     int isRRTValid = t.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);

    //     if(!isRRTValid)
    //     {
    //         std::cout << "Error : Path not valid." << std::endl;
    //     }
    
    //     // Get the backtracking path
    //     std::vector<traj> pathI = t.backtracking_traj();

    //     // Optimize it (straight line)
    //     pathI = t.optimizePath(pathI);

    //     //DEBUG
    //     // t.visualizeTree(pathI);
    //     // std::cin.get();
    //     // t.optimizeTree();

    //     // Add it to the total path
    //     path_RRT.insert(path_RRT.end(), pathI.begin(), pathI.end());

    //     //DEBUG
    //     //t.visualizeTree(path_RRT);
    //     //std::cin.get();

    //     // Update for the next loop
    //     previousLastPoint = lastPoint;
    //     lastPoint.x = path_RRT.back().x;
    //     lastPoint.y = path_RRT.back().y;
    //     lastPoint.th = path_RRT.back().th;
    // }
    // return;
}
