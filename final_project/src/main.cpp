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

// [FROM PROJECT 4]
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
//parameters [FROM PROJECT 4]
int margin = 6;
int K = 1500;
double MaxStep = 2;
int waypoint_margin = 24;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;
std::vector<traj>::iterator currentGoal;    // [FROM PROJECT 4]
traj prevGoal;                              // [FROM PROJECT 4]

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
PID pid_ctrl;       // [FROM PROJECT 4]

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

int main(int argc, char** argv){
    // Seed Random [FROM PROJECT 4]
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
                              std::string("/catkin_ws/src/final_project/src/final.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.7;
            world_x_max = 4.7;
            world_y_min = -10.2;
            world_y_max = 10.2;
            res = 0.05;
            printf("Load map\n");

            // Correct values of MAP [FROM PROJECT 4]
            for(int k = 0; k < map.cols; k++)
            {
                for(int l = 0; l < map.rows; l++)
                {
                    if(map.at<uchar>(l, k) != 0 && map.at<uchar>(l, k) != 125)
                    {
                        map.at<uchar>(l, k) = 255;
                    }

                    // DEBUG
                    // std::cout << (int)map.at<uchar>(l, k) << std::endl;
                }
            }

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            // Initialise the iterator to access the right point of the path [FROM PROJECT 4]
            currentGoal = path_RRT.begin();
            prevGoal = *currentGoal;

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;

        case RUNNING: {
            // TODO 1

            // PART IMPORTED [FROM PROJECT 4]

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
    point waypoint_candid[7];

    // Starting point. (Fixed)
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 8.5;


    //TODO 2
    // Set your own waypoints.
    // The car should turn around the outer track once, and come back to the starting point.
    // This is an example.
    // waypoint_candid[1].x = 2.2;
    // waypoint_candid[1].y = 8.5;
    // waypoint_candid[2].x = 2.5;
    // waypoint_candid[2].y = -8.5;
    // waypoint_candid[3].x = -2.5;
    // waypoint_candid[3].y = -8.0;
    // waypoint_candid[4].x = -3.5;
    // waypoint_candid[4].y = 8.5;

    // Set the margin off the extern wall
    double marginFromBorderX = -(world_x_min - waypoint_candid[0].x) + 0.0;
    double marginFromBorderY = (world_y_max - waypoint_candid[0].y) - 0.4;
    // Set the middle of map for both x and y
    double midX = world_x_min + ((world_x_max - world_x_min) / 2);
    double midY = world_y_min + ((world_y_max - world_y_min) / 2);

    // Choose waypoints : not in the corner : in the extern side of walls

    // WP 1 : right wall
    waypoint_candid[1].x = midX;
    waypoint_candid[1].y = world_y_max - marginFromBorderY;

    // WP 2 : down wall
    waypoint_candid[2].x = world_x_max - marginFromBorderX;
    waypoint_candid[2].y = midY;

    // WP 3 : left wall
    waypoint_candid[3].x = midX;
    waypoint_candid[3].y = world_y_min + marginFromBorderY;

    // WP 4 : upper wall
    waypoint_candid[4].x = world_x_min + marginFromBorderX;
    waypoint_candid[4].y = midY;
    
    // Waypoints for arbitrary goal points.
    // TA will change this part before scoring.
    // This is an example.
    waypoint_candid[5].x = 1.5;
    waypoint_candid[5].y = 1.5;
    waypoint_candid[6].x = -2;
    waypoint_candid[6].y = -9.0;

    int order[] = {0,1,2,3,4,0,5,6};
    int order_size = 8;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);

        //DEBUG
        // std::cout << "Point " << i << " = " << waypoint_candid[order[i]].x << " | " << waypoint_candid[order[i]].y << std::endl;
    }
    // std::cin.get();
}

void generate_path_RRT()
{   
    // TODO 1

    // PART IMPORTED [FROM PROJECT 4]

    //DEBUG
    std::vector<traj> pathMem[7];
    rrtTree treeMem[7];
    int pathNb = 0;

    // Generate a new map for the first turn : close the center so we are sure we are turning
    cv::Mat mapTurn = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows
    int jMarginBorder = (jSize / 10) * 2;
    int iMarginBorder = (iSize / 10) * 2;

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if(i > iMarginBorder && i < (iSize - iMarginBorder) && j > jMarginBorder && j < (jSize - jMarginBorder))
            {
                // If we are at the center, color it black
                mapTurn.at<uchar>(i, j) = 0;
            }
        }
    }

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
        pathNb = 0;

        for(it = waypoints.begin() + 1; it != waypoints.end(); it++)
        {
            int isRRTValid;
            rrtTree t;
            // DEBUG
            // std::cout << "Generation of path for waypoint " << c << "->" << c+1 << std::endl;

            // Create the tree for the current waypoint
            // t = rrtTree(lastPoint, *it, map, map_origin_x, map_origin_y, res, margin);
            if(pathNb < 5)
            {
                // If it's the first turn, use the hand-colored map
                t = rrtTree(lastPoint, *it, mapTurn, map_origin_x, map_origin_y, res, margin);
            }
            else
            {
                t = rrtTree(lastPoint, *it, map, map_origin_x, map_origin_y, res, margin);
            }

            // DEBUG
            treeMem[pathNb] = t;
        
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

            //DEBUG
            //t.visualizeTree(pathI);
            //std::cin.get();
            // t.optimizeTree();

            // DEBUG
            pathMem[pathNb] = pathI;
    
            // Optimize it (straight line)
            pathI = t.optimizePath(pathI);
    
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
            pathNb++;
        }

        // Copy the path a second time for 2 turn
        //path_RRT.insert(path_RRT.end(), path_RRT.begin(), path_RRT.end());

        // Test if a full path is produced
        if(path_RRT.size() <= 20 && !crash)
        {
            // Not enough point generated, try again
            std::cout << "Crashed : " << path_RRT.size() << std::endl;
            crash = true;
        }
        // if(crash)
        // {
        //     set_waypoints();
        // }
    }
    return;

    // DEBUG
    std::cout << "Display of complete final path step by step.. " << std::endl;
    for(int i = 0; i < pathNb; i++)
    {
        std::cout << "Step " << i << " : " << std::endl;
        treeMem[i].visualizeTree(pathMem[i]);
        std::cin.get();
    }

    return;
}
