//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

// [FROM PROJECT 2]
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
int margin = 8;
int K = 500;
double MaxStep = 2;

//parameters [FROM PROJECT 2]
/*
int margin = 15;
int K = 1500;
double MaxStep = 4;
*/

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
gazebo_msgs::ModelStatesConstPtr model_states;
PID pid_ctrl;       // [FROM PROJECT 2]

//FSM state
int state;

//function definition
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    // Seed Random [FROM PROJECT 2]
    srand (static_cast <unsigned> (time(0)));

    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",100,callback_state);
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/output",100);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map

    char* user = getpwuid(getuid())->pw_name;
    map = cv::imread((std::string("/home/") + std::string(user) +
                      std::string("/catkin_ws/src/project3/src/ground_truth_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

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


     if(! map.data )                              // Check for invalid input
    {
        printf("Could not open or find the image\n");
        return -1;
    }

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // Initialise the iterator to access the right point of the path [FROM PROJECT 2]
    std::vector<traj>::iterator currentGoal = path_RRT.begin();
    traj prevGoal = *currentGoal;

    // FSM
    state = INIT;
    bool running = true;
    int look_ahead_idx;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;
	    printf("path size : %d\n", path_RRT.size());
            //visualize path
	    ros::spinOnce();
            for(int i = 0; i < path_RRT.size(); i++){
		for(int j = 0; j < model_states->name.size(); j++){
                    std::ostringstream ball_name;
                    ball_name << i;
            	    if(std::strcmp(model_states->name[j].c_str(), ball_name.str().c_str()) == 0){
                        //initialize robot position
                        geometry_msgs::Pose model_pose;
                        model_pose.position.x = path_RRT[i].x;
                        model_pose.position.y = path_RRT[i].y;
                        model_pose.position.z = 0.7;
                        model_pose.orientation.x = 0.0;
                        model_pose.orientation.y = 0.0;
                        model_pose.orientation.z = 0.0;
                        model_pose.orientation.w = 1.0;

                        geometry_msgs::Twist model_twist;
                        model_twist.linear.x = 0.0;
                        model_twist.linear.y = 0.0;
                        model_twist.linear.z = 0.0;
                        model_twist.angular.x = 0.0;
                        model_twist.angular.y = 0.0;
                        model_twist.angular.z = 0.0;

                        gazebo_msgs::ModelState modelstate;
                        modelstate.model_name = ball_name.str();
                        modelstate.reference_frame = "world";
                        modelstate.pose = model_pose;
                        modelstate.twist = model_twist;

                        gazebo_msgs::SetModelState setmodelstate;
                        setmodelstate.request.model_state = modelstate;

                        gazebo_set.call(setmodelstate);
                        continue;
                    }
    		}
	    
                gazebo_msgs::SpawnModel model;
                model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
			std::string("<static>true</static>") +
                        std::string("<link name=\"ball\">") +
                        std::string("<inertial>") +
                        std::string("<mass value=\"1.0\" />") +
                        std::string("<origin xyz=\"0 0 0\" />") +
                        std::string("<inertia  ixx=\"1.0\" ixy=\"1.0\"  ixz=\"1.0\"  iyy=\"1.0\"  iyz=\"1.0\"  izz=\"1.0\" />") +
                        std::string("</inertial>") +
                        std::string("<visual>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</visual>") +
                        std::string("<collision>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</collision>") +
                        std::string("</link>") +
                        std::string("<gazebo reference=\"ball\">") +
                        std::string("<mu1>10</mu1>") +
                        std::string("<mu2>10</mu2>") +
                        std::string("<material>Gazebo/Blue</material>") +
                        std::string("<turnGravityOff>true</turnGravityOff>") +
                        std::string("</gazebo>") +
                        std::string("</robot>");

                std::ostringstream ball_name;
                ball_name << i;
                model.request.model_name = ball_name.str();
                model.request.reference_frame = "world";
                model.request.initial_pose.position.x = path_RRT[i].x;
                model.request.initial_pose.position.y = path_RRT[i].y;
                model.request.initial_pose.position.z = 0.7;
                model.request.initial_pose.orientation.w = 0.0;
                model.request.initial_pose.orientation.x = 0.0;
                model.request.initial_pose.orientation.y = 0.0;
                model.request.initial_pose.orientation.z = 0.0;
                gazebo_spawn.call(model);
                ros::spinOnce();
            }
            printf("Spawn path\n");
	
            //initialize robot position
            geometry_msgs::Pose model_pose;
            model_pose.position.x = waypoints[0].x;
            model_pose.position.y = waypoints[0].y;
            model_pose.position.z = 0.3;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 1.0;

            geometry_msgs::Twist model_twist;
            model_twist.linear.x = 0.0;
            model_twist.linear.y = 0.0;
            model_twist.linear.z = 0.0;
            model_twist.angular.x = 0.0;
            model_twist.angular.y = 0.0;
            model_twist.angular.z = 0.0;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = "racecar";
            modelstate.reference_frame = "world";
            modelstate.pose = model_pose;
            modelstate.twist = model_twist;

            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;

            gazebo_set.call(setmodelstate);
            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;
        } break;

        case RUNNING: {
            // PART IMPORTED [FROM PROJECT 2]

            // Step 1 : Update the steering angle with PID algorithm
            double turn = pid_ctrl.get_control(robot_pose, prevGoal, *currentGoal);
            double speed = pid_ctrl.set_speed(1.0, MAX_SPEED, robot_pose, *currentGoal, turn);
            std::cout << "Turn <" << turn << \
                " (" << robot_pose.x << "," << robot_pose.y << ")>>(" << (*currentGoal).x << "," << (*currentGoal).y << ") " \
                "with Speed " << speed << \
                std::endl;
            setcmdvel(speed, turn);

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

void generate_path_RRT()
{
    // PART IMPORTED [FROM PROJECT 2]

    // Iterate through all way point
    std::vector<point>::iterator it = waypoints.begin();
    point lastPoint = *it;

    for(it = waypoints.begin() + 1; it != waypoints.end(); it++)
    {
	// Create the tree for the current waypoint
        rrtTree t = rrtTree(lastPoint, *it, map, map_origin_x, map_origin_y, res, margin);
	
        // Generate the RRT Tree
        int isRRTValid = t.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
	
        if (isRRTValid) { std::cout << "[Err] RRT path is not valid." << std::endl; }
	
        // Get the backtracking path
        std::vector<traj> pathI = t.backtracking_traj();

	//DEBUG
       	//t.visualizeTree(pathI);
       	//std::cin.get();
        t.optimizeTree();

        // Add it to the total path
        path_RRT.insert(path_RRT.end(), pathI.begin(), pathI.end());

        // Update for the next loop
        lastPoint.x = path_RRT.back().x;
        lastPoint.y = path_RRT.back().y;
        lastPoint.th = path_RRT.back().th;
    }
}

void set_waypoints()
{
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;
    waypoint_candid[1].x = 2.0;
    waypoint_candid[1].y = 12.0;
    waypoint_candid[2].x = 3.5;
    waypoint_candid[2].y = -10.5;
    waypoint_candid[3].x = -2.0;
    waypoint_candid[3].y = -12.0;
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 10.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    model_states = msgs;
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"racecar") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}
