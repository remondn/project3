#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#include <algorithm>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
    goalBias = 0;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
    goalBias = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    goalBias = 0;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
   // cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	for(int j = 0; j < 10; j++) {
	    double alpha = path[i].alpha;
	    double d = path[i].d;
	    double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	    double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	    double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    //TODO

    // First, create a new node containing all data
    node *newNode = new node;
    newNode->idx = count;
    newNode->idx_parent = idx_near;
    newNode->alpha = alpha;
    newNode->d = d;
    newNode->location = x_new;
    newNode->rand = x_rand;

    // Then link it to the table of nodes of the RRT Tree
    ptrTable[count] = newNode;
    count++;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO

    // Initialisation was done at construction of the object

    // Loop at least K times and until the tree reach the goal
    int loop = 0;
    int goalReached = false;

    // Use this one to know how much we crashed so far ( new State is not valid)
    int crashes = 0;

    mapSize = fmin(x_max - x_min, y_max - y_min);

    // DEBUG
    //visualizeTree();
    //std::cout << "Pause : First print of the tree with only root" << std::endl;
    //std::cin.get();

    //while(loop < K || !goalReached)
    //while(loop < K && !goalReached)
    while(loop < K && (!goalReached || loop < K))
    {
        // DEBUG
        if(loop % 500 == 0 && loop != 0)
        {
            // Print it only sometimes
            std::cout << "generateRRT : loop = " << loop << " & crashes = " << crashes << std::endl;
        }
        if(crashes % K == 0 && crashes != 0)
        {
            // Print it only sometimes
            std::cout << "generateRRT : loop = " << loop << " & crashes = " << crashes << std::endl;
        }

        // If we crash too much, we might be too close of a wall. We should restart the entire path
        if(crashes > K * 10)
        {
            return -1;
        }

        // First step : Generate a random point
        point randPoint = randomState(x_max, x_min, y_max, y_min);

        // DEBUG
        //std::cout << "generateRRT >> Random state generated" << std::endl;

        // Get the nearest neighbor
        int nearIndex = nearestNeighbor2(randPoint, MaxStep);

        // DEBUG
        //std::cout << "generateRRT >> Nearest neighbor got" << std::endl;

        // Create a new state
        double out[5];
        if(newState(out, ptrTable[nearIndex]->location, randPoint, MaxStep) == 0)
        {
            // The new state is not valid, we collided with something !
            // So forget about this state, and start a new one, without
            // incrementing loop, because we want K vertex
            // DEBUG
            //std::cout << "generateRRT >> New state is not valid : continue" << std::endl;
            crashes++;
            continue;
        }

        // DEBUG
        //std::cout << "Index nearest :" << nearIndex << std::endl;

        // The new state is valid ! Create a vertex
        point *newPoint = new point();
        newPoint->x = out[0];
        newPoint->y = out[1];
        newPoint->th = out[2];
        addVertex(*newPoint, randPoint, nearIndex, out[3], out[4]);

        //DEBUG
        //visualize(*newPoint);

        // DEBUG
        //std::cout << "generateRRT >> Vertex added" << std::endl;

        // Check if we reached the goal
        if(!goalReached)
        {
            for(int i = 0; i < count; i++)
            {
                // If the goal is 10cm away from one of the point of the tree,
                // it's fine, we accept
                if(distance(ptrTable[i]->location, x_goal) < 0.1)
                {
                    // DEBUG
                    //std::cout << "generateRRT >> Goal reached" << std::endl;
                    goalReached = true;
                }
            }
        }

        loop++;

        // DEBUG
        //visualizeTree();
        //std::cout << "Pause" << std::endl;
        //std::cin.get();
    }

    return goalReached;
}

int rrtTree::setParentOfVertex(int idx, int idx_parent) {

    if (idx < 1 || idx > count - 1) { return 0; }

    ptrTable[idx]->idx_parent = idx_parent;

    return 1;
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    //TODO

    point randomPoint;

    goalBias++;     // For goal Bias, to count how often we have to set it

    // Every 10 randomly generated points, we set the goal Bias
    if(goalBias % BIAS_FREQ == 0)
    {
        return x_goal;
    }

    do
    {
        // Generate random coordinate within the range asked
        float randX = x_min + (static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(x_max-x_min))));
        float randY = y_min + (static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(y_max-y_min))));

        // Add it to a point
        randomPoint.x = randX;
        randomPoint.y = randY;
        randomPoint.th = 0.0;    // shouldn't be used, but we initialise it still.

        //DEBUG
        //std::cout << "infinite loop ?" << std::endl;
    }while(isCollision(randomPoint, randomPoint, 0.0, 1.0));
    // Keep generating until we got a valid point
    // We don't care of alpha and d since start and endpoint are the same

    return randomPoint;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO

    // TODO : Compute the function f(MaxStep, L, MaxAlpha)
    float thetaMax = max_alpha;

    // Then, go through all the points, minimising the distance
    // and respecting the rule of thetaMax
    node *nearest = root;
    float bestDist = distance(root->location, x_rand);

    float angle;

    // Find the min distance with all the nodes of the tree
    for(int i = 0; i < count; i++)
    {
        //DEBUG
        //std::cout << "Going through the tree !" << std::endl;
        if(distance(ptrTable[i]->location, x_rand) < bestDist)
        {
            // New minimum distance, but save it only if the angle is respecting
            // the condition for thetaMax
            angle = getDirection(ptrTable[i]->location, x_rand) - ptrTable[i]->alpha;

            //DEBUG
            //std::cout << "Found better nearest than root..." << std::endl;
            if(angle <= thetaMax && angle >= -thetaMax)
            {
                //DEBUG
                //std::cout << "... YES !" << std::endl;
                nearest = ptrTable[i];
                bestDist = distance(ptrTable[i]->location, x_rand);
            }
        }
    }

    // Return the index of the nearest point
    return nearest->idx;
}

int rrtTree::nearestNeighbor2(point x_rand, double MaxStep) {
    //TODO

    int idx = nearestNeighbor(x_rand);
    float angle = getDirection(ptrTable[idx]->location, x_rand) - ptrTable[idx]->alpha;
    float step = MaxStep;

    while((angle > max_alpha && angle < -max_alpha) && step > 0.0)
    {
        idx = ptrTable[idx]->idx_parent;
        angle = getDirection(ptrTable[idx]->location, x_rand) - ptrTable[idx]->alpha;
        step -= 1.0;
    }

    if(step > 0.0)
    {
        return idx;
    }
    return 0;
}

int rrtTree::nearestNeighbor(point x_rand) {
    //TODO

    // Initialisation with the root of the tree
    node *nearest = root;
    float bestDist = distance(root->location, x_rand);

    // Find the min distance with all the nodes of the tree
    for(int i = 0; i < count; i++)
    {
        if(distance(ptrTable[i]->location, x_rand) < bestDist)
        {
            // New minimum, save it
            nearest = ptrTable[i];
            bestDist = distance(ptrTable[i]->location, x_rand);
        }
    }

    // Return the index of the nearest point
    return nearest->idx;
}

int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO

    // Struct to store the new states randomly generated
    std::vector<point*> randNewPoints;

    // Struct to store the control generated with the point
    std::vector<control*> randNewControls;

    float xc, yc;

    // Generate, let's say, 20 new states
    for(int i = 0; i < 100; i++)
    {
        // Generate parameters
        float alpha = -max_alpha + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(max_alpha+max_alpha)));
        float d = 0.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(MaxStep-0.5)));

        // Compute new point with these parameters
        float R = L / tan(alpha);
        float beta = d / R;
        xc = x_near.x - R * sin(x_near.th);
        yc = x_near.y + R * cos(x_near.th);

        point *xNew = new point();
        control *cNew = new control();

        xNew->th = x_near.th + beta;
        xNew->x = xc + R * sin(xNew->th);
        xNew->y = yc - R * cos(xNew->th);

        cNew->alpha = alpha;
        cNew->d = d;

        // Add it to our list of randomly generated point/control
        randNewPoints.push_back(xNew);
        randNewControls.push_back(cNew);

        xNew = NULL;
        cNew = NULL;
    }

    // We have generated them. Now choose the closest
    std::vector<point*>::iterator itP = randNewPoints.begin();
    std::vector<control*>::iterator itC = randNewControls.begin();

    point bestPoint = **itP;
    control bestControl = **itC;
    float bestDistance = distance(x_rand, bestPoint);

    for(itP = randNewPoints.begin(); itP != randNewPoints.end(); itP++)
    {
        if(distance(x_rand, **itP) < bestDistance)
        {
            bestPoint = **itP;
            bestControl = **itC;
            bestDistance = distance(x_rand, bestPoint);
        }
        itC++;
    }

    // Now we have the closest point among the 20 randomly generated.
    // So set the output.
    out[0] = bestPoint.x;
    out[1] = bestPoint.y;
    out[2] = bestPoint.th;
    out[3] = bestControl.alpha;
    out[4] = bestControl.d;

    // DEBUG
    //std::cout << "newState >> Before test of collision for the point generated" << std::endl;
    //return 1;

    // Return of the function : if it collides or not
    if(isCollision(x_near, bestPoint, bestControl.d, bestControl.alpha))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

bool rrtTree::isCollision(point x1, point x2, double d, double alpha) {
    //TODO

    // DEBUG
    int i, j;

    // This function can also be used to check if a point is valid
    if(x1.x == x2.x && x1.y == x2.y)
    {
        i = ((int)floor(x1.x / res)) + map_origin_x;
        j = ((int)floor(x1.y / res)) + map_origin_y;
        //return (map.at<uchar>(i, j) == 0);
        return (map.at<uchar>(i, j) != 255);
        //return (map.at<uchar>(i, j) != 250);
    }

    double x = x1.x, y = x1.y, dx = 0, dy = 0;

    dx = (x2.x - x) / 100;
    dy = (x2.y - y) / 100;

    for(int k=0; k<100; ++k) {
        x += dx;
        y += dy;
        int i2 = ((int)floor(x / res)) + map_origin_x;
        int j2 = ((int)floor(y / res)) + map_origin_y;

        //if (map.at<uchar>(i2, j2) == 0)
        if (map.at<uchar>(i2, j2) != 255)
        //if (map.at<uchar>(i2, j2) != 250)
        {
            //DEBUG
            //std::cout << "Collision !" << std::endl;
            //std::cout << "Collision = " << (int)map.at<uchar>(i2, j2) << "(" << i2 << ";" << j2 << ")" << std::endl;
            return true;
        }
    }
    //return false;


    int i2, j2;
    double Res = 2;

    double constPreventingWall = mapSize / 6.5;
    double x4X = x1.x + constPreventingWall*cos(x1.th), x4Y = x1.y + constPreventingWall*sin(x1.th);
    x = x1.x, y = x1.y, dx = 0, dy = 0;
    dx = (x4X - x1.x) / 100;
    dy = (x4Y - x1.y) / 100;

    for(int k=0; k<100; ++k) {
        x += dx;
        y += dy;
        int i2 = ((int)floor(x / res)) + map_origin_x;
        int j2 = ((int)floor(y / res)) + map_origin_y;

        if (map.at<uchar>(i2, j2) == 0) { return true; }
    }

    double x3X = x2.x + constPreventingWall*cos(x2.th), x3Y = x2.y + constPreventingWall*sin(x2.th);
    x = x2.x, y = x2.y, dx = 0, dy = 0;
    dx = (x3X - x2.x) / 100;
    dy = (x3Y - x2.y) / 100;

    for(int k=0; k<100; ++k) {
        x += dx;
        y += dy;
        int i2 = ((int)floor(x / res)) + map_origin_x;
        int j2 = ((int)floor(y / res)) + map_origin_y;

        if (map.at<uchar>(i2, j2) == 0) { return true; }
    }

    int freq = 10;
	for(int j = 0; j < freq; j++) {
        // Compute the points of the round line, like in the visualization
	    double p1_th = x1.th + d*j/freq*tan(alpha)/L;
        double p2_th = x1.th + d*(j+1)/freq*tan(alpha)/L;
        double p1_x = x1.x + L/tan(alpha)*(sin(p1_th) - sin(x1.th));
	    double p1_y = x1.y + L/tan(alpha)*(cos(x1.th) - cos(p1_th));
        double p2_x = x1.x + L/tan(alpha)*(sin(p2_th) - sin(x1.th));
        double p2_y = x1.y + L/tan(alpha)*(cos(x1.th) - cos(p2_th));
        i = ((int)floor(p1_x / res)) + map_origin_x;
        j = ((int)floor(p1_y / res)) + map_origin_y;
        i2 = ((int)floor(p2_x / res)) + map_origin_x;
        j2 = ((int)floor(p2_y / res)) + map_origin_y;
        //x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
        //x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));

        // Check if the map is occupied or not
        //if(map.at<uchar>(i, j) == 0 || map.at<uchar>(i2, j2) == 0)
        if(map.at<uchar>(i, j) != 255 || map.at<uchar>(i2, j2) != 255)
        //if(map.at<uchar>(i, j) != 250 || map.at<uchar>(i2, j2) != 250)
        {
            return true;
        }
    }
    return false;
}

int rrtTree::optimizeTree() {
    // Remove close nodes.
    int idx = count - 1;
    while(idx != NULL)
    {
        int idx_parent = ptrTable[idx]->idx_parent;
        int idx_parent_of_parent = ptrTable[idx_parent]->idx_parent;
        double dist = distance(ptrTable[idx]->location, ptrTable[idx_parent]->location);
        if (dist < 1 && idx_parent_of_parent != NULL) {
            setParentOfVertex(idx, idx_parent_of_parent);
        }
        idx = idx_parent;
    }

    return 1;
}

std::vector<traj> rrtTree::backtracking_traj(){
    //TODO

    // First, find the nearest node to the goal
    float bestDistance = distance(ptrTable[0]->location, x_goal);
    int bestI = 0;
    for(int i = 0; i < count; i++)
    {
        if(distance(ptrTable[i]->location, x_goal) < bestDistance)
        {
            bestDistance = distance(ptrTable[i]->location, x_goal);
            bestI = i;
        }
    }

    // Then, track parents one by one and add them to the traj
    std::vector<traj> path;
    node currentNode = *(ptrTable[bestI]);
    traj *trajPoint = NULL;
    while(currentNode.idx_parent != NULL)
    {
        // Create equivalent 'traj' of the current node
        trajPoint = new traj();
        trajPoint->x = currentNode.location.x;
        trajPoint->y = currentNode.location.y;
        trajPoint->th = currentNode.location.th;
        trajPoint->d = currentNode.d;
        trajPoint->alpha = currentNode.alpha;

        // Add it to our vector
        path.push_back(*trajPoint);

        trajPoint = NULL;

        // Go to the parent
        currentNode = *(ptrTable[currentNode.idx_parent]);
    }

    // Add the root
    trajPoint = new traj();
    trajPoint->x = currentNode.location.x;
    trajPoint->y = currentNode.location.y;
    trajPoint->th = currentNode.location.th;
    trajPoint->d = currentNode.d;
    trajPoint->alpha = currentNode.alpha;
    path.push_back(*trajPoint);
    trajPoint = NULL;

    trajPoint = new traj();
    trajPoint->x = ptrTable[0]->location.x;
    trajPoint->y = ptrTable[0]->location.y;
    trajPoint->th = ptrTable[0]->location.th;
    trajPoint->d = ptrTable[0]->d;
    trajPoint->alpha = ptrTable[0]->alpha;
    path.push_back(*trajPoint);

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<traj> rrtTree::optimizePath(std::vector<traj> path)
{
    /* To optimize the path, we will try to get rid of useless nodes.
    It is the case for example when the car can go straight (no obstacles)
    but the path generated is turning around a bit.
    */
    // New vector of traj
    std::vector<traj> optimized;

    //DEBUG
    // std::cout << "path = {";
    // std::vector<traj>::iterator disp;
    // for(disp = path.begin(); disp != path.end(); disp++)
    // {
    //     std::cout << "(" << disp->x << ";" << disp->y << ";" << disp->th << ")   ";
    // }
    // std::cout << "}" << std::endl;

    // Check every point 
    std::vector<traj>::iterator currentPoint;
    std::vector<traj>::iterator previousPoint = path.begin();
    std::vector<traj>::iterator pointToCompare;
    std::vector<traj>::iterator nextOptimized;
    for(currentPoint = path.begin(); currentPoint != path.end(); currentPoint++)
    {
        // Add it to our vector
        optimized.push_back(*currentPoint);
        nextOptimized = currentPoint;

        // Optimize only if it's not the last point of the path
        if(currentPoint != path.end() - 1)
        {
            // For every point, compare it with all the next point on the path
            for(pointToCompare = currentPoint; pointToCompare != path.end(); pointToCompare++)
            {
                // If there is no collision between the points
                if(!isCollision2(*currentPoint, *pointToCompare))
                {
                    double dif = currentPoint->th - pointToCompare->th;
                    double oldDif = currentPoint->th - previousPoint->th;
                    if(dif < (PI/20) && dif > -(PI/20) && oldDif < (PI/10) && oldDif > -(PI/10))
                    {
                        nextOptimized = pointToCompare;
                    }
                }
            }
        }       
        
        // If we can optimize, optimize
        if(nextOptimized != currentPoint)
        {
            currentPoint = nextOptimized;
            currentPoint--;
        }
        previousPoint = currentPoint;

        //DEBUG
        // std::cout << "path optimized = {";
        // for(disp = optimized.begin(); disp != optimized.end(); disp++)
        // {
        //     std::cout << "(" << disp->x << ";" << disp->y << ";" << disp->th << ")   ";
        // }
        // std::cout << "}" << std::endl;

        // std::cin.get();
    }
    return optimized;
}

bool rrtTree::isCollision2(traj t1, traj t2) {
    // Convert to point
    point x1, x2;
    x1.x = t1.x;
    x1.y = t1.y;
    x1.th = t1.th;
    x2.x = t2.x;
    x2.y = t2.y;
    x2.th = t2.th;

    // DEBUG
    int i, j;

    // If it's the same point, return true
    if(x1.x == x2.x && x1.y == x2.y)
    {
        return true;
    }

    double x = x1.x, y = x1.y, dx = 0, dy = 0;

    dx = (x2.x - x) / 100;
    dy = (x2.y - y) / 100;

    for(int k=0; k<100; ++k) {
        x += dx;
        y += dy;
        int i2 = ((int)floor(x / res)) + map_origin_x;
        int j2 = ((int)floor(y / res)) + map_origin_y;

        //if (map.at<uchar>(i2, j2) == 0)
        if (map.at<uchar>(i2, j2) != 255)
        //if (map.at<uchar>(i2, j2) != 250)
        {
            //DEBUG
            //std::cout << "Collision !" << std::endl;
            //std::cout << "Collision = " << (int)map.at<uchar>(i2, j2) << "(" << i2 << ";" << j2 << ")" << std::endl;
            return true;
        }
    }
    //return false;


    int i2, j2;
    double Res = 2;

    double constPreventingWall = mapSize / 6.5;
    double x4X = x1.x + constPreventingWall*cos(x1.th), x4Y = x1.y + constPreventingWall*sin(x1.th);
    x = x1.x, y = x1.y, dx = 0, dy = 0;
    dx = (x4X - x1.x) / 100;
    dy = (x4Y - x1.y) / 100;

    for(int k=0; k<100; ++k) {
        x += dx;
        y += dy;
        int i2 = ((int)floor(x / res)) + map_origin_x;
        int j2 = ((int)floor(y / res)) + map_origin_y;

        if (map.at<uchar>(i2, j2) == 0) { return true; }
    }

    double x3X = x2.x + constPreventingWall*cos(x2.th), x3Y = x2.y + constPreventingWall*sin(x2.th);
    x = x2.x, y = x2.y, dx = 0, dy = 0;
    dx = (x3X - x2.x) / 100;
    dy = (x3Y - x2.y) / 100;

    for(int k=0; k<100; ++k) {
        x += dx;
        y += dy;
        int i2 = ((int)floor(x / res)) + map_origin_x;
        int j2 = ((int)floor(y / res)) + map_origin_y;

        if (map.at<uchar>(i2, j2) == 0) { return true; }
    }

    return false;
}