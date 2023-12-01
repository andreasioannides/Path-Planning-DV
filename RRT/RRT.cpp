#include "RRT.h"
#include "../helper_funcs/helper.h"
#include "../params/json.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <cmath>

using json = nlohmann::json;
std::ifstream jsonRRT("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/params/RRT.json");
json paramsRRT = json::parse(jsonRRT);

RRT::RRT ( Node *start,  std::vector<Point> local_map, std::vector<Point> rrtTargets ) : 
    turnAngle(paramsRRT["turnAngle"].get<double>()), 
    planDist(paramsRRT["planDist"].get<double>()), 
    cone_inner_size(paramsRRT["cone_inner_size"].get<double>()), 
    cone_outer_size(paramsRRT["cone_outer_size"].get<double>()),
    turnAngleRad(turnAngle * M_PI/180),
    expandDist(paramsRRT["expandDist"].get<double>()),
    leaf_min_cost(paramsRRT["leaf_min_cost"].get<double>()),
    iters(paramsRRT["iters"].get<int>()),
    start(start->x, start->y, start->yaw)
{   
    this->local_map = local_map;
    this->rrtTargets = rrtTargets;
    this->nodeList = nodeList;
}

Point *RRT::random_point () 
{
    double randX = this->random_num_generator<double>(0, planDist, true);
    double randY = this->random_num_generator<double>(-planDist, planDist, true);
    
    //Initial yaw of the car = root_node.point[2]
    double car_rot_matrix[2][2] = {
        {std::cos(start.yaw), -std::sin(start.yaw)},
        {std::sin(start.yaw), std::cos(start.yaw)}
                                  }; 

    Point *rndm = new Point();
    rndm->x = car_rot_matrix[0][0]*randX + car_rot_matrix[0][1]*randY;
    rndm->y = car_rot_matrix[1][0]*randX + car_rot_matrix[1][1]*randY;
    rndm->x = rndm->x + start.x;
    rndm->y = rndm->y + start.y;

    return rndm;
}

Point *RRT::random_point_from_target_list () 
{
    if (this->rrtTargets.empty()) {
        return this->random_point();
    }
    
    int ntargets = this->rrtTargets.size();      //Number of target cones
    int targetIdx = this->random_num_generator<int>(0, ntargets-1, false);

    double randAngle = this->random_num_generator<double>(0, 2*M_PI, true);
    double randDist = this->random_num_generator<double>(cone_inner_size, cone_outer_size, true);
   
    Point *rndm = new Point();
    rndm->x = this->rrtTargets[targetIdx].x + randDist * std::cos(randAngle);
    rndm->y = this->rrtTargets[targetIdx].y + randDist * std::sin(randAngle);

    return rndm;
}

template<typename T>
T RRT::random_num_generator ( T a, T b, bool real ) 
{
    std::random_device rd;
    std::mt19937 generator(rd());

    if (real) {
        std::uniform_real_distribution<double> distribution(a, b);
        double rand = distribution(generator);
        return rand;
    } else {
        std::uniform_int_distribution<int> distribution(a, b);
        int rand = distribution(generator);
        return rand;
    }
}

Node *RRT::NNS ( Point *query ) 
{
    // initialize an "empty" Node
    Node *nearest_node = new Node(0, 0, 0);

    double best_dist = __INT_MAX__;
    double dist;

    for (Node*& node : this->nodeList) {
        dist = distance(query, node);

        if (dist < best_dist) {
            best_dist = dist;
            *(&nearest_node) = node;
        }
    }

    return nearest_node;
}

bool RRT::nodeCollisionFree ( Node *node ) 
{
    double dist;

    for (Point obs : this->local_map) {
        dist = distance(&obs, node);

        if (dist <= cone_inner_size) return false;
    }

    return true;
}

bool RRT::edgeCollisionFree ( Node *node1, Node *node2 ) 
{
    // Distance between cone and edge: d=|Ax+By+C|/sqrt(lamda^2+1)
    double lamda = (node2->x - node1->x) / (node2->y - node1->y);
    double nmt;  //numerator
    double den;  //denominator

    for (Point obs : local_map) {
        nmt = abs(-lamda*obs.y + obs.x - node1->x + lamda*node1->y);
        den = sqrt(pow(lamda, 2) + 1);

        if (nmt/den <= cone_inner_size) return false;
    }
        
    return true;
}

bool RRT::uniqueNode ( Node *node ) 
{
    for (Node*& n : this->nodeList) {
        if (n->x == node->x && n->y == node->y) {
            return false;
        } 
    }

    return true;      
}

double RRT::pi_2_pi ( double angle ) 
{
    // Modulo operator % can't be used for non-integer
    // values such as : (angle + M_PI) % (2*M_PI) - M_PI 
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

Node *RRT::steering ( Point *rndm, Node **nn ) 
{
    double theta = std::atan2(rndm->y - (*nn)->y, rndm->x - (*nn)->x);
    double angleChange = this->pi_2_pi(theta - (*nn)->yaw);
    
    if (angleChange > turnAngleRad) {
        angleChange = turnAngleRad;
    } else if (angleChange >= -turnAngleRad) {
        angleChange = 0;    
    } else {
        angleChange = -turnAngleRad;
    }
    
    Node *new_node = new Node();

    new_node->yaw = (*nn)->yaw + angleChange;
    new_node->x = (*nn)->x + expandDist * std::cos(new_node->yaw);
    new_node->y = (*nn)->y + expandDist * std::sin(new_node->yaw);
    new_node->cost = (*nn)->cost + expandDist;
    new_node->parent = (*nn);
    new_node->childs = 0;
    (*nn)->childs += 1;

    return new_node;
}

std::vector<Node*> RRT::findLeafNodes () 
{
    std::vector<Node*> leafNodes;

    for (Node*& node : this->nodeList) {
        if (node->childs == 0 && node->cost >= leaf_min_cost) {
            leafNodes.push_back(node);
        }
    }

    return leafNodes;
}

std::vector<Node*> RRT::createRRT () {
    std::ifstream jsonPlanner("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/params/PathPlanner.json");
    json paramsPlanner = json::parse(jsonPlanner);
    double targetConesShortDist = paramsPlanner["targetConesShortDist"].get<double>();
    jsonPlanner.close();

    Point *rndm;   
    
    this->nodeList.push_back(&start);

    double dx, dy;
    
    for (int i=0; i<iters; ++i) {
        rndm = this->random_point_from_target_list();

        if (distance(rndm, &start) <= 10) {
            continue;
        }

        Node *nn = this->NNS(rndm);

        if (nn->cost >= planDist) { // den mou aresei ws kritirio
            delete rndm;
            continue;
        }

        Node *new_node = this->steering(rndm, &nn);
        
        if (this->nodeCollisionFree(new_node) && this->uniqueNode(new_node)) {
            this->nodeList.push_back(new_node);

            this->rndList.push_back({rndm->x, rndm->y});   //List of random points. This vector is created only for plot purposes.
        }
        delete rndm;
    }

    std::vector<Node*> leafNodes = this->findLeafNodes();

    // for (Node*& leaf : leafNodes) {
    //     std::cout << leaf->x << "," << leaf->y << std::endl;
    // }

    return leafNodes;
}