#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <random>
#include <fstream>
#include <string>
#include <chrono>
#include "RRT/RRT.h"
#include "helper_funcs/helper.h"
#include "Planner/planner.h"
#include "params/json.hpp"

std::vector<Point> getPatch(std::vector<Point> obstacleList, Node *start );

std::vector<Point> getFrontCones ( std::vector<Point> obstacleList, double dist, double backConesDist, Node *start );

double *headingVector ( Node *start );

int main() { 
    std::vector<Point> obstacleList;
    obstacleList = {
        {1, -3},
        {1, 3},
        {3, -3},
        {3, 3},
        {6, -3},
        {6, 3},
        {9, -2.8},
        {9, 3.2},
        {12.5, -2.5},
        {12, 4},
        {16, -2},
        {15, 5},
        {20, -1},
        {18.5, 6.5},
        {24, 1},
        {22, 8}
    };

    // Track -> vector (load the whole track)
    // std::string path0 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/tracks/track1.txt";
    // std::vector<Point> obstacleList = txt_to_vector(path0);

    // local map -> txt file
    std::string path1 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/obstacleList.txt";
    std::ofstream track1(path1);

    for (Point& obstacle : obstacleList) {
        track1 << obstacle.x << " ";
        track1 << obstacle.y << " ";
        track1 << "\n";
    }
    track1.close();

    Node start(0, 0, 0);

    std::vector<Point> patch = getPatch(obstacleList, &start);
    
    // auto startTime = std::chrono::high_resolution_clock::now();

    PathPlanner planner(&start, patch);
    
    std::vector<Point*> pathPoints = planner.Planning();
    
    return 0;
}

std::vector<Point> getPatch(std::vector<Point> obstacleList, Node *start ) {
    using json = nlohmann::json;
    std::ifstream jsonPlanner("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/params/PathPlanner.json");
    json paramsPlanner = json::parse(jsonPlanner);

    const double targetConesBigDist = paramsPlanner["targetConesBigDist"].get<double>();
    const double backConesDist = paramsPlanner["backConesDist"].get<double>();

    jsonPlanner.close();

    std::vector<Point> patch = getFrontCones(obstacleList, targetConesBigDist, backConesDist, start);

    return patch;
}

std::vector<Point> getFrontCones ( std::vector<Point> obstacleList, double dist, double backConesDist, Node *start ) {
    std::vector<Point> frontConesList;
    double *heading = headingVector(start);
    double headingOrth[2] = {-heading[1], heading[0]};  // Orthogonal (Perpendicular) heading vector.

    double carPosBehindCone[2] = {start->x - backConesDist*heading[0],
                                  start->y - backConesDist*heading[1]};

    double dx, dy;

    for (Point obs : obstacleList) {
        if (headingOrth[0]*(obs.y - carPosBehindCone[1]) - headingOrth[1]*(obs.x - carPosBehindCone[0]) < 0) {
            dx = pow(obs.x - start->x, 2);
            dy = pow(obs.y - start->y, 2);

            if (sqrt(dx + dy) <= dist) {
                frontConesList.push_back(obs);
            }
        }
    }
    delete[] heading;
    
    return frontConesList;
}

double *headingVector ( Node *start ) {
    double carRotMatrix[2][2] = {
        {std::cos(start->yaw), -std::sin(start->yaw)},
        {std::sin(start->yaw), std::cos(start->yaw)}
                                }; 

    double *heading = new double[2];
    heading[0] = 1;
    heading[1] = 0;

    heading[0] = carRotMatrix[0][0]*heading[0] + carRotMatrix[0][1]*heading[1];
    heading[1] = carRotMatrix[1][0]*heading[0] + carRotMatrix[1][1]*heading[1];

    return heading;
}