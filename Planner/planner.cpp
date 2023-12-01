#include "planner.h"
#include "../RRT/RRT.h"
#include "../params/json.hpp"
#include "../helper_funcs/helper.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <gmp.h>
#include <string>
#include <chrono>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay_t;
typedef Delaunay_t::Vertex_handle Face_handle;
typedef Delaunay_t::Finite_faces_iterator Finite_faces_iterator;
typedef Delaunay_t::Point delaunayPoint;

using json = nlohmann::json;
std::ifstream jsonPlanner("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/params/PathPlanner.json");
json paramsPlanner = json::parse(jsonPlanner);

PathPlanner::PathPlanner ( Node *start, std::vector<Point> local_map) :          
    targetConesShortDist(paramsPlanner["targetConesShortDist"].get<double>()),
    targetConesBigDist(paramsPlanner["targetConesBigDist"].get<double>()),
    maxEdgeLenght(paramsPlanner["maxEdgeLenght"].get<double>()),
    start(start->x, start->y, start->yaw)
{   
    this->local_map = local_map;  //get map from ROS
}

Node *PathPlanner::bestLeaf ( std::vector<Node*> leafNodes )
{
    std::vector<double> distances;
    std::vector<double> scores; // score for each leaf node.

    double dist;  // Distance between leaf node and cone.
    double dx, dy;
    double mean;
    double standardDeviation;

    for (Node*& leaf : leafNodes) {

        double total_dist = 0;  
        for (Point& cone : this->local_map) {

            dist = distance(&cone, leaf);
            total_dist += dist;
            distances.push_back(dist);
        }

        mean = total_dist / distances.size();

        standardDeviation = 0.0;

        for (double& d : distances) {
            standardDeviation += pow(d - mean, 2);
        }
        distances.clear();
        scores.push_back(standardDeviation);
    }

    std::vector<double>::iterator minIt = std::min_element(std::begin(scores), std::end(scores));
    int minIdx = std::distance(std::begin(scores), minIt);    

    return leafNodes[minIdx];
}

std::vector<Edge> PathPlanner::bestBranch ( Node *best_leaf ) 
{
    std::vector<Edge> best_branch;
    Node *node = best_leaf;

    while (node->parent != nullptr) {
        Point *pA = new Point(node->x, node->y);
        Point *pB = new Point(node->parent->x, node->parent->y);

        Edge edge(pA, pB);

        best_branch.push_back(edge);

        node = node->parent;
    }

    return best_branch;
}

std::vector<Edge> PathPlanner::delaunayEdges ( std::vector<Point> frontCones ) 
{    
    std::vector<Edge> edgesList;
    Delaunay_t dt;

    for (Point& obs : frontCones) {
        delaunayPoint p(obs.x, obs.y);
        dt.insert(p);
    }

    // Loop over finite faces
    for (Finite_faces_iterator it = dt.finite_faces_begin(); it != dt.finite_faces_end(); ++it) {
        for (int i = 0; i < 3; i++) {
            delaunayPoint pointA = it->vertex((i + 1) % 3)->point();
            delaunayPoint pointB = it->vertex((i + 2) % 3)->point();

            Point *pA = new Point(pointA);
            Point *pB = new Point(pointB);

            Edge edge(pA, pB);

            if (this->uniqueEdge(edge, edgesList) && edge.lenght() <= maxEdgeLenght) {
                edgesList.push_back(edge);
            }
        }
    }  

    return edgesList;
}

bool PathPlanner::uniqueEdge ( Edge edge, std::vector<Edge> delaunayEdges ) 
{
    for (Edge& del_edge : delaunayEdges) {
        Point *del_pointA = del_edge.a();
        Point *del_pointB = del_edge.b();
        Point *pointA = edge.a();
        Point *pointB = edge.b();

        if (pointA->x == del_pointA->x && pointA->y == del_pointA->y) {
            if (pointB->x == del_pointB->x && pointB->y == del_pointB->y) {
                return false;
            }
        } else if (pointA->x == del_pointB->x && pointA->y == del_pointB->y) {
            if (pointB->x == del_pointA->x && pointB->y == del_pointA->y) {
                return false;
            }
        }
    }

    return true;
}

std::vector<Point*> PathPlanner::getMidpoints(std::vector<Edge> delaunayEdges, std::vector<Edge> bestBranch) 
{
    std::vector<Point*> midpoints;

    for (Edge& del_edge : delaunayEdges) {
        for (Edge& edge : bestBranch) {
            if (checkEdgeIntersection(edge, del_edge)) {
                Point* mid = del_edge.midPoint();
                midpoints.push_back(mid);
                break;
            }
        }
    }

    sort(midpoints.begin(), midpoints.end(), [this](Point* p1, Point* p2) {
        return compare(p1, p2, start);
    });

    return midpoints;
}

bool PathPlanner::checkEdgeIntersection( Edge edge1, Edge edge2 ) 
{
    Point *p1 = this->left(edge1);
    Point *q1 = this->right(edge1);
    Point *p2 = this->left(edge2);
    Point *q2 = this->right(edge2);
 
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    if (o1 != o2 && o3 != o4) {
        return true;
    } else {
        return false;
    }
}

Point *PathPlanner::left ( Edge edge ) 
{
    if (edge.a()->x < edge.b()->x) {
        return edge.a();
    } else {
        return edge.b();
    }   
}

Point *PathPlanner::right ( Edge edge ) 
{
    if (edge.a()->x < edge.b()->x) {
        return edge.b();
    } else {
        return edge.a();
    }   
}

int PathPlanner::orientation ( Point *p, Point *q, Point *r ) 
{
    //https://www.geeksforgeeks.org/orientation-3-ordered-points/
    int val = (q->y - p->y) * (r->x - q->x) -
              (q->x - p->x) * (r->y - q->y);
 
    if (val == 0) return 0;  
    else if (val > 0) return 1;  
    else return 2;
}

bool PathPlanner::compare( Point *p1, Point *p2, const Node& r )
{
    double dx1 = pow(p1->x - r.x, 2);
    double dy1 = pow(p1->y - r.y, 2);
    double dx2 = pow(p2->x - r.x, 2);
    double dy2 = pow(p2->y - r.y, 2);

    double dist1 = sqrt(dx1 + dy1);
    double dist2 = sqrt(dx2 + dy2);

    if (dist1 < dist2) {
        return true;
    } else {
        return false;
    }
}

std::vector<Point*> PathPlanner::Planning () 
{   
    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<Point> rrtTargets;
    double dx, dy, coneDist;

    for (Point& cone : this->local_map) {
        coneDist = distance(&cone, &start);

        if (coneDist > targetConesShortDist && coneDist < targetConesBigDist) {
            rrtTargets.push_back(cone);
        }
    }
    
    // RRT
    RRT rrt(&start, this->local_map, rrtTargets);
    std::vector<Node*> leafNodes = rrt.createRRT();
    
    // Find the best branch
    Node *best_leaf = this->bestLeaf(leafNodes);

    std::vector<Edge> best_branch = this->bestBranch(best_leaf);

    // Delauany Triangulation
    std::vector<Edge> delaunayEdges;
    delaunayEdges = this->delaunayEdges(this->local_map);

    // Get the midpoints
    std::vector<Point*> midpoints = this->getMidpoints(delaunayEdges, best_branch);

    auto stopTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);

    std::cout << "Total time: " << elapsedTime.count() << "ms" << std::endl;
    
    //////  The rest code is only for plotting purposes  ///////

    std::string path1 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/bestLeaf.txt";
    std::ofstream outputFile1(path1);
    outputFile1 << best_leaf->x << " " << best_leaf->y << "\n";
    outputFile1.close();

    std::string path2 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/nodeList.txt";
    std::ofstream outputFile11(path2);

    for (auto p : rrt.nodeList) {
        if (p->parent == nullptr) {
            outputFile11 << p->x << " " << p->y << " " << 0 << " " << 0 << "\n";
        } else {
            outputFile11 << p->x << " " << p->y << " " << p->parent->x << " " << p->parent->y << "\n";
        }
    }
    outputFile11.close();

    // Random points List -> txt file
    std::string path3 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/rndList.txt";
    save_as_txt(rrt.rndList, path3);

    // Delaunay edges List -> txt file
    std::string path4 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/delaunayEdges.txt";
    std::ofstream outputFile4(path4);

    for (Edge& edge : delaunayEdges) {
        outputFile4 << edge.a()->x << " " << edge.a()->y << " ";
        outputFile4 << edge.b()->x << " " << edge.b()->y << " ";
        outputFile4 << "\n";
    }
    outputFile4.close();

    // Best branch edges List -> txt file
    std::string path5 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/bestBranch.txt";
    std::ofstream outputFile5(path5);

    for (Edge& edge : best_branch) {
        outputFile5 << edge.a()->x << " " << edge.a()->y << " ";
        outputFile5 << edge.b()->x << " " << edge.b()->y << " ";
        outputFile5 << "\n";
    }
    outputFile5.close();

    // Midpoints List -> txt file
    std::string path6 = "/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/midpoints.txt";
    std::ofstream outputFile6(path6);

    for (Point*& mid : midpoints) {
        outputFile6 << mid->x << " " << mid->y << " ";
        outputFile6 << "\n";
    }
    outputFile6.close();

    return midpoints;
}