#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "../helper_funcs/helper.h"
#include "../RRT/RRT.h"

class PathPlanner {
    public:

    /**
     * @brief Planner constructor.
     * 
     * @param start the initial (x,y,yaw) of the car. Of type struct Node.
     * @param local_map vector with all cones/obstacles. Of type vector with struct Point.
     * 
    */
    PathPlanner ( Node *start, std::vector<Point> local_map );  

    /**
     * @brief Create the path.
     * 
     * @return vector the points of the path.
    */
    std::vector<Point*> Planning();  

    private:

    // const double backConesDist; // Distance behind the car to exclude front cones which are too close to the car.
    const double targetConesShortDist; // Short distance from the car which defines the sampling area.
    const double targetConesBigDist; // Big distance from the car which defines the sampling area.
    const double maxEdgeLenght;  
    Node start;  // car position
    std::vector<Point> local_map;
    
    /**
     * @brief Get the edges of delaunay triangulation.
     * 
     * @param frontCones vector with all cones in front of the car.
     * 
     * @return vector of type Edge.
    */
    std::vector<Edge> delaunayEdges ( std::vector<Point> frontCones );

    /**
     * @brief Check if an edge is unique.
     * 
     * @param edge of type class Edge.
     * @param delaunayEdges vector with all edges created with Delaunay triangulation.
     * 
     * @return True: unique - False: already exists.
    */
    bool uniqueEdge ( Edge edge, std::vector<Edge> delaunayEdges );

    /**
     * @brief Get the edges of delaunay triangulation.
     * 
     * @param frontCones vector with all cones in front of the car.
     * 
     * @return vector of type Edge.
    */
    Node *bestLeaf ( std::vector<Node*> leafNodes );

    /**
     * @brief Get the edges of delaunay triangulation.
     * 
     * @param frontCones vector with all cones in front of the car.
     * 
     * @return vector of type Edge.
    */
    std::vector<Edge> bestBranch ( Node *best_leaf );

    /**
     * @brief Get the midpoints where the car will pass through.
     * 
     * @param delaunayEdges edges created with Delaunay triangulation. Of type struct Edge.
     * @param bestBranch best branch created with RRT. Of type struct Edge.
     * 
     * @return vector with all unique midpoints of type struct Point.
    */
    std::vector<Point*> getMidpoints(std::vector<Edge> delaunayEdges, std::vector<Edge> bestBranch);

    /**
     * @brief Compare the distance of two points from a reference point(start). Helper function for sorting middle points.
     * 
     * @param point1
     * @param point2
     * @param start reference point
     * 
     * @return True if the euclidean distance of point1 is greater than the distance of point2.
    */
    bool compare( Point *point1, Point *point2, const Node& r );

    /**
     * @brief Check if two edges intersect.
     * 
     * @param edge1 of type struct Edge.
     * @param edge2 of type struct Edge.
     * 
     * @return True: intersect, False: intersecte.
    */
    bool checkEdgeIntersection ( Edge edge1, Edge edge2 );

    /**
     * @brief Get the left point of an edge.
     * 
     * @param edge of type class Edge.
     * 
     * @return return a pointer to the left point of the edge.
    */
    Point *left ( Edge edge );

    /**
     * @brief Get the right point of an edge.
     * 
     * @param edge of type class Edge.
     * 
     * @return return a pointer to the right point of the edge.
    */
    Point *right ( Edge edge );

    /**
     * @brief Find the orientation of 3 points.
     * 
     * @param p of type struct Point.
     * @param q of type struct Point.
     * @param r of type struct Point.
     * 
     * @return 0: collinear, 1: clockwise, 2: counterclockwise
    */
    int orientation ( Point *p, Point *q, Point *r );
};
                                 
#endif 