#ifndef RRT_H
#define RRT_H

#include "../helper_funcs/helper.h"
#include <iostream>
#include <vector>

class RRT {
    public:

    std::vector<Node*> nodeList; // otan den tha xreiastw pleon ta plots tha paei sta private

    std::vector<std::vector<double>> rndList;  //plot

    /**
     * @brief RRT constructor.
     * 
     * @param start the initial (x,y,yaw) of the car. Of type struct Node.
     * @param local_map vector with all cones/obstacles. Of type vector with struct Point.
     * @param rrtTargets vector with target cones which define the sampling area. Of type vector with struct Point.
     * 
    */
    RRT ( Node *start, std::vector<Point> local_map, std::vector<Point> rrtTargets );

    /**
     * @brief Create the Tree.
     * 
     * @return vector with leaf nodes. Contain pointers of type struct Node.
    */
    std::vector<Node*> createRRT ();

    private:

    double const planDist;
    double const cone_inner_size;
    double const cone_outer_size;
    double const turnAngle;
    double const turnAngleRad;
    double const expandDist;
    double const leaf_min_cost;  // minimum cost of a node to be consider as a leaf.
    int const iters;
    Node start;

    std::vector<Point> local_map;
    std::vector<Point> rrtTargets;  

    /**
     * @brief Generate a random point.
     *  
     * @return struct Point.
    */
    Point *random_point ();

    /**
     * @brief Generate a random point within a defined target area.
     *  
     * @return struct Point.
    */
    Point *random_point_from_target_list ();

    /**
     * @brief Generate a random number.
     *  
     * @param a lower number of the interval of type T.
     * @param b higher number of the interval of type T.
     * @param real True: generate float numbers - False: generate integer numbers. 
     * 
     * @return A number of type T.
    */
    template<typename T>
    T random_num_generator ( T a, T b , bool real );

    /**
     * @brief Nearest Neighbor Search.
     * 
     * @param query point to find the NN. Type struct Point.
     * 
     * @return pointer to the node of the nearest neighbor to the point.
    */
    Node *NNS ( Point *query );

    /**
     * @brief Check if a node is collision free.
     * 
     * @param point struct Node.
     * 
     * @return True: free - False: collision.
    */
    bool nodeCollisionFree ( Node *node );

    /**
     * @brief Check if an edge is collision free.
     * 
     * @param node1 of type struct Node.
     * @param node2 of type struct Node.
     * 
     * @return True: free - False: collision.
    */
    bool edgeCollisionFree ( Node *node1, Node *node2 );
    
    /**
     * @brief Check if a node is unique
     * 
     * @param node of type struct Node.
     * 
     * @return True: unique - False: node already exists.
    */
    bool uniqueNode ( Node *node );

    /**
     * @brief Create a new node to be added in RRT.
     *
     * @param rndm random point of type struct Point.
     * @param nn nearest neighbor node of type struct Node.
     * 
     * @return New node of type struct Node. 
    */
    Node *steering ( Point *rndm, Node **nn );

    /**
     * @brief Angle within the range [-π,π].
     * 
     * @param angle angle between new point and it's nearest.
     * 
     * @return Equivalent angle that falls within the range [-π, π].
    */
    double pi_2_pi ( double angle );

    /**
     * @brief Find the leaf nodes from node list.
     * 
     * @return a vector with all leaf nodes of type struct Node.
    */
    std::vector<Node*> findLeafNodes ();
};

#endif //RRT_H