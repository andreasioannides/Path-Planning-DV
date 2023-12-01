#ifndef HELPER_H
#define HELPER_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <fstream>
#include <string>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay_t;
typedef Delaunay_t::Point delaunayPoint;

struct Point {
    double x;  
    double y;

    /**
     * @brief Default Point constructor.
    */
    Point ();

    /**
     * @brief Point constructor. Initialize "x" and "y".
     * 
     * @param coordX x coordinate of point.
     * @param coordY y coordinate of point.
    */
    Point ( double coordX, double coordY );

    /**
     * @brief Point constructor which copies the constructor of Delaunay::Point. Used in triangulation (triangulation.cpp).
     * 
     * @param point struct Point defined in CGAL Delaunay library.
    */
    Point ( delaunayPoint &point );
};

class Edge {
    private:

    Point *pointA;
    Point *pointB;
    
    public:

    /**
     * @brief Default constructor of Edge.
    */
    Edge ();

    /**
     * @brief Constructor of Edge. Initialize pointA and pointB.
     * @param pointA of type struct Point. Pointer to the first point of the edge.
     * @param pointB of type struct Point. Pointer to the second point of the edge.
    */
    Edge ( Point *pointA, Point *pointB );

    /**
     * @brief Get the point A.
     * 
     * @return pointer of struct Point.
    */
    Point *a();

    /**
     * @brief Get the point B.
     * 
     * @return pointer of struct Point.
    */
    Point *b();

    /**
     * @brief Get the middle point of the edge.
     * 
     * @return pointer of struct Point.
    */
    Point *midPoint ();

    /**
     * @brief Get the lenght of the edge.
     * 
     * @return the magnitude of a vector(edge).
    */
    double lenght ();   
};

struct Node {
        Node *parent;
        double x;
        double y;
        double yaw;
        double cost;
        unsigned childs; // number of it's childs
        
        /**
        * @brief Default constructor of Node. 
        */
        Node();

        /**
        * @brief Constructor of Node. Initializes x,y,yaw.
        * 
        * @param coordX coordinate of Node.
        * @param coordY coordinate of Node.
        * @param Yaw of Node. 
        */
        Node ( double coordX, double coordY, double Yaw );

        /**
        * @brief Constructor of Node. Initializes x,y,yaw,cost,parent.
        * 
        * @param coordX coordinate of Node.
        * @param coordY coordinate of Node.
        * @param Yaw of Node. 
        * @param Cost of Node. 
        * @param Parent of Node. 
        */
        Node ( double coordX, double coordY, double Yaw, int Cost, Node *Parent );
};

/**
 * @brief Euclidean distance.
 * 
 * @param a of type Point.
 * @param b of type Node.
 * 
 * @returns Euclidean distance between two points in space.
*/
double distance ( Point *a, Node *b );


/**
 * @brief  Save a vector as a txt file. Used for plot with python matplotlib.
*/
void save_as_txt ( std::vector<std::vector<double>> list, std::string path );

/**
 * @brief  Read the content of a txt file and create a vector. Used to read the track .
*/
std::vector<Point> txt_to_vector ( std::string path );

#endif