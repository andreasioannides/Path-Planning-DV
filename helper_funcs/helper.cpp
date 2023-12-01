#include "helper.h"
#include "../params/json.hpp"
#include <fstream>
#include <string>
#include <vector>

Point::Point () : x(), y() {}

Point::Point ( double coordX, double coordY ) : x(coordX), y(coordY) {}

Point::Point ( delaunayPoint &point ) : x(point.x()), y(point.y()) {}

Edge::Edge () : pointA(), pointB() {}

Edge::Edge ( Point *pointA, Point *pointB ) : pointA(pointA), pointB(pointB) {}

Point *Edge::a ()
 {
    return pointA;
}

Point *Edge::b () 
{
    return pointB;
}

Point *Edge::midPoint () 
{
    Point *mid = new Point();

    mid->x = (pointA->x + pointB->x) / 2;
    mid->y = (pointA->y + pointB->y) / 2;

    return mid;
}

double Edge::lenght ()
{
    double lenghtSq;
    lenghtSq = pow(pointA->x - pointB->x, 2) + pow(pointA->y - pointB->y, 2);

    return sqrt(lenghtSq);
}

Node::Node () : x(), y(), yaw(), cost(), parent(), childs() {}

Node::Node ( double coordX, double coordY, double Yaw ) : x(coordX), y(coordY), yaw(Yaw), cost(0), parent(nullptr), childs(0) {}

Node::Node ( double coordX, double coordY, double Yaw, int Cost, Node *Parent ) : x(coordX), y(coordY), yaw(Yaw), cost(Cost), parent(Parent), childs(0) {}

double distance ( Point *a, Node *b ) 
{
    double sum = pow(a->x - b->x, 2) + pow(a->y - b->y, 2);

    return sqrt(sum);
}



void save_as_txt( std::vector<std::vector<double>> list, std::string path ) 
{
    std::ofstream outputFile(path);

    for (auto point : list) {
        for (auto p : point) {
            outputFile << p << " ";
        }
        outputFile << "\n";
    }
    outputFile.close();
}

std::vector<Point> txt_to_vector ( std::string path ) 
{
    std::ifstream input(path);
    
    std::vector<Point> track;
    double x, y;

    while (input >> x >> y) {
        // x = x*10;
        // y = y*10;
        Point cone(x, y);
        track.push_back(cone);
    }

    return track;
} 