#ifndef _ASTAR_H_
#define _ASTAR_H_
#include <iostream>
#include <vector>
#include<queue>
#include<map>
#include <nav_msgs/OccupancyGrid.h>

#include "path_planner/reeds_shepp_path.h"
#include "path_planner/Node3d.h"

namespace path_plan
{

class NodeComparator
{
public:
    bool operator()(Node3dPtr node1, Node3dPtr node2){
        return node1->g_ + node1->h_  > node2->g_ + node2->h_;
    }
};

class astar
{
public:
    astar();
    ~astar();
    bool findPath(Node3d start, Node3d end);
    std::vector<Node3d> getPath() const {return path_; }
    void reset();
    void reversePath(Node3dPtr node);
    void setMap(nav_msgs::OccupancyGrid map) { map_ = map; }
    void setResolution(const double resolution) { resolution_ = resolution; }
    void setOrigin(const double origin_x, const double origin_y){ origin_x_ = origin_x, origin_y_ = origin_y; }
 private:
    std::pair<int, int> pose2index(const double x, const double y){ return std::make_pair((x-origin_x_)/resolution_, (y-origin_y_)/resolution_); }
    int index2id(const int x, const int y) { return x*map_max_x + map_max_y; }
    int pose2id(Node3dPtr node);
    bool isInGrid(const int index_x, const int index_y);
    bool isOccupied(Node3dPtr node);
    double getHeuristic(Node3dPtr node1, Node3dPtr node2);
    double getGroundTruth(Node3dPtr node1, Node3dPtr node2);
    std::vector<Node3d> getNeighbourNodes(Node3dPtr node);
    double normalizeAngle(double angle);
    void setSampleParam();
 private:   
    double resolution_;
    double origin_x_, origin_y_;
    int map_max_x, map_max_y;
    int node_max_nums_;
    const int dir_ = 3;
    const double radius = 6;
    const double d_angle = 6.75*M_PI/180.0;
    const int headings = 72;
    const double deltaHeadingRad = 2 * M_PI / headings;
    double penaltyTurning, penaltyReversiong, penaltyCOD;
    std::vector<double> dx, dy, dt;
    std::vector<Node3d> path_;
    nav_msgs::OccupancyGrid map_;
    std::priority_queue<Node3dPtr, std::vector<Node3dPtr>, NodeComparator> open_set_;
    std::vector<Node3dPtr> node3d_pool_;
    std::unique_ptr<ReedShepp> reed_shepp_generator_;
};

};

#endif