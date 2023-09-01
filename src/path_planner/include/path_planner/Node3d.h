#pragma once

#include <iostream>

namespace path_plan{
    
enum class NodeState
{
    NO_EXTEND = 0,
    IS_OPEN,
    IS_CLOSED
};

class Node3d
{
public:
    double x_, y_, theta_;
    int index_x_, index_y;
    double g_, h_;
    NodeState node_state_;//-1,0,1
    int prim_;
    Node3d *father_;
    Node3d():x_(0), y_(0), theta_(0), index_x_(0), index_y(0), g_(0), h_(0), node_state_(NodeState::NO_EXTEND), prim_(1), father_(nullptr){}
    Node3d(double x, double y, double theta):x_(x), y_(y), theta_(theta){}
    Node3d(double x, double y, double theta, int prim):x_(x), y_(y), theta_(theta), prim_(prim){}
    double GetX() const { return x_; }
    double GetY() const { return y_; }
    double GetPhi() const { return theta_; }
};

using Node3dPtr = Node3d*;
}