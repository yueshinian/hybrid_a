#include "path_planner/astar.h"
#include "path_planner/reeds_shepp_path.h"

namespace path_plan
{

astar::astar():map_max_x(1000), map_max_y(1000), resolution_(0.1), origin_x_(-50), origin_y_(-50)
{
    node_max_nums_ = map_max_x * map_max_y * headings;
    node3d_pool_.resize(node_max_nums_);
    for(int i = 0; i < node_max_nums_; ++i){
        node3d_pool_[i] = new Node3d();
    }

    setSampleParam();

    penaltyTurning = 1.05;
    penaltyReversiong = 2.0;
    penaltyCOD = 2.0;

    reed_shepp_generator_ = std::make_unique<ReedShepp>();
    printf("astar init finished.\r\n");
}

astar::~astar()
{
    for(int i = 0; i < node_max_nums_; ++i){
        delete node3d_pool_[i];
        node3d_pool_[i] = nullptr;
    }
}

double astar::getGroundTruth(Node3dPtr node1, Node3dPtr node2)
{
    double g = std::hypot(node1->x_ - node2->x_, node1->y_ - node2->y_);
    if(node2->prim_ < dir_){
        if(node1->prim_ != node2->prim_){
            if(node1->prim_ < dir_){
                g = dx[0] * penaltyTurning;
            }else{
                g = dx[0] * penaltyTurning * penaltyCOD;
            }
        }else{
            g = dx[0];
        }
    }else{
        if(node1->prim_ != node2->prim_){
            if(node1->prim_ < dir_){
                g = dx[0] * penaltyTurning * penaltyCOD * penaltyReversiong;
            }else{
                g = dx[0] * penaltyTurning * penaltyReversiong;
            }
        }else{
            g = dx[0]*penaltyReversiong;
        }
    }
    return g;
}

double astar::getHeuristic(Node3dPtr node1, Node3dPtr node2)
{
    double h = std::hypot(node1->x_ - node2->x_, node1->y_ - node2->y_);
    return h;
}

std::vector<Node3d> astar::getNeighbourNodes(Node3dPtr node)
{
    std::vector<Node3d> neighbourNodes;
    for(int i = 0; i < dir_; ++i){
        double x  = node->x_ + dx[i]*std::cos(node->theta_) - dy[i]*std::sin(node->theta_);
        double y = node->y_ + dx[i]*std::sin(node->theta_) + dy[i]*std::cos(node->theta_);
        double theta = node->theta_ + dt[i];
        neighbourNodes.emplace_back(Node3d(x, y, theta,i));
    }

    for(int i = 0; i < dir_; ++i){
        double x  = node->x_ - (dx[i]*std::cos(node->theta_) + dy[i]*std::sin(node->theta_));
        double y = node->y_ - (dx[i]*std::sin(node->theta_) - dy[i]*std::cos(node->theta_));
        double theta = node->theta_ - dt[i];
        neighbourNodes.emplace_back(Node3d(x, y, theta,i+dir_)); 
    }

    return neighbourNodes;
}

bool astar::isInGrid(const int index_x, const int index_y)
{
    return index_x >= 0 && index_y >= 0 && index_x < map_max_x && index_y < map_max_y;
}

bool astar::isOccupied(Node3dPtr node)
{
    return false;
}

double astar::normalizeAngle(double angle)
{
    double res_angle = angle;
    if(res_angle > M_PI){
        res_angle -= 2*M_PI;
    }else if(res_angle <= -M_PI){
        res_angle += 2*M_PI;
    }
    return res_angle;
}

int astar::pose2id(Node3dPtr node)
{
    double angle = node->theta_;
    if(angle < 0){
        angle += 2*M_PI;
    }
    return static_cast<int>(angle/deltaHeadingRad) * map_max_x * map_max_y 
        + static_cast<int>((node->x_ - origin_x_)/resolution_) * map_max_y + static_cast<int>((node->y_ - origin_y_)/resolution_);
}

void astar::reset()
{
    for(int i = 0; i < node_max_nums_; ++i){
        Node3dPtr node_ptr = node3d_pool_[i];
        node_ptr->node_state_ = NodeState::NO_EXTEND;
        node_ptr->father_ = nullptr;
    }

    path_.clear();

    std::priority_queue<Node3dPtr, std::vector<Node3dPtr>, NodeComparator>().swap(open_set_);
}

void astar::reversePath(Node3dPtr node)
{
    Node3dPtr node_ptr = node;
    path_.clear();
    while(node_ptr != nullptr){
        Node3d node_3d = *node_ptr;
        path_.emplace_back(node_3d);
        node_ptr = node_ptr->father_;
    }
    std::reverse(path_.begin(), path_.end());
}

void astar::setSampleParam()
{
    double s = radius * d_angle;
    double temp_dx = s*sin(M_PI_2 - d_angle/2), temp_dy = s*cos(M_PI_2 - d_angle/2);
    dt = {0, d_angle, -d_angle};
    dx = {s, temp_dx, temp_dx};
    dy = {0, temp_dy, -temp_dy};
    for(int i = 0; i < dir_; ++i){
        printf("sample %.6f, %d, [%.6f, %.6f, %.6f].\r\n", d_angle, i, dx[i], dy[i], dt[i]);
    }
}

bool astar::findPath(Node3d start, Node3d end)
{
    reset();

    int index_x, index_y;
    std::tie(index_x, index_y) = pose2index(start.x_, start.y_);
    // int id = index2id(index_x, index_y);
    int id = pose2id(&start);
    Node3dPtr start_node = node3d_pool_[id];
    start_node->x_ = start.x_, start_node->y_ = start.y_, start_node->theta_ = start.theta_;
    start_node->father_ = nullptr;
    start_node->g_ = 0;
    start_node->node_state_ = NodeState::IS_OPEN;
    start_node->index_x_ = index_x, start_node->index_y = index_y;
    start_node->prim_ = 0;
    printf("start node3d:[%d,%d], [%.2f. %.2f, %.2f].\r\n", 
        index_x, index_y, start_node->x_, start_node->y_, start_node->theta_);
    std::tie(index_x, index_y) = pose2index(end.x_, end.y_);
    // id = index2id(index_x, index_y);
    id = pose2id(&end);
    Node3dPtr end_node = node3d_pool_[id];
    end_node->x_ = end.x_, end_node->y_ = end.y_, end_node->theta_ = end.theta_;
    end_node->father_ = nullptr;
    // end_node->g_ = 0;
    // end_node->node_state_ = NodeState::IS_CLOSED;
    end_node->index_x_ = index_x, end_node->index_y = index_y;
    printf("end node3d:[%d.%d], [%.2f,%.2f,%.2f].\r\n", 
        index_x, index_y, end_node->x_, end_node->y_, end_node->theta_);

    start_node->h_ = getHeuristic(start_node, end_node);

    open_set_.emplace(start_node);
    int plan_time_count = 0;
    while(!open_set_.empty())
    {
        if(plan_time_count++ %1000 == 0)
            printf("plan count %d.\r\n", plan_time_count);
        Node3dPtr cur_node = open_set_.top();
        cur_node->node_state_ = NodeState::IS_CLOSED;
        open_set_.pop();
        if(abs(cur_node->x_ - end.x_) < resolution_ 
            && abs(cur_node->y_ - end.y_) < resolution_
            && abs(normalizeAngle(cur_node->theta_ - end.theta_)) < 0.17)
        {
            printf("near end.[%.6f, %.6f, %.6f], [%.6f, %.6f, %.6f]\r\n", 
                cur_node->x_, cur_node->y_, cur_node->theta_, end.x_, end.y_, end.theta_);
            reversePath(cur_node);
            break;
        }

        if(plan_time_count %100 == 0){
        std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
        std::make_shared<ReedSheppPath>();
        if (!reed_shepp_generator_->ShortestRSP(
            std::make_shared<Node3d>(*cur_node), std::make_shared<Node3d>(end), reeds_shepp_to_check)) {
            std::cout << "ShortestRSP failed"<<std::endl;
        }else{
            std::cout<<"ShortestRSP succees"<<std::endl;
            size_t size = reeds_shepp_to_check->x.size();
            reversePath(cur_node);
            for(size_t i= 0; i < size; ++i){
                // std::cout<<"["<<reeds_shepp_to_check->x[i]<<", "<<reeds_shepp_to_check->y[i]<<", "<<reeds_shepp_to_check->phi[i]<<"]"<<std::endl;
                path_.emplace_back(Node3d(reeds_shepp_to_check->x[i], reeds_shepp_to_check->y[i], reeds_shepp_to_check->phi[i]));
            }
            break;
        }
        }

        std::vector<Node3d> neighbourNodes = getNeighbourNodes(cur_node);
        // printf("neighbour nodes size is:%d.\r\n", neighbourNodes.size());
        for(auto &node:neighbourNodes){
            int index_x, index_y, id;
            std::tie(index_x, index_y) = pose2index(node.x_, node.y_);
            // id = index2id(index_x, index_y);
            id = pose2id(&node);
            Node3dPtr temp_node_ptr = &node;
            if(!isInGrid(index_x, index_y) || isOccupied(temp_node_ptr) || id >= node_max_nums_  || id == pose2id(cur_node)){
                // printf("this node is not valid:[%d,%d, %d].\r\n", index_x, index_y, id);
                continue;
            }
            Node3dPtr temp_node = node3d_pool_[id];
            // if(plan_time_count%100 == 1){
            //     printf("cur_node:[%.6f,%.6f,%.6f], neighbour:[%.6f,%.6f,%.6f].\r\n", 
            //         cur_node->x_, cur_node->y_, cur_node->theta_, 
            //         node.x_, node.y_, node.theta_);
            // }

            if(temp_node->node_state_ == NodeState::NO_EXTEND){
                temp_node->x_ = node.x_;
                temp_node->y_ = node.y_;
                temp_node->theta_ = node.theta_;
                temp_node->prim_ = node.prim_;
                temp_node->node_state_ = NodeState::IS_OPEN;
                temp_node->index_x_ = index_x;
                temp_node->index_y = index_y;
                temp_node->g_ = cur_node->g_ + getGroundTruth(cur_node, temp_node);
                temp_node->h_ = getHeuristic(temp_node, end_node);
                temp_node->father_ = cur_node;
                open_set_.emplace(temp_node);
            }else if(temp_node->node_state_ == NodeState::IS_OPEN){
                Node3dPtr node_ptr = &node;
                node_ptr->g_ = cur_node->g_ + getGroundTruth(cur_node, node_ptr);
                node_ptr->h_ = getHeuristic(node_ptr, end_node);
                if(node_ptr->g_ + node_ptr->h_ < temp_node->g_ + temp_node->h_){
                    temp_node->x_ = node.x_;
                    temp_node->y_ = node.y_;
                    temp_node->theta_ = node.theta_;
                    temp_node->prim_ = node.prim_;
                    temp_node->node_state_ = NodeState::IS_OPEN;
                    temp_node->index_x_ = index_x;
                    temp_node->index_y = index_y;
                    temp_node->g_ = cur_node->g_ + getGroundTruth(cur_node, temp_node);
                    temp_node->h_ = getHeuristic(temp_node, end_node);
                    temp_node->father_ = cur_node;
                }
            }
        }
    }
    // path_.emplace_back(end);
    return !path_.empty();
}

}
