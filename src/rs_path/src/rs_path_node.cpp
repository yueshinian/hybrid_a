#include<iostream>
#include<vector>
#include<cmath>
#include<cstring>

#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_datatypes.h>

class Node3d
{
public:
    double x_, y_, theta_;
    Node3d():x_(0), y_(0), theta_(0){}
    Node3d(double x, double y, double theta):x_(x), y_(y), theta_(theta){}
};

Node3d startNode, goalNode;

ros::Publisher path_pub_;
ros::Subscriber start_sub_, goal_sub_;

nav_msgs::Path path_ros;

class rs_path_param
{
public:
    double total_length;
    std::vector<double> seg_lengths;
    std::vector<char> seg_types;
};

double normalizeAngle(const double angle)
{
    double res_angle = std::fmod(angle + M_PI, 2*M_PI);
    if(res_angle < 0){
        res_angle += 2*M_PI;
    }
    return res_angle -= M_PI;
}

std::pair<double, double> cartesiab2polar(const double x, const double y)
{
    const double r = std::hypot(x, y);
    const double theta = std::atan2(y,x);
    return std::make_pair(r, theta);
}

void interpolation(const char type, const double ox, const double oy, const double phi, const double line_length, Node3d &pose, const double r)
{
    if(type == 'S'){
        pose.x_ = ox + line_length * std::cos(phi) * r;
        pose.y_ = oy + line_length * std::sin(phi) * r;
        pose.theta_ = phi;
    }else{
        const double dx = sin(line_length)*r;
        double dy = 0;
        double dtheta = 0;
        if(type == 'L'){
            dy = (1 - std::cos(line_length)) * r;
            dtheta = line_length;
        }else if(type == 'R'){
            dy = -(1 - std::cos(line_length)) * r;
            dtheta = -line_length;
        }
        pose.x_ = ox + dx * std::cos(phi) - dy * std::sin(phi);
        pose.y_ = oy + dx * std::sin(phi) + dy * std::cos(phi);
        pose.theta_ = phi + dtheta;
    }
}

std::vector<Node3d> LSL(const Node3d start_node, const Node3d goal_node)
{
    std::vector<Node3d> rs_path;
    const double dx = goal_node.x_ - start_node.x_;
    const double dy = goal_node.y_ - start_node.y_;
    const double r= 2.0;
    const double c = std::cos(start_node.theta_), s = std::sin(start_node.theta_);
    const double x = (dx * c + dy * s)/r, y = (-dx * s + dy * c)/r;
    const double phi = normalizeAngle(goal_node.theta_ - start_node.theta_);

    double t, u, v;
    std::tie(u, t) = cartesiab2polar(x - std::sin(phi), y + std::cos(phi) - 1);
    v = normalizeAngle(phi - t);
    std::cout<<"t, u, v: "<<t<<", "<<u<<", "<<v<<std::endl;
    rs_path_param rpp;
    rpp.total_length = std::abs(t) + std::abs(u) + std::abs(v);
    rpp.seg_lengths = {t, u, v};
    rpp.seg_types = {'L', 'S', 'L'};

    const double step = 0.5 / r;
    double total_length = rpp.total_length;
    const int points_nums = std::floor( total_length/step + rpp.seg_lengths.size());
    std::vector<Node3d> points(points_nums);
    if(t >= 0 && v >= 0){
        int index = 1;
        size_t size = rpp.seg_types.size();
        for(size_t i = 0; i< size; ++i){
            const char  type = rpp.seg_types[i];
            const double length = rpp.seg_lengths[i];
            double ox = points[index].x_, oy = points[index].y_, phi = points[index].theta_;
            --index;
            double d_l = step; 
            while(d_l < length){
                ++index;
                interpolation(type, ox, oy, phi, d_l, points[index], r);
                d_l += step;
            }
            interpolation(type, ox, oy, phi, length, points[++index], r);
        }

        const double epsillon = 1e-5;
        while(abs(points.back().x_) <epsillon && abs(points.back().y_) < epsillon && abs(points.back().theta_) < epsillon){
            points.pop_back();
        }
    }

    for(size_t i = 0; i < points.size(); ++i){
        Node3d node(start_node.x_ + points[i].x_* std::cos(start_node.theta_) - points[i].y_ * std::sin(start_node.theta_),
                                        start_node.y_ + points[i].x_ * std::sin(start_node.theta_) + points[i].y_ * std::cos(start_node.theta_),
                                        normalizeAngle(points[i].theta_ + start_node.theta_));
        rs_path.emplace_back(node);       
        printf("[%.2f, %.2f, %.2f].\r\n", node.x_, node.y_, node.theta_);                                 
    }
    return rs_path;
}

std::vector<Node3d> CSC(const Node3d start_node, const Node3d goal_node)
{
    return LSL(start_node, goal_node);
}

std::vector<Node3d> calRSPath(const Node3d start_node, const Node3d goal_node)
{
    std::vector<Node3d> rs_path;
    rs_path = CSC(start_node, goal_node);
    return rs_path;

}

void plan(const Node3d start_node, const Node3d goal_node)
{
    std::vector<Node3d> path;

    path = calRSPath(start_node, goal_node);

    path_ros.poses.clear();
    path_ros.header.frame_id = "/map";
    for(size_t i = 0; i < path.size(); ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose.position.x = path[i].x_;
        pose.pose.position.y = path[i].y_;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].theta_);
        path_ros.poses.emplace_back(pose);
    }
    path_pub_.publish(path_ros);
}

void subStartHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start)
{
        startNode.x_ = start->pose.pose.position.x;
        startNode.y_ = start->pose.pose.position.y;
        startNode.theta_ = tf::getYaw(start->pose.pose.orientation);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose = start->pose.pose;
        // start_pub_.publish(pose);
        ROS_INFO("receive start.[%.2f, %.2f, %.2f]", startNode.x_, startNode.y_, startNode.theta_);
   }

    void subGoalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
        goalNode.x_ = goal->pose.position.x;
        goalNode.y_ = goal->pose.position.y;
        goalNode.theta_ = tf::getYaw(goal->pose.orientation);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose = goal->pose;
        // goal_pub_.publish(pose);
        ROS_INFO("receive goal.[%.2f,%.2f,%.2f]",goalNode.x_, goalNode.y_, goalNode.theta_);
        plan(startNode, goalNode);
    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rs_path");
    ros::NodeHandle nh_;
    path_pub_= nh_.advertise<nav_msgs::Path>("/path", 10);
    start_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &subStartHandler); 
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &subGoalHandler);

    ros::spin();
    return 0;
}
