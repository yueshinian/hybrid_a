#include<memory>

#include<ros/ros.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<tf/transform_datatypes.h>

#include "path_planner/astar.h"

class path_planner
{
public:
    path_planner(ros::NodeHandle nh):nh_(nh)
    {
        init();
    }

    void run()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
        
    }   
private:
    void init()
    {
        astar_plan_ = std::make_shared<path_plan::astar>();
        path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10);
        start_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("start_pose", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 10);
        start_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &path_planner::subStartHandler, this); 
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &path_planner::subGoalHandler, this);
    }

    void subStartHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start)
    {
        startNode.x_ = start->pose.pose.position.x;
        startNode.y_ = start->pose.pose.position.y;
        startNode.theta_ = tf::getYaw(start->pose.pose.orientation);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose = start->pose.pose;
        start_pub_.publish(pose);
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
        goal_pub_.publish(pose);
        ROS_INFO("receive goal.[%.2f,%.2f,%.2f]",goalNode.x_, goalNode.y_, goalNode.theta_);
        plan();
    }

    void plan()
    {
        bool is_success = astar_plan_->findPath(startNode, goalNode);
        if(!is_success){
            ROS_INFO("path plan failed.");
        }
        std::vector<path_plan::Node3d> path = astar_plan_->getPath();
        nav_msgs::Path path_ros;
        path_ros.header.frame_id = "/map";
        for(size_t i = 0; i < path.size(); ++i)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.pose.position.x = path[i].x_;
            pose.pose.position.y = path[i].y_;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].theta_);
            path_ros.poses.emplace_back(pose);
            // printf("point %d, [%.2f, %.2f, %.2f].\r\n", i, path[i].x_, path[i].y_, path[i].theta_);
        }
        path_pub_.publish(path_ros);
        ROS_INFO("path end.");
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_, start_pub_, goal_pub_ ;
    ros::Subscriber start_sub_, goal_sub_;
    std::shared_ptr<path_plan::astar> astar_plan_;
    path_plan::Node3d startNode, goalNode;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    std::shared_ptr<path_planner> path_planner_ = std::make_shared<path_planner>(nh);
    path_planner_->run();
    return 0;
}

