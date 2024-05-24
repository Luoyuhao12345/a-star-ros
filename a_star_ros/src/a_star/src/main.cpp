#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "planner.h"

using namespace std;
bool map_flag = false;
bool start_flag = false;
bool goal_flag = false;
Planner a_star = Planner();
// 地图就是1m2一个栅格
float resolution = 1;

void mapCallback(const nav_msgs::OccupancyGrid::Ptr map)
{
    int height = map->info.height;
    int width = map->info.width;
    bool** binMap;
    binMap = new bool*[width];

    for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

    for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
        binMap[x][y] = map->data[y * width + x] ? true : false;
        }
    }
    a_star.set_map(binMap, width, height, resolution);
    map_flag = true;
}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr start)
{
    float x = start->pose.pose.position.x;
    float y = start->pose.pose.position.y;
    int x_index = x / resolution;
    int y_index = y / resolution;
    ROS_INFO("start x:%.2f, start y:%.2f", x, y);
    Node2D start_node = Node2D(x_index, y_index, x, y, 0, 0, nullptr);
    a_star.set_start(start_node);
    start_flag = true;
    if(map_flag&&start_flag&goal_flag) a_star.plan();
}

void goalCallback(const geometry_msgs::PoseStamped::Ptr goal)
{
    float x = goal->pose.position.x;
    float y = goal->pose.position.y;
    int x_index = x / resolution;
    int y_index = y / resolution;
    ROS_INFO("goal x:%.2f, goal y:%.2f", x, y);
    Node2D end_node = Node2D(x_index, y_index, x, y, 0, 0, nullptr);
    a_star.set_goal(end_node);
    goal_flag = true;
    if(map_flag&&start_flag&goal_flag) a_star.plan();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "a_star");
    ros::NodeHandle nh;
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 1);
    a_star.set_publisher(&pubPath);
    ros::Subscriber subMap = nh.subscribe("/map", 1, &mapCallback);
    ros::Subscriber subStart = nh.subscribe("/initialpose", 1, &startCallback);
    ros::Subscriber subGoal = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);
    ros::spin();
    return 0;
}