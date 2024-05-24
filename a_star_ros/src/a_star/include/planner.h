#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include "node_2d.h"
#include <vector>
#include <nav_msgs/Path.h>

using namespace std;

struct Point {
    int x;
    int y;

    Point():x(0), y(0){};
    Point(int a, int b): x(a), y(b) {};
};

class Planner
{
	public:
		void plan();
		void set_map(bool** binMap, int height, int width, float cell_size);
		void set_goal(Node2D goal);
		void set_start(Node2D start);
		void set_publisher(ros::Publisher* pub);
	private:
		bool** binMap;
		vector<Point> obs;
		int height;
		int width;
		float cell_size;
		Node2D start;
		Node2D goal;
		vector<Node2D*> open_list;
		vector<Node2D*> close_list;
		ros::Publisher* path_pub;
	private:
		bool is_add_open_list(int x, int y);
        bool is_exceed_index(int x, int y);
        bool is_in_close_list(int x, int y);
        bool is_in_open_list(int x, int y);
        bool is_obstacle(int x, int y);
        bool is_reach_goal();
        void add_close_list(Node2D *node);
        void find_neighbor(Node2D *node);
        void add_open_list(Node2D *node, int x_i, int y_i, float x, float y);
        void remove_open_list(int index);
        int find_min_f();
		void publish_path(vector<Node2D*> open_list);
		void delete_all_nodes();
};


#endif