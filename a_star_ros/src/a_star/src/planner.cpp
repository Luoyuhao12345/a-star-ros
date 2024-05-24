#include "planner.h"

using namespace std;
void Planner::plan()
{
    ROS_INFO("Planning...");
    open_list = {};
    close_list = {};
    open_list.push_back(&start);
    bool reach_goal = false;
    while(1)
    {
        int open_list_num = open_list.size();
        int index = find_min_f();
        if(index==-1) break;
        Node2D *father_node = new Node2D(
            open_list[index]->x_index,
            open_list[index]->y_index,
            open_list[index]->x,
            open_list[index]->y,
            open_list[index]->g,
            open_list[index]->h,
            open_list[index]
        );
        add_close_list(father_node);
        remove_open_list(index);
        find_neighbor(father_node);
        if(is_reach_goal())
        {
            reach_goal = true;
            break;
        }
        if(open_list_num==0)
        {
            reach_goal = false;
            break;
        }
    }
    if(reach_goal)
    {
        cout<<"path has be found"<<endl;
        publish_path(open_list);
    }
    else cout<<"there is no path to be found"<<endl;
    delete_all_nodes();
}

void Planner::set_map(bool** binMap, int height, int width, float cell_size)
{
    this->binMap = binMap;
    this->height = height;
    this->width = width;
    this->cell_size = cell_size;
    for(int i = 0; i<height; i++)
    {
        for(int j = 0; j<width; j++)
        {
            if(binMap[i][j] == true)
                obs.push_back(Point(i,j));
        }
    }
}

void Planner::set_start(Node2D start)
{
    this->start = start;
}

void Planner::set_goal(Node2D goal)
{
    this->goal = goal;
}

void Planner::add_close_list(Node2D *node)
{
    close_list.push_back(node);
}

void Planner::remove_open_list(int index)
{
    auto it = open_list.begin() + index;
    open_list.erase(it);
}

int Planner::find_min_f()
{
    int index = -1;
    int min_f = 9999;
    int open_list_num = open_list.size();
    for(int i =0; i<open_list_num; i++)
    {
        if(min_f > open_list[i]->f)
        {
            min_f = open_list[i]->f;
            index = i;
        }
    }
    return index;
}

void Planner::find_neighbor(Node2D *node)
{
    for(int i=-1; i<2; i++)
    {
        for(int j=-1; j<2; j++)
        {
            if(i==0 &&  j==0) continue;
            float dis = cell_size;
            float x = node->x + i*dis;
            float y = node->y + j*dis;
            int x_index = x/cell_size;
            int y_index = y/cell_size;
            if(!is_add_open_list(x_index, y_index)) continue;
            add_open_list(node, x_index, y_index, x, y);
            if(is_reach_goal()) break;
        }
    }
}

void Planner::add_open_list(Node2D *node, int x_i, int y_i, float x, float y)
{
    float g = 0;
    float delta_x = abs(node->x-x);
    float delta_y = abs(node->y-y);
    g += sqrt(delta_x*delta_x+delta_y*delta_y);
    float h = abs(goal.x-x)+abs(goal.y-y);
    float f = g+h;
    if(is_in_open_list(x_i, y_i))
    {
        // 要更新的点
        int open_list_num = open_list.size();
        int cur_index = 0;
        for(int i=0; i<open_list_num; i++)
        {
            if(x_i==open_list[i]->x_index &&y_i==open_list[i]->y_index)
            {
                cur_index = i;
                break;
            }
        }
        // 判断是否更新父节点
        if(f<open_list[cur_index]->f)
        {
            open_list[cur_index]->g = g;
            open_list[cur_index]->f = f;
            open_list[cur_index]->pred = node;
        }
    }
    else  //直接加入open_list
    {
        Node2D *new_node = new Node2D() ;
        new_node->x = x;
        new_node->y = y;
        new_node->x_index = x_i;
        new_node->y_index = y_i;
        new_node->pred = node;
        new_node->g = g;
        new_node->h = h;
        new_node->f = f;
        open_list.push_back(new_node);
    }
}

bool Planner::is_in_open_list(int x, int y)
{
    int n = open_list.size();
    for(int i = 0; i<n; i++)
    {
        int a = open_list[i]->x_index;
        int b = open_list[i]->y_index;
        if(x==a && y==b) return true;
    }
    return false;
}

bool Planner::is_add_open_list(int x, int y)
{
    bool a = is_exceed_index(x, y);
    bool b = is_in_close_list(x, y);
    bool c = is_obstacle(x, y);
    bool result = (!a)&&(!b)&&(!c);
    return result;
}

bool Planner::is_exceed_index(int x, int y)
{
    if(x<0 || x>width || y<0 || y>height) return true;
    else return false;
}

bool Planner::is_in_close_list(int x, int y)
{
    int close_list_num = close_list.size();
    for(int i = 0; i<close_list_num; i++)
    {
        int a = close_list[i]->x_index;
        int b = close_list[i]->y_index ;
        if(x == a && y == b) return true;
    }
    return false;
}

bool Planner::is_obstacle(int x, int y)
{
    int n = obs.size();
    for(int i = 0; i<n; i++)
    {
        int checked_x = obs[i].x;
        int checked_y = obs[i].y ;
        if(x == checked_x && y == checked_y) return true;
    }
    return false;
}

bool Planner::is_reach_goal()
{
    int open_list_num = open_list.size();
    for(int i = 0; i<open_list_num; i++)
    {
        int a = open_list[i]->x_index;
        int b = open_list[i]->y_index;
        if(a == goal.x_index && b == goal.y_index) return true;
    }
    return false;
}

void Planner::set_publisher(ros::Publisher* pub)
{
    path_pub = pub;
}

void Planner::publish_path(vector<Node2D*> open_list)
{
    int n = open_list.size();
    int k = 0;
    for(int i = 0; i<n; i++)
    {
        if(open_list[i]->x_index == goal.x_index && open_list[i]->y_index == goal.y_index)
        {
            k = i;
            break;
        }
    }
    Node2D *node = open_list[k];
    nav_msgs::Path path;
    path.header.frame_id="/map";
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = node->x;
    pose.pose.position.y = node->y;
    path.poses.push_back(pose);
    while(node->pred != nullptr)
    {
        Node2D *father_node = node->pred;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id="/map";
        pose.pose.position.x = father_node->x;
        pose.pose.position.y = father_node->y;
        path.poses.push_back(pose);
        node = node->pred;
    }
    path_pub->publish(path);
}

void Planner::delete_all_nodes()
{
    int open_list_num = open_list.size();
    for(int i = 0; i<open_list_num; i++)
    {
        delete open_list[i];
    }
    int close_list_num = close_list.size();
    for(int i = 0; i<close_list_num; i++)
    {
        delete close_list[i];
    }
    cout<<"delete all nodes"<<endl;
}
