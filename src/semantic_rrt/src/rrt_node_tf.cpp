/******************************************************************************
Copyright (c) 2020 Georgia Instititue of Technology 
Copyright (c) 2020 Tsinghua University
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Author: Jianming TONG (jtong45@gatech.edu)
*******************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"
#include <math.h>
#include <opencv2/opencv.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <random>
#include <actionlib/client/simple_action_client.h>
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include <mutex>

// #define SEND_PYTHON_ACTIONLIB
// #define SEND_PYTHON_MOVEBASE
#define SEND_CPP_ACTIONLIB
#define THRESHOLD_TRANSFORM 0.5
#define IF_DETECT_END
std::mutex _mt;

// -----------------------------------  functions
double mapValue(const nav_msgs::OccupancyGrid & mapDataIn, const geometry_msgs::PointStamped & point){
    _mt.lock();
    double tempResult;
    int idx;
    idx = (floor((point.point.y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((point.point.x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
    tempResult = mapDataIn.data[idx];
    _mt.unlock();
    return tempResult;
}//给定的点映射到地图中的索引位置

double norm2Dpoints(const double& point1_x, const double& point1_y, const geometry_msgs::PointStamped & point2)
{
    _mt.lock();  //mt.lock()被用来保护计算部分的代码，确保多个线程不会同时访问同一个变量（如tempResult）。在计算完成后，_mt.unlock()被用来释放锁
    double tempResult;
    tempResult= pow(	(pow((point2.point.x-point1_x),2)+pow((point2.point.y-point1_y),2))	,0.5);
    _mt.unlock();
    return tempResult;
}//计算两个二维点之间的欧几里德距离

double norm2Dpoints_2(const double& point1_x, const double& point1_y, const geometry_msgs::Point & point2)
{
    _mt.lock();  //mt.lock()被用来保护计算部分的代码，确保多个线程不会同时访问同一个变量（如tempResult）。在计算完成后，_mt.unlock()被用来释放锁
    double tempResult;
    tempResult= pow(	(pow((point2.x-point1_x),2)+pow((point2.y-point1_y),2))	,0.5);
    _mt.unlock();
    return tempResult;
}//计算两个二维点之间的欧几里德距离


double informationGain(const nav_msgs::OccupancyGrid& mapDataIn, const geometry_msgs::PointStamped & point, const double r){
    _mt.lock();
    double infoGainValue = 0, tempResult;
    int index, init_index, last_index, length, start, end, r_region, limit;
    double x = point.point.x, y = point.point.y;

    index = (floor((y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
                  
    r_region = int(r/mapDataIn.info.resolution);
    init_index = index-r_region*(mapDataIn.info.width+1);
    last_index = index+r_region*(mapDataIn.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if(last_index < mapDataIn.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            
            int deltaIdx_y = abs(r_region - i);
            int deltaIdx_x = floor(std::sqrt(r_region*r_region - deltaIdx_y*deltaIdx_y));
            int temp = r_region-deltaIdx_x;
            int startIdxCircle = start + temp;
            int endIdxCircle   = end   - temp;
            
            for(int j = startIdxCircle; j < endIdxCircle; j++){
                switch(mapDataIn.data[j]){
                    case  -1: {infoGainValue++; break;}//占据网格地图中未知区域的表示方法（-1 表示未知区域，100表示自由区域）
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){

            int deltaIdx_y = abs(r_region - i);
            int deltaIdx_x = floor(std::sqrt(r_region*r_region - deltaIdx_y*deltaIdx_y));
            int temp = r_region-deltaIdx_x;
            int startIdxCircle = start + temp;
            int endIdxCircle   = end   - temp;
            
            for(int j = startIdxCircle; j < end; j++){
                limit = ((start/mapDataIn.info.width) + 2)*mapDataIn.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && j < mapDataIn.data.size()){
                    switch(mapDataIn.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    tempResult = infoGainValue*(pow(mapDataIn.info.resolution,2));
    _mt.unlock();
    return tempResult;
}//计算给定地图中指定位置周围半径为r的圆形区域内最大矩形的信息增益(information gain)



double informationRectangleGain(const nav_msgs::OccupancyGrid& mapDataIn, const geometry_msgs::PointStamped & point, const double r){
    _mt.lock();
    double infoGainValue = 0, tempResult;
    int index, init_index, last_index, length, start, end, r_region, limit;
    double x = point.point.x, y = point.point.y;

    index = (floor((y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
                  
    r_region = int(r/mapDataIn.info.resolution);
    init_index = index-r_region*(mapDataIn.info.width+1);
    last_index = index+r_region*(mapDataIn.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if(last_index < mapDataIn.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                switch(mapDataIn.data[j]){
                    case  -1: {infoGainValue++; break;}
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                limit = ((start/mapDataIn.info.width) + 2)*mapDataIn.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && j < mapDataIn.data.size()){
                    switch(mapDataIn.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    tempResult = infoGainValue*(pow(mapDataIn.info.resolution,2));
    _mt.unlock();
    return tempResult;
}//计算矩形区域内的信息增益,根据给定的半径计算出矩形区域的起始索引init_index和结束索引last_index，以及矩形区域的长度length


// -----------------------------------  global variables
nav_msgs::OccupancyGrid     mapData, costmapData,semantic_map;
geometry_msgs::PointStamped clickedpoint;
visualization_msgs::Marker  points,semantic_points,line,end_detect,lines_data,semantic_line;
int robot_semantic_value,goal_semantic_value,points_end_detect_semantic_value;
int init_num = 1, semantic_num = 0 ;

// -----------------------------------  for genrating random numbers
rdm r; 

// -----------------------------------  Subscribers callback functions-
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	mapData=*msg;
    // std::cout << "assigner receives map" << std::endl;
    _mt.unlock();
}//接收并存储机器人所在环境的地图数据

void costmapMergedCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	costmapData=*msg;
    // std::cout << "assigner receives costmap" << std::endl;
    _mt.unlock();
    
}

void semanticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
    semantic_map = *msg;
    //std::cout << "semanticMapCallback receives semanticMapCallback" << std::endl;
    _mt.unlock();

}

int getSemanticValueAtPosition(const double& point1_x, const double& point1_y)
{
    _mt.lock();
    int map_width = semantic_map.info.width;
    int map_height = semantic_map.info.height;
    double map_resolution = semantic_map.info.resolution;
    int map_origin_x = semantic_map.info.origin.position.x;
    int map_origin_y = semantic_map.info.origin.position.y;

    int grid_x = (int)((point1_x - map_origin_x) / map_resolution);
    int grid_y = (int)((point1_y - map_origin_y) / map_resolution);

    // Check bounds
    if (grid_x < 0 || grid_x >= map_width || grid_y < 0 || grid_y >= map_height) {
        ROS_WARN("Requested position is out of map bounds.");
        return -1;  // Return an invalid value for out-of-bounds
    }

    // Calculate 1D index in the occupancy grid
    int index = grid_y * map_width + grid_x;

    // Retrieve and return semantic value at the specified position
    _mt.unlock();
    return semantic_map.data[index];


}

// void extended_linesCallBack(const visualization_msgs::Marker::ConstPtr& lines_msg)
// {
//     _mt.lock();
// 	lines_data=*lines_msg;
//     // std::cout << "assigner receives costmap" << std::endl;
//     _mt.unlock();
    
// }
//接收并存储机器人所在环境地图数据

// void endDetectCallback(const visualization_msgs::Marker::ConstPtr& msg)
// {
//     _mt.lock();
// 	end_detect=*msg;
//     // std::cout << "assigner receives costmap" << std::endl;
//     _mt.unlock();
// }

geometry_msgs::Point p,semantic_p;  //表示一个三维空间中的点坐标


void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 
	p.x=msg->point.x;
	p.y=msg->point.y;
	p.z=msg->point.z;
	points.points.push_back(p);
}//名为rvizCallBack，用于接收并存储RViz中用户在界面上点击的点坐标。该函数没有使用线程锁（mutex），因为该函数只会在主线程中被执行，不存在多线程访问的问题


int main(int argc, char **argv)
{
	// -------------------------------------random number generator
 	// this is an example of initializing by an array
	// you may use MTRand(seed) with any 32bit integer
	// as a seed for a simpler initialization
	unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
	MTRand_int32 irand(init, length); // 32-bit int generator
	MTRand drand; // double in [0, 1) generator, already init
    //使用Mersenne Twister随机数生成器库来生成随机数
	// -------------------------------------initialize variables
	std::string map_topic, costmap_topic, base_frame_topic, frontier_topic;
    std::string robot_frame, robot_base_frame, robot_ano_frame_preffix, robot_ano_frame_suffix;
    std::string trajectory_query_name, output_file, output_map_file;
	int rateHz, detectFrontierNum = 0,Frontier_Num;
    int count_num = 0;
	double costmap_pixel_threshold, info_radius;
    float hysteresis_radius, hysteresis_gain;
    geometry_msgs::PointStamped exploration_goal, goalRobotFrame, previous_goal_map_frame;
    float xdim, ydim, resolution, centerX, centerY, map_lengthX, map_lengthY, eta, range;
#ifdef SEND_PYTHON_MOVEBASE
    geometry_msgs::PoseStamped moveBaseGoal;
#endif
    // -------------------------------------initialized the variables which are in need in exploration
	std::vector<geometry_msgs::PointStamped> frontiers;
	int i=0;
	float xr,yr,xd,yd;
	std::vector<float> x_rand,x_nearest,x_new,Frontier_crash,crash_nearest,x_detect, choose_end_points,choose_end_points_semantic,x_nearest_semantic;
    double previous_goal_infoGain;
    int    n_robot, this_robot_idx;
    
    // -------------------------------------initialize the robot node
	ros::init(argc, argv, "rrt_exploration_node");
	ros::NodeHandle nh;
	std::string nodename, ns;
	nodename=ros::this_node::getName();//获取ROS节点的名称，并将其保存到变量nodename中
	
	// -------------------------------------fetch params
	ros::param::param<std::string>(nodename+"/namespace", ns, "robot1/");
	ros::param::param<std::string>(nodename+"/map_topic", map_topic, "robot1/map"); 
    ros::param::param<std::string>(nodename+"/costmap_topic", costmap_topic, "robot1/move_base/global_costmap/costmap");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_frame, "robot1/map");
    ros::param::param<std::string>(nodename+"/robot_base_frame", robot_base_frame, "robot1/base_link");
    ros::param::param<std::string>(nodename+"/robot_ano_frame_preffix", robot_ano_frame_preffix, "robot");
    ros::param::param<std::string>(nodename+"/robot_ano_frame_suffix", robot_ano_frame_suffix, "/map");
    ros::param::param<std::string>(nodename+"/trajectory_query_name", trajectory_query_name, "robot1/trajectory_query");
    ros::param::param<std::string>(nodename+"/output_file", output_file, "/home/oseasy/work/YOEO_ws/src/rrt/trajectory/robot1_rrt_trajectory.txt");
    ros::param::param<std::string>(nodename+"/output_map_file", output_map_file, "/home/oseasy/work/YOEO_ws/src/rrt/maps/robot1_rrt_explored_map.txt");

    ros::param::param<int>(nodename+"/n_robot", n_robot, 1);
    ros::param::param<int>(nodename+"/this_robot_idx", this_robot_idx, 1);

    ros::param::param<int>(nodename+"/rate", rateHz, 20);
    ros::param::param<double>(nodename+"/info_radius", info_radius, 1.0);
    ros::param::param<double>(nodename+"/costmap_pixel_threshold", costmap_pixel_threshold, 30);//成本地图（costmap）中像素阈值（pixel threshold）
	ros::param::param<float>(nodename+"/eta", eta, 0.5);

	ros::param::param<float>(nodename+"/hysteresis_radius", hysteresis_radius, 3.0);//滞后半径（hysteresis radius）
	ros::param::param<float>(nodename+"/hysteresis_gain", hysteresis_gain, 2.0); //滞后增益（hysteresis gain）

	ros::Rate rate(rateHz);

    // -------------------------------------Transforms between all robots initilization;
    std::vector<tf::StampedTransform> transform_vec;
    for (int i = 0; i < n_robot; i++){
        tf::StampedTransform trans_ ;
        tf::Vector3 translation;
        translation.setX(0);
        translation.setY(0);
        translation.setZ(0);
        translation.setW(0);

        trans_.setOrigin(translation);
        transform_vec.push_back(trans_);
    }//创建了 n_robot 个具有初始原点位置的 tf::StampedTransform 对象，并将它们存储在 transform_vec 中

    // -------------------------------------Initialize all robots' frame;
    std::vector<std::string> robots_frame;

    for (int i = 0; i < n_robot; i++){

        std::stringstream ss;              
        ss << robot_ano_frame_preffix;
        ss << i+1;
        ss << robot_ano_frame_suffix;

        robots_frame.push_back(ss.str());
    }//将 robot_ano_frame_preffix、i+1 和 robot_ano_frame_suffix 依次写入 ss 中。最后，使用 ss.str() 方法将 ss 转换为字符串，并将其添加到 robots_frame 中

	// -------------------------------------subscribe the map topics & clicked points
	tf::TransformListener listener;
	ros::Subscriber sub       = nh.subscribe(map_topic, 100 ,mapCallBack);	
    ros::Subscriber costMapSub= nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 10, costmapMergedCallBack);
	
    ros::Subscriber rviz_sub  = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, rvizCallBack);
    ros::Subscriber semantic_map_sub = nh.subscribe("/semantic_grid", 10, semanticMapCallback);
    //ros::Subscriber extended_lines_sub  = nh.subscribe<visualization_msgs::Marker>("/extended_lines",1, extended_linesCallBack);
//第一个订阅器 sub 用于接收名为 map_topic 的话题消息，消息类型为 nav_msgs::OccupancyGrid，队列长度为 100。当有新的消息到达时，将调用 mapCallBack 函数进行处理。

// 第二个订阅器 costMapSub 用于接收名为 costmap_topic 的话题消息，消息类型为 nav_msgs::OccupancyGrid，队列长度为 10。当有新的消息到达时，将调用 costmapMergedCallBack 函数进行处理。

// 第三个订阅器 rviz_sub 用于接收名为 /clicked_point 的话题消息，消息类型为 geometry_msgs::PointStamped，队列长度为 10。当有新的消息到达时，将调用 rvizCallBack 函数进行处理。


	// -------------------------------------publish the detected points for following processing & display
	ros::Publisher pub          = nh.advertise<visualization_msgs::Marker>(nodename+"_shapes", 10);
    ros::Publisher semantic_pub          = nh.advertise<visualization_msgs::Marker>(nodename+"_semantic_shapes", 10);
	ros::Publisher pub_frontier = nh.advertise<visualization_msgs::Marker>(nodename+"_detected_frontiers", 10);
    ros::Publisher pub_semantic_frontier = nh.advertise<visualization_msgs::Marker>(nodename+"_semantic_detected_frontiers", 10);
    //第一个发布器 pub 用于发布名为 nodename+"_shapes" 的话题消息，消息类型为 visualization_msgs::Marker，队列长度为 10。该话题用于发布机器人运动轨迹、目标点等形状信息的可视化标记。

    //第二个发布器 pub_frontier 用于发布名为 nodename+"_detected_frontiers" 的话题消息，消息类型为 visualization_msgs::Marker，队列长度为 10。该话题用于发布探测到的新边界点或者未探索区域的可视化标记。

#ifdef SEND_PYTHON_ACTIONLIB
	ros::Publisher goalPub    = nh.advertise<geometry_msgs::PointStamped>("/robot1/rrt_goal", 10);
#endif//创建了一个发布器（Publisher），用于发布名为 "/robot1/rrt_goal" 的话题消息，消息类型为 geometry_msgs::PointStamped，队列长度为 10。如果定义了 SEND_PYTHON_ACTIONLIB 宏，则会创建该发布器。
#ifdef SEND_PYTHON_MOVEBASE
    ros::Publisher goalMoveBasePub = nh.advertise<move_base_msgs::MoveBaseGoal>("/robot1/rrt_goal", 10);
#endif//创建了一个发布器（Publisher），用于发布名为 "/robot1/rrt_goal" 的话题消息，消息类型为 move_base_msgs::MoveBaseGoal，队列长度为 10。如果定义了 SEND_PYTHON_MOVEBASE 宏，则会创建该发布器

	// -------------------------------------wait until map is received
    std::cout << ns << "wait for map "<< std::endl;
	while (mapData.header.seq < 1  or  mapData.data.size() < 1 ){  ros::spinOnce();  ros::Duration(0.1).sleep(); }
	std::cout << ns << "wait for costmap "<< std::endl;
    while ( costmapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}
    
    // ------------------------------------- save trajectory when finishing exploration
    ros::ServiceClient trajectory_query_client = nh.serviceClient<cartographer_ros_msgs::TrajectoryQuery>(trajectory_query_name);//创建了一个 ServiceClient（服务客户端），用于调用名为 trajectory_query_name 的服务。
    
    std::cout << ns << "wait for trajectory service"<< std::endl;
    trajectory_query_client.waitForExistence();//通过调用 waitForExistence() 方法等待服务的存在性，确保服务端已经启动并可供调用。该方法会一直阻塞程序直到服务可用或超时

    // ------------------------------------- action lib
#ifdef SEND_CPP_ACTIONLIB 
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(ns + "/move_base", true);// MoveBase 客户端（SimpleActionClient）,用于向名为 ns + "/move_base" 的 MoveBase 服务器端发送路径规划目标
    std::cout << ns << "wait for actionserver"<< std::endl;//waitForServer() 方法等待服务器端启动。该方法会一直阻塞程序直到服务器端启动或超时时间到达
    ac.waitForServer();
#endif
#ifdef SEND_CPP_ACTIONLIB
    move_base_msgs::MoveBaseGoal robotGoal;
    robotGoal.target_pose.header.frame_id = robot_frame;
    robotGoal.target_pose.pose.position.z = 0;
    robotGoal.target_pose.pose.orientation.w = 1.0;
#endif//设置机器人的目标位置和方向_ z 坐标设置为 0_w 分量设置为 1.0 表示机器人目标方向不发生旋转

	// -------------------------------------initilize the visualized points & lines  
	points.header.frame_id  = mapData.header.frame_id;
	points.header.stamp     = ros::Time(0);
	points.type 			= points.POINTS;
	points.action           = points.ADD;
	points.pose.orientation.w =1.0;
	points.scale.x 			= 0.3; 
	points.scale.y			= 0.3; 
	points.color.r 			= 1.0;   // 255.0/255.0;
	points.color.g 			= 0.0;   // 0.0/255.0;
	points.color.b 			= 0.0;   // 0.0/255.0;
	points.color.a			= 1.0;
	points.lifetime         = ros::Duration();

    semantic_points.header.frame_id  = mapData.header.frame_id;
	semantic_points.header.stamp     = ros::Time(0);
	semantic_points.type 			 = points.POINTS;
	semantic_points.action           = points.ADD;
	semantic_points.pose.orientation.w =1.0;
	semantic_points.scale.x 	     = 0.5; 
	semantic_points.scale.y			 = 0.5; 
	semantic_points.color.r 		 = 0.0;   // 255.0/255.0;
	semantic_points.color.g 		 = 1.0;   // 0.0/255.0;
	semantic_points.color.b 		 = 0.0;   // 0.0/255.0;
	semantic_points.color.a		     = 1.0;
	semantic_points.lifetime         = ros::Duration();

	line.header.frame_id    = mapData.header.frame_id;
	line.header.stamp       = ros::Time(0);
	line.type				= line.LINE_LIST;
	line.action             = line.ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x 			= 0.03;
	line.scale.y			= 0.03;
	line.color.r			= 1.0;   // 0.0/255.0;
	line.color.g			= 0.0;   // 0.0/255.0;
	line.color.b 			= 1.0;   // 236.0/255.0;
	line.color.a 			= 1.0;
	line.lifetime           = ros::Duration();

    semantic_line.header.frame_id    = mapData.header.frame_id;
	semantic_line.header.stamp       = ros::Time(0);
	semantic_line.type				= line.LINE_LIST;
	semantic_line.action             = line.ADD;
	semantic_line.pose.orientation.w = 1.0;
	semantic_line.scale.x 			= 0.03;
	semantic_line.scale.y			= 0.03;
	semantic_line.color.r			= 0.0;   // 0.0/255.0;
	semantic_line.color.g			= 1.0;   // 0.0/255.0;
	semantic_line.color.b 			= 0.0;   // 236.0/255.0;
	semantic_line.color.a 			= 1.0;
	semantic_line.lifetime           = ros::Duration();

#ifdef IF_DETECT_END
    ros::Publisher pub_end_detect = nh.advertise<visualization_msgs::Marker>(nodename+"_end_detect", 10);
    visualization_msgs::Marker  points_end_detect;
	points_end_detect.header.frame_id  = mapData.header.frame_id;//使用了 mapData.header.frame_id，表示参考坐标系与 mapData 的参考坐标系相同
	points_end_detect.header.stamp     = ros::Time(0);
	points_end_detect.type 			   = points_end_detect.POINTS;
	points_end_detect.action           = points_end_detect.ADD;
	points_end_detect.pose.orientation.w =1.0;
	points_end_detect.scale.x 			= 0.3; 
	points_end_detect.scale.y			= 0.3; 
	points_end_detect.color.r 			= 0.0;   // 255.0/255.0;
	points_end_detect.color.g 			= 0.0;   // 0.0/255.0;
	points_end_detect.color.b 			= 1.0;   // 0.0/255.0;
	points_end_detect.color.a			= 1.0;
	points_end_detect.lifetime         = ros::Duration();

#endif
	// -------------------------------------calculate the map size and the center point which are used in random number generation afterward
	// centerX     =   costmapData.info.origin.position.x + map_lengthX*.5;
	// centerY     =   costmapData.info.origin.position.y + map_lengthY*.5;
	// ROS_INFO("Map width: %3.2f: height %3.2f", map_lengthX, map_lengthY);

	// -------------------------------------set the origin of the RRT
	std::vector< std::vector<float>  > V;  
	std::vector<float> xnew; 
	std::cout << ns << "wait for transform"<< std::endl;
	tf::StampedTransform transform;
	int  temp=0;//通过循环等待获取坐标变换信息
	while (temp==0){
		try{
			temp=1;
			listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform); //mapData.header.frame_id 坐标系到 robot_base_frame 坐标系的坐标变换信息，并将其保存到 transform 变量中
		}
		catch (tf::TransformException ex){
			temp=0;
			ros::Duration(0.1).sleep();
		}//如果获取坐标变换信息时出现异常（tf::TransformException），则将 temp 设置为 0，并通过 ros::Duration(0.1).sleep() 函数暂停 0.1 秒，再进行下一次循环
	}

	xnew.push_back(transform.getOrigin().x()+0.3);//偏移量 0.3 的作用可能是调整机器人在地图中的位置。例如，如果机器人底盘设计不够准确，或者机器人与地面的接触点不够稳定，那么在进行坐标变换时可能会产生一些误差，导致机器人的实际位置与地图中的位置有一定偏差。因此，通过加上一个固定的偏移量来调整机器人在地图中的位置。
	xnew.push_back(transform.getOrigin().y()); 
	V.push_back(xnew);

    // -------------------------------------initialize the previous goal point
    previous_goal_map_frame.header.frame_id = robot_frame;
    previous_goal_map_frame.point.x         = xnew[0]+mapData.info.resolution*3;
    previous_goal_map_frame.point.y         = xnew[1]+mapData.info.resolution*3;//这里的mapData.info.resolution*3表示将目标位置向右上方偏移3个网格的距离，其中mapData.info.resolution是地图的分辨率，乘以3是为了确保目标位置足够远离机器人当前位置，避免与当前位置太近而导致路径规划算法的不准确性
    previous_goal_map_frame.point.z         = 0;

	std:: cout << ns << "RRT Tree starts growing origin: (" << xnew[0] << ", "  << xnew[1] << ")" << std::endl;//显示了RRT树开始生长的起点坐标，即机器人当前位置

    // -------------------------------------wait the clicked points  
	std::cout << ns << "wait to start" << std::endl;
    while(points.points.size()<1)
	{
		ros::spinOnce();
		// pub_frontier.publish(points);
	}
    // -------------------------------------clear clicked points
	points.points.clear();
	// pub_frontier.publish(points);
    //---------------------------------------------------------------
    //------------------     Main   Loop     ------------------------
    //---------------------------------------------------------------
    // int number_iteration_nofrontier = 0;
#ifdef IF_DETECT_END
    int no_targets_count = 0;
    int iteration_counts= 0;
#endif
    int number_iteration = 0;

    // -------------------------------------activate experiment record node
    ros::Publisher start_pub = nh.advertise<std_msgs::String>("/start_exp", 1);
    std_msgs::String start_msg;
    std::stringstream ss;
    ss << "Start!";
    start_msg.data = ss.str();//将一个字符串消息发布到主题（Topic）/start_exp
    std::cout<<"Start"<<std::endl;

	while (ros::ok()){
        start_pub.publish(start_msg);

        // ------------------------------- detect the end of exploration.
 #ifdef IF_DETECT_END
        iteration_counts++;
        if(iteration_counts == 100){
            iteration_counts = 0;
                // ---------------------------------------- variables from ROS input;
            int HEIGHT = mapData.info.height;
            int WIDTH  = mapData.info.width;
            ros::spinOnce();
            // ---------------------------------------- define variables;
            std::vector<int* > obstacles, path, targets,end_points;
            int map[HEIGHT*WIDTH];

            // ---------------------------------------- initialize the map & dismap
            for (int i=0; i<HEIGHT; i++)
            {
                for (int j=0; j<WIDTH; j++)
                {
                    map[i*WIDTH + j] = (int) mapData.data[i*mapData.info.width + j];
                }
            }//我们将地图数据从一维数组的形式转换为二维数组的形式,将获取到的地图数据转换为整型

            // ------------------------------------------ find the obstacles & targets
            /* for small place */
            //避免处理地图边缘部分
            for (int i = 2; i < HEIGHT-2; i++){
                for (int j = 2; j < WIDTH-2; j++){
                    if(map[i*WIDTH + j] == 100){
                        obstacles.push_back(new int[2]{i,j});
                    }//表示该位置是可访问的前沿
                    else if(map[i*WIDTH + j] == -1){
                        // accessiable frontiers
                        int numFree = 0, temp1 = 0;

                        if (map[(i + 1)*WIDTH + j] == 0){
                            temp1 += (map[(i + 2)*WIDTH + j    ] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);
                        }

                        if (map[i*WIDTH + j + 1] == 0){
                            temp1 = 0;
                            temp1 += (map[      i*WIDTH + j + 2] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);
                        }

                        if (map[(i - 1) *WIDTH + j] == 0){
                            temp1 = 0;
                            temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 2)*WIDTH + j    ] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);
                        }

                        if (map[i * WIDTH + j - 1] == 0){
                            temp1 = 0;
                            temp1 += (map[    i  *WIDTH + j - 2] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);
                        }

                        if( numFree > 0 ) {
                            targets.push_back(new int[2]{i,j});
                        }
                    }//这种算法通常用于基于栅格地图的路径规划或者避障算法中，用于评估周围环境的可行性
                }
            }
            
            /* for large place */
            /*
            for (int i = 2; i < HEIGHT-2; i++){
                for (int j = 2; j < WIDTH-2; j++){
                    if(map[i*WIDTH + j] == 100){
                        obstacles.push_back(new int[2]{i,j});
                    }
                    else if(map[i*WIDTH + j] == -1){
                        // accessiable frontiers
                        int numFree = 0, temp1 = 0, num_unknown = 0;

                        if (map[(i + 1)*WIDTH + j] == 0){
                            temp1 += (map[(i + 2)*WIDTH + j    ] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);

                            num_unknown += (map[(i + 2)*WIDTH + j    ] == -1) ? 1 : 0;
                            num_unknown += (map[(i + 1)*WIDTH + j + 1] == -1) ? 1 : 0;
                            num_unknown += (map[(i + 1)*WIDTH + j - 1] == -1) ? 1 : 0;
                        }

                        if (map[i*WIDTH + j + 1] == 0){
                            temp1 = 0;
                            temp1 += (map[      i*WIDTH + j + 2] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);

                            num_unknown += (map[      i*WIDTH + j + 2] == 0) ? 1 : 0;
                            num_unknown += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            num_unknown += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        }

                        if (map[(i - 1) *WIDTH + j] == 0){
                            temp1 = 0;
                            temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 2)*WIDTH + j    ] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);

                            num_unknown += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                            num_unknown += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            num_unknown += (map[(i - 2)*WIDTH + j    ] == 0) ? 1 : 0;
                        }

                        if (map[i * WIDTH + j - 1] == 0){
                            temp1 = 0;
                            temp1 += (map[    i  *WIDTH + j - 2] == 0) ? 1 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            numFree += (temp1 > 0);

                            num_unknown += (map[    i  *WIDTH + j - 2] == 0) ? 1 : 0;
                            num_unknown += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                            num_unknown += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        }

                        if( numFree > 0  && num_unknown>3) {
                            targets.push_back(new int[2]{i,j});
                        }
                    }
                }
            }
            */

            // ------------------------------------------ remove targets within the inflation layer of costmap.
            {
                // std::cout << "number targets" << targets.size() << std::endl;
        
                for (int idx_target = targets.size()-1; idx_target >= 0; idx_target--) {
                    
                    float loc_x = targets[idx_target][1]*mapData.info.resolution + mapData.info.origin.position.x;
                    float loc_y = targets[idx_target][0]*mapData.info.resolution + mapData.info.origin.position.y;
                    int index_costmap = (loc_y - costmapData.info.origin.position.y)/costmapData.info.resolution * costmapData.info.width + (loc_x - costmapData.info.origin.position.x)/costmapData.info.resolution;
                    if (costmapData.data[index_costmap] >0){
                        targets.erase(targets.begin() + idx_target);
                        continue;
                    }
                }
                // std::cout << "(costmap) number targets after erase" << targets.size() << std::endl;
            }//targets的二维向量，检查每个目标的位置是否在代价地图中有障碍物，并将有障碍物的目标从targets中删除
            
            // ------------------------------------------ remove targets within the inflation radius of obstacles.
            {
                for(int idx_target = targets.size()-1; idx_target>=0; idx_target--) {
                    for (int i = 0; i < obstacles.size(); i++) {
                        if (std::abs(targets[idx_target][0] - obstacles[i][0]) +
                            std::abs(targets[idx_target][1] - obstacles[i][1]) < 4) {
                            targets.erase(targets.begin() + idx_target);
                            break;
                        }
                    }
                }
                // std::cout << "(inforadius) number targets after erase" << targets.size() << std::endl;
            }//通过计算目标和障碍物在x和y方向上的差值的绝对值之和，判断目标和障碍物之间的距离是否小于4。如果距离小于4，则将该目标从targets中删除，并使用break语句跳出内部循环。


            // ------------------------------------------ exploration finish detection
            if(targets.size() < 2){
                if(no_targets_count == 4){
                    std::cout << "exploration done" << std::endl;
                    std::vector<geometry_msgs::PointStamped> path_list;
                    cartographer_ros_msgs::TrajectoryQuery srv;
                    srv.request.trajectory_id = 0;
                    trajectory_query_client.call(srv);
                    double trajectory_length, exploration_time;
                    exploration_time = srv.response.trajectory[0].header.stamp.sec;
                    exploration_time =srv.response.trajectory.back().header.stamp.sec - exploration_time;
                    std::cout << "exploration_time is:" << exploration_time << " seconds" << std::endl;
                    std::ofstream ofile(output_file);
                    double trajectory_x = srv.response.trajectory[0].pose.position.x;
                    double trajectory_y = srv.response.trajectory[0].pose.position.y;                
                    ofile << "[";
                    ofile << trajectory_x << "," << trajectory_y << std::endl;
                    for (int i = 1; i < srv.response.trajectory.size(); i++){
                        double temp_x = srv.response.trajectory[i].pose.position.x;
                        double temp_y = srv.response.trajectory[i].pose.position.y;
                        ofile << temp_x  << ", " <<  temp_y << ";" << std::endl;
                        double delta_x = trajectory_x - temp_x;
                        double delta_y = trajectory_y - temp_y;
                        trajectory_length += std::sqrt(delta_x*delta_x + delta_y*delta_y);
                        trajectory_x = temp_x;
                        trajectory_y = temp_y; 
                    }//该代码段的作用是将srv.response.trajectory中的坐标信息写入文件，并计算轨迹的长度。每次循环都会更新上一个点的坐标，以便在下一次循环中计算与当前点的位置差。这样，通过累加每两个相邻点之间的距离，最终可以得到整个轨迹的长度。
                    ofile << "]" << std::endl;
                    ofile.close();
                    std::cout << "exploration trajectory length = " << trajectory_length << " meters" << std::endl;
                    
                    std::ofstream ofile2(output_map_file);//该代码段的作用是将srv.response.trajectory中的坐标信息写入文件，并计算轨迹的长度。每次循环都会更新上一个点的坐标，以便在下一次循环中计算与当前点的位置差。这样，通过累加每两个相邻点之间的距离，最终可以得到整个轨迹的长度。;
                    ofile2 << "map Origin (" << mapData.info.origin.position.x << " ," << mapData.info.origin.position.y << ")" << std::endl;
                    for(int i = 0; i < mapData.data.size(); i++){
                        ofile2 << mapData.data[i] << " ";
                    }
                    ofile2.close();

                    return 0;
                }   

                no_targets_count ++;
                std::cout << "no targets count = " << no_targets_count << std::endl;
                rate.sleep();
                continue;
            }
            else{

                double reso = mapData.info.resolution;
                double ori_x =  mapData.info.origin.position.x;
                double ori_y =  mapData.info.origin.position.y;
                points_end_detect.points.clear();
                for( int k_ = 0; k_ < targets.size();k_++){
                    geometry_msgs::Point tempPoint_;
                    tempPoint_.x = targets[k_][1]*reso + ori_x; 
                    tempPoint_.y = targets[k_][0]*reso + ori_y; 
                    points_end_detect.points.push_back(tempPoint_);
                }
                pub_end_detect.publish(points_end_detect);

                no_targets_count = 0;
            }
        }
 #endif

        

        // ------------------------------- find fronters ****************************************important***************************************8
            map_lengthX = mapData.info.width*mapData.info.resolution;
            map_lengthY = mapData.info.height*mapData.info.resolution;
            // std::cout << "start rrt tree growing" << std::endl;
            // Sample free
            int numPoints = points_end_detect.points.size();
            // std::cout<<"numPoints:"<<numPoints<<std::endl;
            if(numPoints > 10)
            {
            if(count_num >= 1000)
            {
                count_num = 0;
                x_detect.clear();

                // int j = 0;
                // float minDist = std::numeric_limits<float>::max();
                // for (int i = 0; i < points_end_detect.points.size(); i++) {
                //     double travel_distance =  norm2Dpoints_2(transform.getOrigin().x(), transform.getOrigin().y(), points_end_detect.points[i]);
                //     if (travel_distance < minDist) {
                //         minDist = travel_distance ;
                //         j = i;
                //     }
                // }
                int j = 0;
                robot_semantic_value = getSemanticValueAtPosition(transform.getOrigin().x(), transform.getOrigin().y());
                for (int i = 0; i < points_end_detect.points.size(); i++) {
                    points_end_detect_semantic_value = getSemanticValueAtPosition(points_end_detect.points[i].x,points_end_detect.points[i].y);
                    if (points_end_detect_semantic_value != 100)
                    {
                    if (robot_semantic_value == points_end_detect_semantic_value) {
                        j = i;
                        break;
                    }
                    }
                }
                 xd = (points_end_detect.points[j].x);
                 yd = (points_end_detect.points[j].y);
                 x_detect.push_back(xd);x_detect.push_back(yd);
                 semantic_points.points.clear();
                 semantic_p.x = xd;
                 semantic_p.y = yd;
                 semantic_p.z = 0;
                 semantic_points.points.push_back(semantic_p);
                 pub_semantic_frontier.publish(semantic_points); 
                 x_nearest_semantic=Nearest(V,x_detect);//在所有的robot中找最近的
                 //std::cout<<"x_nearest_semantic.x = "<<x_nearest_semantic[0]<<"x_nearest_semantic.y = "<<x_nearest_semantic[1]<<"      "<<"x_detect.x = "<<xd<<"x_detect.y = "<<yd<<std::endl;
                 x_new=Steer(x_nearest_semantic, x_detect, eta);
                 char checking=ObstacleFree(x_nearest_semantic, x_new, mapData);
            //Froniters  frontiers列表用于存储探索的目标点，而line列表则用于可视化路径段
                    if (checking=='2'){
                        exploration_goal.header.stamp=ros::Time(0);
                        exploration_goal.header.frame_id=mapData.header.frame_id;
                        exploration_goal.point.x=x_new[0];
                        exploration_goal.point.y=x_new[1];
                        exploration_goal.point.z=0.0;
                        // if(mapValue(mapData, exploration_goal)==-1 || mapValue(costmapData, exploration_goal) < costmap_pixel_threshold){
                        frontiers.push_back(exploration_goal);
                        // }
                        // points.points.push_back(exploration_goal.point);
                        semantic_line.points.push_back(exploration_goal.point);
                        p.x=x_nearest_semantic[0]; 
                        p.y=x_nearest_semantic[1]; 
                        p.z=0.0;
                        semantic_line.points.push_back(p);//前沿就去到最近点
                        // points.pose.
                    }//Free
                    else if (checking=='0'){
                        V.push_back(x_new);
                        
                        p.x=x_new[0]; 
                        p.y=x_new[1]; 
                        p.z=0.0;
                        semantic_line.points.push_back(p);
                        p.x=x_nearest_semantic[0]; 
                        p.y=x_nearest_semantic[1]; 
                        p.z=0.0;
                        semantic_line.points.push_back(p);	//最近点和终点也添加 
            }
            semantic_pub.publish(semantic_line); 
            number_iteration++; 
            }else
            {
                count_num++ ;
                //semantic_points.points.clear();
                x_rand.clear();
                xr=(drand()*map_lengthX) + mapData.info.origin.position.x;
                yr=(drand()*map_lengthY) + mapData.info.origin.position.y;
                x_rand.push_back( xr ); x_rand.push_back( yr );//使用drand()函数生成一个0到1之间的随机数，然后将其乘以地图的横向和纵向长度map_lengthX和map_lengthY，得到一个在地图范围内的随机点        将该随机点的坐标值与地图原点的坐标相加，得到该点在地图中的绝对坐标，并将其分别存储在xr和yr变量
                x_nearest=Nearest(V,x_rand);//在所有的robot中找最近的
                x_new=Steer(x_nearest, x_rand, eta);
            // ------------------ ObstacleFree -1:unkown (frontier region)  0:free 1:obstacle
            char checking=ObstacleFree(x_nearest, x_new, mapData);
            //Froniters  frontiers列表用于存储探索的目标点，而line列表则用于可视化路径段
            if (checking=='2'){
                exploration_goal.header.stamp=ros::Time(0);
                exploration_goal.header.frame_id=mapData.header.frame_id;
                exploration_goal.point.x=x_new[0];
                exploration_goal.point.y=x_new[1];
                exploration_goal.point.z=0.0;
                // if(mapValue(mapData, exploration_goal)==-1 || mapValue(costmapData, exploration_goal) < costmap_pixel_threshold){
                frontiers.push_back(exploration_goal);
                // }
                // points.points.push_back(exploration_goal.point);
                line.points.push_back(exploration_goal.point);
                p.x=x_nearest[0]; 
                p.y=x_nearest[1]; 
                p.z=0.0;
                line.points.push_back(p);//前沿就去到最近点
                // points.pose.
            }//Free
            else if (checking=='0'){
                V.push_back(x_new);
                
                p.x=x_new[0]; 
                p.y=x_new[1]; 
                p.z=0.0;
                line.points.push_back(p);
                p.x=x_nearest[0]; 
                p.y=x_nearest[1]; 
                p.z=0.0;
                line.points.push_back(p);	//最近点和终点也添加
                
            }
            // -----------------------Frontier_crash ---------ObstacleFree -1:unkown (frontier region)  0:free 1:obstacle
            pub.publish(line); 
            number_iteration++; 
            }
            }
            else{
                count_num++;
                 x_rand.clear();
                xr=(drand()*map_lengthX) + mapData.info.origin.position.x;
                yr=(drand()*map_lengthY) + mapData.info.origin.position.y;
                x_rand.push_back( xr ); x_rand.push_back( yr );//使用drand()函数生成一个0到1之间的随机数，然后将其乘以地图的横向和纵向长度map_lengthX和map_lengthY，得到一个在地图范围内的随机点        将该随机点的坐标值与地图原点的坐标相加，得到该点在地图中的绝对坐标，并将其分别存储在xr和yr变量
                x_nearest=Nearest(V,x_rand);//在所有的robot中找最近的
                x_new=Steer(x_nearest, x_rand, eta);
            // ------------------ ObstacleFree -1:unkown (frontier region)  0:free 1:obstacle
            char checking=ObstacleFree(x_nearest, x_new, mapData);
            //Froniters  frontiers列表用于存储探索的目标点，而line列表则用于可视化路径段
            if (checking=='2'){
                exploration_goal.header.stamp=ros::Time(0);
                exploration_goal.header.frame_id=mapData.header.frame_id;
                exploration_goal.point.x=x_new[0];
                exploration_goal.point.y=x_new[1];
                exploration_goal.point.z=0.0;
                // if(mapValue(mapData, exploration_goal)==-1 || mapValue(costmapData, exploration_goal) < costmap_pixel_threshold){
                frontiers.push_back(exploration_goal);
                // }
                // points.points.push_back(exploration_goal.point);
                line.points.push_back(exploration_goal.point);
                p.x=x_nearest[0]; 
                p.y=x_nearest[1]; 
                p.z=0.0;
                line.points.push_back(p);//前沿就去到最近点
                // points.pose.
            }//Free
            else if (checking=='0'){
                V.push_back(x_new);
                
                p.x=x_new[0]; 
                p.y=x_new[1]; 
                p.z=0.0;
                line.points.push_back(p);
                p.x=x_nearest[0]; 
                p.y=x_nearest[1]; 
                p.z=0.0;
                line.points.push_back(p);	//最近点和终点也添加
                
            }
            // -----------------------Frontier_crash ---------ObstacleFree -1:unkown (frontier region)  0:free 1:obstacle
            pub.publish(line); 
            number_iteration++; 
            }

                    // std::cout << "targets number:" << targets.size() << std::endl;
                    // std::cout << "clear no targets count" << std::endl;
                
        //这段代码的作用是将检测到的所有目标点转换为地图坐标，并通过pub_end_detect发布这些目标点。points_end_detect，使得所有目标点能够被其他节点获取并使用

        // ------------------------------- add previous goal
        if(mapValue(mapData, previous_goal_map_frame)!=-1 || mapValue(costmapData, previous_goal_map_frame) > costmap_pixel_threshold){
        }
        else{
            frontiers.push_back(previous_goal_map_frame);
        }

        // ------------------------------- remove old frontiers whose places have already been explored & the invalid frontiers.
        // std::cout <<  ns << "detects frontiers' number " << frontiers.size() << std::endl;
        for(int i= frontiers.size()-1; i>-1;i--){
            if(mapValue(mapData, frontiers[i])!=-1 || mapValue(costmapData, frontiers[i]) > costmap_pixel_threshold){
                frontiers.erase(frontiers.begin() + i);
            }
        }

        // // if(frontiers.size() == 0){number_iteration_nofrontier++;}
        // rate.sleep();
        // continue;
        //判断当前探测到的未知区域数量与上一次探测到的未知区域数量之差是否大于 25
        if( ((frontiers.size() - detectFrontierNum) > 25) || (number_iteration > 500) ){

            // ------------------------------- clear detection's iteration recording.
            number_iteration = 0;  

            // ------------------------------- initializes current goal's information.
            double current_goal_score = -500.0, current_goal_infoGain = -500.0;
            int current_goal_idx   = -1;
            
         
            // ------------------------------- remove frontiers lying out of the valid map (uncertainty of map will cause some frontiers lie out of valid map).
            for(int i= frontiers.size()-1; i>-1;i--){
                bool condition_ = false;
                double x_,y_;
                int tempResult[4];
                x_ = frontiers[i].point.x;
                y_ = frontiers[i].point.y;
                int idx = (floor((y_-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_+mapData.info.resolution-mapData.info.origin.position.x)/mapData.info.resolution));
                tempResult[0] = mapData.data[idx];
                idx = floor((y_+mapData.info.resolution-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width+floor((x_-mapData.info.origin.position.x)/mapData.info.resolution);
                tempResult[1] = mapData.data[idx];
                idx = (floor((y_-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_-mapData.info.resolution-mapData.info.origin.position.x)/mapData.info.resolution));
                tempResult[2] = mapData.data[idx];
                idx = (floor((y_-mapData.info.resolution-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_-mapData.info.origin.position.x)/mapData.info.resolution));
                tempResult[3] = mapData.data[idx];
                // if surring pixels are all unknown, elminate it.
                if( (tempResult[0]+tempResult[1]+tempResult[2]+tempResult[3]) == -4){
                    frontiers.erase(frontiers.begin() + i);
                }
            }

            detectFrontierNum = frontiers.size();

            // ------------------------------- start a new iteration if no frontiers remain.          
            // std::cout <<  ns << "remains frontiers' number " << frontiers.size() << std::endl;
            if(frontiers.size() == 0){
                rate.sleep();
                ros::spinOnce();
                continue;
            }
                        
            // ------------------------------- display valid frontiers.
            points.points.clear();
            for(int i= frontiers.size()-1; i>-1;i--){
                points.points.push_back(frontiers[i].point);
            }
            pub_frontier.publish(points);

            // ------------------------------- find the goal which has the highest score.improtant***********************************
            try{
                listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                std::cout << "don't receive transform from robot frame to map's frame" << std::endl;
                continue;
            }
            robot_semantic_value = getSemanticValueAtPosition(transform.getOrigin().x(), transform.getOrigin().y());

            for(int i=0; i<frontiers.size(); i++){
                double score, pixel, travel_distance, infoGain, Awards = 3;
                infoGain = informationRectangleGain(mapData, frontiers[i], info_radius);
                // if(infoGain<0.2 || pixel > costmap_pixel_threshold){continue;}
                travel_distance =  norm2Dpoints(transform.getOrigin().x(), transform.getOrigin().y(), frontiers[i]);//计算机器人到前沿点的行进距离（travel_distance）
                goal_semantic_value = getSemanticValueAtPosition(frontiers[i].point.x,frontiers[i].point.y);
                // hysteresis here is to reduce overlap as much as possible
                if(travel_distance <= hysteresis_radius)
                {infoGain*=hysteresis_gain;}
  
                score = 3*infoGain - travel_distance;
                // if (robot_semantic_value == goal_semantic_value && robot_semantic_value != -1 ) {

                //     score = 3*infoGain - 2*travel_distance + Awards;
                // }
                // if(robot_semantic_value != goal_semantic_value)
                // {
                //     score = 3*infoGain - 2*travel_distance - Awards;
                // }
               
                if(score > current_goal_score){
                    current_goal_score    = score;
                    current_goal_infoGain = infoGain;
                    current_goal_idx      = i;
                }
                 //std::cout<<"The score of this frontier is :"<<current_goal_score<<std::endl;
            }
           //  std::cout<<"The score of this frontier is :"<<current_goal_score<<std::endl;
            


            if(current_goal_idx == -1){
                continue;
            }
           
                
#ifdef SEND_PYTHON_ACTIONLIB 
            goalPub.publish(frontiers[current_goal_idx]);
#endif
            if(frontiers[current_goal_idx].header.frame_id != robot_frame){
                try{
                    // std::cout << "try converts wait for transform" << std::endl;
                    if(listener.waitForTransform(frontiers[current_goal_idx].header.frame_id, robot_frame, ros::Time(0), ros::Duration(5))){
                        // std::cout << "try converts goal point" << std::endl;
                        listener.transformPoint(robot_frame, frontiers[current_goal_idx], goalRobotFrame);
                        // std::cout << "convert done" << std::endl;
                    }
                }
                catch (tf::TransformException ex){
                    std::cout << "don't receive transform from robot frame to frontier's frame" << std::endl;
                    continue;
                }
            }
            else{
                goalRobotFrame = frontiers[current_goal_idx];
            }

#if defined SEND_PYTHON_MOVEBASE 
            // ------------------------------ send move_base goal actionlib 
            std::cout << "send goal point" << std::endl;
            moveBaseGoal.header = mapData.header;
            moveBaseGoal.pose.orientation.w = 1.0;
            moveBaseGoal.pose.position.x    = frontiers[current_goal_idx].point.x;
            moveBaseGoal.pose.position.y    = frontiers[current_goal_idx].point.y;
            std::cout << "defined movebase goal" << std::endl;
            goalMoveBasePub.publish(moveBaseGoal);
#endif

            //  ------------------------------ actionlib 
#ifdef SEND_CPP_ACTIONLIB 
           
            if (init_num == 1)
            {
              robotGoal.target_pose.pose.position.x = 4.67;
              robotGoal.target_pose.pose.position.y = -0.3266630172729492; 
              ac.sendGoal(robotGoal);
              init_num += 1;
                    
            }
            // else if (!semantic_points.points.empty() && semantic_num == 100) {

            //   semantic_num = 0;
            //   std::cout << "semantic_points is running !!!!!!" << std::endl;
            
            //   robotGoal.target_pose.pose.position.x = semantic_p.x;
            //   robotGoal.target_pose.pose.position.y = semantic_p.y; 
            //   ac.sendGoal(robotGoal);

            // }
            else{
            semantic_num ++ ;
            robotGoal.target_pose.pose.position.x = goalRobotFrame.point.x;
            robotGoal.target_pose.pose.position.y = goalRobotFrame.point.y;
            robotGoal.target_pose.header.stamp    = ros::Time(0);
            if(abs(previous_goal_map_frame.point.x-robotGoal.target_pose.pose.position.x) + abs(previous_goal_map_frame.point.y-robotGoal.target_pose.pose.position.y) < 3*mapData.info.resolution){
                ros::spinOnce();
                continue;
            }
            else {
            ac.sendGoal(robotGoal);
            previous_goal_map_frame = frontiers[current_goal_idx];
        }
             
           
            }
            
          
#endif
            // std::cout << ns <<  "flush frontiers" << std::endl;
            // frontiers.clear();
            // std::vector<geometry_msgs::PointStamped>().swap(frontiers);
        }

        if(n_robot>1){
            bool reset_rrt = false;
            for (int i_ = 0; i_ < n_robot; i_++){
                if(i_ + 1 == this_robot_idx){
                    continue;
                }

                tf::StampedTransform transform_;
                try{
                    listener.lookupTransform(robot_frame, robots_frame[i_], ros::Time(0), transform_);
                }
                catch (tf::TransformException &ex) {
                    continue;
                }

                float dis_origin = 0;
                dis_origin = abs(transform_.getOrigin().x() - transform_vec[i_].getOrigin().x()) ;
                dis_origin += abs(transform_.getOrigin().y() - transform_vec[i_].getOrigin().y()) ;
                dis_origin += abs(transform_.getOrigin().z() - transform_vec[i_].getOrigin().z()) ;
                dis_origin += abs(transform_.getRotation().x() - transform_vec[i_].getRotation().x()) ;
                dis_origin += abs(transform_.getRotation().y() - transform_vec[i_].getRotation().y()) ;
                dis_origin += abs(transform_.getRotation().z() - transform_vec[i_].getRotation().z()) ;
                dis_origin += abs(transform_.getRotation().w() - transform_vec[i_].getRotation().w()) ;

                if(dis_origin > THRESHOLD_TRANSFORM){
                    reset_rrt = true;
                    transform_vec[i_].setOrigin(transform_.getOrigin());
                    transform_vec[i_].setRotation(transform_.getRotation());
                }
                
            }
            
            if(reset_rrt){
                std::cout << ns <<  "flush frontiers" << std::endl;
                frontiers.clear();
                std::vector<geometry_msgs::PointStamped>().swap(frontiers);

                line.points.clear();
                semantic_line.points.clear();
                std::vector<std::vector<float> >().swap(V); 
                tf::StampedTransform transform_;
                int  temp_=0;
                std::vector<float> xnew_;
                while (temp_==0){
                    try{
                        listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform_);
                        temp_=1;
                    }
                    catch (tf::TransformException ex){
                        ros::Duration(0.1).sleep();
                    }
                }

                xnew_.push_back(transform_.getOrigin().x());
                xnew_.push_back(transform_.getOrigin().y()); 
                V.push_back(xnew_);
            }
        }

		ros::spinOnce();
		rate.sleep();
        // std::cout << ns << "loop ends" << std::endl;
	}

	return 0;
}
