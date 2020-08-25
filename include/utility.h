#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array> // c++11
#include <thread> // c++11
#include <mutex> // c++11


#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "planner/rollout_generator.h"


using namespace std;

extern const int NUM_COSTS = 4;

typedef pcl::PointXYZI PointType;
struct state_t;

class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    string _pointCloudTopic;

    //Occupancy Map Params
    float _mapResolution;
    float _occuMapInflation;
    float _occuMapField;

    // Filter Threshold Params
    float _sensorRangeLimitMin;
    float _sensorRangeLimitMax;
    float _sensorHeightLimitUpper;
    float _sensorHeightLimitDown;
    float _sensorCloudTimeout;

    int   _local_map_grid_num;
    float _local_map_length;

    // Alternative Path Params
    float _samplingTipMargin;
    float _samplingOutMargin;
    float _pathResolution;
    float _maxPathDistance;
    float _rollOutDensity;
    int   _rollOutNumber;
    int   _rollOutCenter;
    
    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("roboat_planning/_pointCloudTopic", _pointCloudTopic, "points_raw");

        nh.param<float>("roboat_planning/_mapResolution",    _mapResolution,    0.1);
        nh.param<float>("roboat_planning/_occuMapInflation", _occuMapInflation, 0.5);
        nh.param<float>("roboat_planning/_occuMapField",     _occuMapField,     0.5);

        nh.param<float>("roboat_planning/_sensorRangeLimitMin",    _sensorRangeLimitMin,    1.0);
        nh.param<float>("roboat_planning/_sensorRangeLimitMax",    _sensorRangeLimitMax,    30.0);
        nh.param<float>("roboat_planning/_sensorHeightLimitUpper", _sensorHeightLimitUpper, 0.5);
        nh.param<float>("roboat_planning/_sensorHeightLimitDown",  _sensorHeightLimitDown,  -0.2);
        nh.param<float>("roboat_planning/_sensorCloudTimeout",     _sensorCloudTimeout,   25);


        nh.param<float>("roboat_planning/_samplingTipMargin", _samplingTipMargin, 1.0);
        nh.param<float>("roboat_planning/_samplingOutMargin", _samplingOutMargin, 1.0);
        nh.param<float>("roboat_planning/_pathResolution",    _pathResolution,    0.1);
        nh.param<float>("roboat_planning/_maxPathDistance",   _maxPathDistance,   10.0);
        nh.param<float>("roboat_planning/_rollOutDensity",    _rollOutDensity,    0.1);
        nh.param<int>("roboat_planning/_rollOutNumber",       _rollOutNumber,     20);
        _rollOutCenter = _rollOutNumber / 2;
        if (_maxPathDistance < _sensorRangeLimitMax * 2.0)
            ROS_WARN("Assigned length for generating alternative paths might not be long enough!");

        _local_map_grid_num = round(_sensorRangeLimitMax * 4.0 / _mapResolution);
        _local_map_length = _sensorRangeLimitMax * 4.0;
    }

    nav_msgs::Path processPath(nav_msgs::Path pathIn)
    {
        pathIn = calculatePathYaw(pathIn);
        pathIn = fixPathDensity(pathIn);
        pathIn = smoothPath(pathIn);
        return pathIn;
    }

    nav_msgs::Path calculatePathYaw(nav_msgs::Path pathIn)
    {
        int length = pathIn.poses.size();
        if (length <= 1)
        {
            if (length == 1)
                pathIn.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            return pathIn;
        }

        for (int i = 0; i < length - 1; ++i)
        {
            double dx = pathIn.poses[i+1].pose.position.x - pathIn.poses[i].pose.position.x;
            double dy = pathIn.poses[i+1].pose.position.y - pathIn.poses[i].pose.position.y;
            double theta = atan2(dy, dx);
            pathIn.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        }

        pathIn.poses.back().pose.orientation = pathIn.poses[length-2].pose.orientation;

        return pathIn;
    }

    nav_msgs::Path smoothPath(nav_msgs::Path path)
    {
        if (path.poses.size() <= 2)
            return path;

        double weight_data = 0.45;
        double weight_smooth = 0.4;
        double tolerance = 0.05;

        nav_msgs::Path smoothPath_out = path;

        double change = tolerance;
        double xtemp, ytemp;
        int nIterations = 0;

        int size = path.poses.size();

        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < size - 1; i++) {
                xtemp = smoothPath_out.poses[i].pose.position.x;
                ytemp = smoothPath_out.poses[i].pose.position.y;

                smoothPath_out.poses[i].pose.position.x += weight_data * (path.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
                smoothPath_out.poses[i].pose.position.y += weight_data * (path.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);

                smoothPath_out.poses[i].pose.position.x += weight_smooth * (smoothPath_out.poses[i-1].pose.position.x + smoothPath_out.poses[i+1].pose.position.x - (2.0 * smoothPath_out.poses[i].pose.position.x));
                smoothPath_out.poses[i].pose.position.y += weight_smooth * (smoothPath_out.poses[i-1].pose.position.y + smoothPath_out.poses[i+1].pose.position.y - (2.0 * smoothPath_out.poses[i].pose.position.y));

                change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
                change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
            }
            nIterations++;
        }

        return smoothPath_out;
    }

    nav_msgs::Path fixPathDensity(nav_msgs::Path path)
    {
        if (path.poses.size() == 0 || _pathResolution == 0)
            return path;

        double dis = 0, ang = 0;
        double margin = _pathResolution * 0.01;
        double remaining = 0;
        int nPoints = 0;

        nav_msgs::Path fixedPath = path;
        fixedPath.poses.clear();
        fixedPath.poses.push_back(path.poses[0]);

        size_t start = 0, next = 1;
        while (next < path.poses.size())
        {
            dis += hypot(path.poses[next].pose.position.x - path.poses[next-1].pose.position.x, path.poses[next].pose.position.y - path.poses[next-1].pose.position.y) + remaining;
            ang = atan2(path.poses[next].pose.position.y - path.poses[start].pose.position.y, path.poses[next].pose.position.x - path.poses[start].pose.position.x);

            if (dis < _pathResolution - margin)
            {
                next++;
                remaining = 0;
            } else if (dis > (_pathResolution + margin))
            {
                geometry_msgs::PoseStamped point_start = path.poses[start];
                nPoints = dis / _pathResolution;
                for (int j = 0; j < nPoints; j++)
                {
                    point_start.pose.position.x = point_start.pose.position.x + _pathResolution * cos(ang);
                    point_start.pose.position.y = point_start.pose.position.y + _pathResolution * sin(ang);
                    point_start.pose.orientation = tf::createQuaternionMsgFromYaw(ang);
                    fixedPath.poses.push_back(point_start);
                }
                remaining = dis - nPoints * _pathResolution;
                start++;
                path.poses[start].pose.position = point_start.pose.position;
                dis = 0;
                next++;
            } else {
                dis = 0;
                remaining = 0;
                fixedPath.poses.push_back(path.poses[next]);
                next++;
                start = next - 1;
            }
        }

        return fixedPath;
    }
};

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

float pointDistance(PointType p)
{
    return sqrt((p.x*p.x)+(p.y*p.y)+(p.z*p.z));
}


struct state_t;
struct neighbor_t;

struct state_t
{
    float x = 0;
    float y = 0;
    float z = 0;
    float theta = 0;
    float costsToRoot[NUM_COSTS] = {FLT_MAX};

    int idx = -1;
    int idy = -1;
    int stateId = -1;

    bool validFlag = false;

    state_t* parentState = NULL;
    vector<neighbor_t> neighborList;
};

struct neighbor_t{
    state_t* neighbor;
    float edgeCosts[NUM_COSTS]; // the cost from this state to neighbor
    neighbor_t(){
        neighbor = NULL;
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = FLT_MAX;
    }
};

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    if (thisPub->getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud);
}

// coordinate system transform

// lidar = camera
// x = z
// y = x
// z = y
// roll = yaw
// pitch = roll
// yaw = pitch

// camera = lidar
// x = y
// y = z
// z = x
// roll = pitch
// pitch = yaw
// yaw = roll

#endif