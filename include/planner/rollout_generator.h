/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-03-15 11:50:55
 */

#ifndef ROLLOUT_GENERATOR_H
#define ROLLOUT_GENERATOR_H

// #include "utils/utils.h"


#include <ros/ros.h>
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>

using namespace std;

namespace UtilityNS
{

class GPSPoint {
public:
    double x;
    double y;
    double z;
    double yaw;

    GPSPoint()
    {
        x = 0;
        y = 0;
        z = 0;
        yaw = 0;
    }

    GPSPoint(const double& x, const double& y, const double& z, const double& yaw)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->yaw = yaw;
    }

    string ToString()
    {
        stringstream str;
        str.precision(12);
        str << "X:" << x << ", Y:" << y << ", Z:" << z << ", Yaw:" << yaw << endl;
        return str.str();
    }
};

class Rotation {
public:
    double x;
    double y;
    double z;
    double w;

    Rotation()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
};

class WayPoint {
public:
    GPSPoint pos;
    Rotation rot;
    double v;
    double cost;
    int laneId;

    WayPoint()
    {
        v = 0;
        cost = 0;
        laneId = -1;
    }

    WayPoint(const double& x, const double& y, const double& z, const double& a)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.yaw = a;

        v = 0;
        cost = 0;
        laneId = -1;
    }
};

class Mat3 {
    double m[3][3];

public:
    Mat3()
    {
        //initialize Identity by default
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = 0;

        m[0][0] = m[1][1] = m[2][2] = 1;
    }

    Mat3(double transX, double transY, bool mirrorX, bool mirrorY)
    {
        m[0][0] = (mirrorX == true) ? -1 : 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = (mirrorY == true) ? -1 : 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double transX, double transY)
    {
        m[0][0] = 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double rotation_angle)
    {
        double c = cos(rotation_angle);
        double s = sin(rotation_angle);
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = 0;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = 0;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(GPSPoint rotationCenter)
    {
        double c = cos(rotationCenter.yaw);
        double s = sin(rotationCenter.yaw);
        double u = rotationCenter.x;
        double v = rotationCenter.y;
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = -u * c + v * s + u;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = -u * s - v * c + v;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    GPSPoint operator*(GPSPoint v)
    {
        GPSPoint _v = v;
        v.x = m[0][0] * _v.x + m[0][1] * _v.y + m[0][2] * 1;
        v.y = m[1][0] * _v.x + m[1][1] * _v.y + m[1][2] * 1;
        return v;
    }
};

class RelativeInfo {
public:
    double perp_distance;
    double to_front_distance; //negative
    double from_back_distance;
    int iFront;
    int iBack;
    WayPoint perp_point;
    double angle_diff; // degrees
    double direct_distance;

    RelativeInfo()
    {
        perp_distance = 0;
        to_front_distance = 0;
        iFront = 0;
        iBack = 0;
        angle_diff = 0;
        direct_distance = 0;
    }
};

class DetectedObject {
public:
    int id;
    string label;
    WayPoint center;
    vector<GPSPoint> contour;
    double w;
    double l;
    double h;

    ros::Time start_time;

    DetectedObject()
    {
        id = 0;
        w = 0;
        l = 0;
        h = 0;
    }
};

class TrajectoryCost {
public:
    int index;
    int relative_index;

    double cost;
    double priority_cost; //0 to 1
    double transition_cost; // 0 to 1
    double lateral_cost;
    double longitudinal_cost;

    double closest_obj_distance;
    double closest_obj_velocity;

    bool bBlocked;
    double distance_from_center;

    TrajectoryCost()
    {
        index = -1;
        distance_from_center = 0;
        relative_index = -100;
        closest_obj_velocity = 0;
        priority_cost = 0;
        transition_cost = 0;
        cost = 0;
        closest_obj_distance = -1;
        lateral_cost = 0;
        longitudinal_cost = 0;
        bBlocked = false;
    }

    string ToString()
    {
        ostringstream str;
        str.precision(4);
        str << ", In : " << relative_index;
        str << ", Co : " << cost;
        str << ", Pr : " << priority_cost;
        str << ", Tr : " << transition_cost;
        str << ", La : " << lateral_cost;
        str << ", Lo : " << longitudinal_cost;
        str << ", Bl : " << bBlocked;
        str << "\n";

        return str.str();
    }
};

class PlanningParams {
public:
    double maxSpeed;
    double minSpeed;
    double planningDistance;
    double microPlanDistance;
    double carTipMargin;
    double rollInMargin;
    double rollInSpeedFactor;
    double pathDensity;
    double rollOutDensity;
    int rollOutNumber;
    double horizonDistance;
    double minFollowingDistance; //should be bigger than Distance to follow
    double minDistanceToAvoid; // should be smaller than minFollowingDistance and larger than maxDistanceToAvoid
    double maxDistanceToAvoid; // should be smaller than minDistanceToAvoid
    double lateralSkipDistance;
    double speedProfileFactor;
    double smoothingDataWeight;
    double smoothingSmoothWeight;
    double smoothingToleranceError;

    double stopSignStopTime;

    double additionalBrakingDistance;
    double verticalSafetyDistance;
    double horizontalSafetyDistancel;

    double giveUpDistance;

    int nReliableCount;

    bool enableLaneChange;
    bool enableSwerving;
    bool enableFollowing;
    bool enableHeadingSmoothing;
    bool enableTrafficLightBehavior;
    bool enableStopSignBehavior;

    bool enabTrajectoryVelocities;
    double minIndicationDistance;

    PlanningParams()
    {
        maxSpeed = 3;
        lateralSkipDistance = 6.0;
        minSpeed = 0;
        planningDistance = 10000;
        microPlanDistance = 30;
        carTipMargin = 4.0;
        rollInMargin = 12.0;
        rollInSpeedFactor = 0.25;
        pathDensity = 0.25;
        rollOutDensity = 0.5;
        rollOutNumber = 4;
        horizonDistance = 120;
        minFollowingDistance = 35;
        minDistanceToAvoid = 15;
        maxDistanceToAvoid = 5;
        speedProfileFactor = 1.0;
        smoothingDataWeight = 0.47;
        smoothingSmoothWeight = 0.2;
        smoothingToleranceError = 0.05;

        stopSignStopTime = 2.0;

        additionalBrakingDistance = 10.0;
        verticalSafetyDistance = 0.0;
        horizontalSafetyDistancel = 0.0;

        giveUpDistance = -4;
        nReliableCount = 2;

        enableHeadingSmoothing = false;
        enableSwerving = false;
        enableFollowing = false;
        enableTrafficLightBehavior = false;
        enableLaneChange = false;
        enableStopSignBehavior = false;
        enabTrajectoryVelocities = false;
        minIndicationDistance = 15;
    }
};

class CAR_BASIC_INFO {
public:
    double wheel_base;
    double length;
    double width;

    CAR_BASIC_INFO()
    {
        wheel_base = 2.7;
        length = 4.3;
        width = 1.82;
    }
};

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2points_pow(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define calLength(v) sqrt(v.x *v.x + v.y * v.y)
// #define DEG2RAD M_PI / 180.
#define RAD2DEG_ME 180. / M_PI

int getNextClosePointIndex(const vector<UtilityNS::WayPoint> &trajectory,
                           const UtilityNS::WayPoint &curr_pos,
                           const int &prevIndex = 0);

double calDiffBetweenTwoAngle(const double &a1, const double &a2);

double cast_from_0_to_2PI_Angle(const double &ang);

double cast_from_PI_to_PI_Angle(const double &ang);

double diffBetweenTwoAngle(const double &a1, const double &a2);

void visualLaneInRviz(const vector<UtilityNS::WayPoint> &lane, ros::Publisher pub_testLane);

bool getRelativeInfo(const vector<UtilityNS::WayPoint> &trajectory,
                     const UtilityNS::WayPoint &p,
                     UtilityNS::RelativeInfo &info);

} // namespace UtilityNS



namespace UtilityNS {
/**
 * @description: 获取轨迹上距离当前位置最近的轨迹点（前方） 
 * @param {type} 
 * @return: 
 */
int getNextClosePointIndex(const vector<UtilityNS::WayPoint>& trajectory,
    const UtilityNS::WayPoint& curr_pos,
    const int& prevIndex)
{
    if (trajectory.size() < 2 || prevIndex < 0)
        return 0;
    double dis = 0, min_dis = DBL_MAX;
    int min_index = prevIndex;

    for (int i = prevIndex; i < trajectory.size(); i++) {
        dis = distance2points_pow(trajectory[i].pos, curr_pos.pos);

        if (dis < min_dis) {
            min_index = i;
            min_dis = dis;
        }
    }
    // printf("index %d min_dis %f\n", min_index, min_dis);

    if (min_index < (int)trajectory.size() - 2) {
        UtilityNS::GPSPoint closest, next;
        closest = trajectory[min_index].pos;
        next = trajectory[min_index + 1].pos;
        UtilityNS::GPSPoint v_1(curr_pos.pos.x - closest.x, curr_pos.pos.y - closest.y, 0, 0);
        double length1 = calLength(v_1);
        UtilityNS::GPSPoint v_2(next.x - closest.x, next.y - closest.y, 0, 0);
        double length2 = calLength(v_2);
        double angle = cast_from_0_to_2PI_Angle(acos((v_1.x * v_2.x + v_1.y * v_2.y) / (length1 * length2)));
        if (angle <= M_PI_2)
            min_index = min_index + 1;
    }
    return min_index;
}
/**
 * @description: 显示一条车道线（测试） 
 * @param {type} 
 * @return: 
 */
void visualLaneInRviz(const vector<UtilityNS::WayPoint>& lane, ros::Publisher pub_testLane)
{
    if (pub_testLane.getNumSubscribers() == 0)
        return;
    visualization_msgs::Marker lane_marker;

    lane_marker.header.frame_id = "map";
    lane_marker.header.stamp = ros::Time();
    lane_marker.ns = "test_lane";
    lane_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_marker.action = visualization_msgs::Marker::ADD;
    lane_marker.frame_locked = false;

    lane_marker.scale.x = 0.02;
    lane_marker.frame_locked = false;

    lane_marker.points.clear();
    for (size_t k = 0; k < lane.size(); k++) {
        geometry_msgs::Point wp;
        wp.x = lane[k].pos.x;
        wp.y = lane[k].pos.y;
        wp.z = lane[k].pos.z;
        lane_marker.points.push_back(wp);
    }

    lane_marker.color.b = 0;
    lane_marker.color.g = 0;
    lane_marker.color.r = 1;
    lane_marker.color.a = 1;

    pub_testLane.publish(lane_marker);
}

double calDiffBetweenTwoAngle(const double& a1, const double& a2)
{
    double diff = a1 - a2;
    if (diff < 0)
        diff = a2 - a1;
    if (diff > M_PI)
        diff = 2.0 * M_PI - diff;
    return diff;
}
/**
 * @description:将当前角度转换到0～2pi 
 * @param {type} 
 * @return: 
 */
double cast_from_0_to_2PI_Angle(const double& ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
        angle = fmod(ang, 2.0 * M_PI);
    } else
        angle = ang;

    if (angle < 0) {
        angle = 2.0 * M_PI + angle;
    }
    return angle;
}
/**
 * @description:将当前角度转换到-pi~pi 
 * @param {type} 
 * @return: 
 */
double cast_from_PI_to_PI_Angle(const double& ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
        angle = fmod(ang, 2.0 * M_PI);
    } else
        angle = ang;

    if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    } else if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

double diffBetweenTwoAngle(const double& a1, const double& a2)
{
    double diff = a1 - a2;
    if (diff < 0)
        diff = -diff;
    if (diff > M_PI)
        diff = 2 * M_PI - diff;
    return diff;
}

/**
 * @description: 计算某一个轨迹到某一个点的相对位置
 */
bool getRelativeInfo(const vector<UtilityNS::WayPoint>& trajectory,
    const UtilityNS::WayPoint& p,
    UtilityNS::RelativeInfo& info)
{
    if (trajectory.size() < 2)
        return false;

    UtilityNS::WayPoint p0, p1;
    if (trajectory.size() == 2) {
        p0 = trajectory[0];
        p1 = UtilityNS::WayPoint((p0.pos.x + trajectory[1].pos.x) / 2.0,
            (p0.pos.y + trajectory[1].pos.y) / 2.0,
            (p0.pos.z + trajectory[1].pos.z) / 2.0,
            p0.pos.yaw);
        info.iBack = 0;
        info.iFront = 1;
    } else {
        info.iFront = getNextClosePointIndex(trajectory, p);

        if (info.iFront > 0)
            info.iBack = info.iFront - 1;
        else
            info.iBack = 0;

        if (info.iFront == 0) {
            p0 = trajectory[info.iFront];
            p1 = trajectory[info.iFront + 1];
        } else if (info.iFront > 0 && info.iFront < trajectory.size() - 1) {
            p0 = trajectory[info.iFront - 1];
            p1 = trajectory[info.iFront];
        } else {
            p0 = trajectory[info.iFront - 1];
            p1 = UtilityNS::WayPoint((p0.pos.x + trajectory[info.iFront].pos.x) / 2.0,
                (p0.pos.y + trajectory[info.iFront].pos.y) / 2.0,
                (p0.pos.z + trajectory[info.iFront].pos.z) / 2.0,
                p0.pos.yaw);
        }
    }

    UtilityNS::WayPoint prevWP = p0;
    UtilityNS::Mat3 rotationMat(-p1.pos.yaw);
    UtilityNS::Mat3 translationMat(-p.pos.x, -p.pos.y);
    UtilityNS::Mat3 invRotationMat(p1.pos.yaw);
    UtilityNS::Mat3 invTranslationMat(p.pos.x, p.pos.y);

    p0.pos = translationMat * p0.pos;
    p0.pos = rotationMat * p0.pos;

    p1.pos = translationMat * p1.pos;
    p1.pos = rotationMat * p1.pos;

    double k = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
    info.perp_distance = p1.pos.y - k * p1.pos.x;

    if (isnan(info.perp_distance) || isinf(info.perp_distance))
        info.perp_distance = 0;

    info.to_front_distance = fabs(p1.pos.x);

    info.perp_point = p1;
    info.perp_point.pos.x = 0;
    info.perp_point.pos.y = info.perp_distance;

    info.perp_point.pos = invRotationMat * info.perp_point.pos;
    info.perp_point.pos = invTranslationMat * info.perp_point.pos;

    info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);
    info.angle_diff = UtilityNS::diffBetweenTwoAngle(p1.pos.yaw, p.pos.yaw) * RAD2DEG_ME;

    info.direct_distance = hypot(p1.pos.y - p.pos.y, p1.pos.x - p.pos.x);
    return true;
}
} // namespace UtilityNS







namespace RolloutGeneratorNS {

#define LANE_CHANGE_SPEED_FACTOR 0.5

    class RolloutGenerator {
    private:
        UtilityNS::WayPoint current_pose;
        UtilityNS::WayPoint init_pose;
        vector<vector<UtilityNS::WayPoint>> globalPaths;
        vector<UtilityNS::WayPoint> centralTrajectorySmoothed;
        vector<vector<UtilityNS::WayPoint>> globalPathSections;
        vector<vector<vector<UtilityNS::WayPoint>>> rollOuts;

        bool currentPose_flag;

        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        ros::Publisher pub_localTrajectoriesRviz;
        ros::Publisher pub_testLane;

        ros::Publisher pub_global_path;
        ros::Publisher pub_center_path;
        ros::Publisher pub_remaining_path;

        UtilityNS::PlanningParams PlanningParams;
        UtilityNS::CAR_BASIC_INFO CarInfo;
        double speed;

        void getGlobalPlannerPath_cb(nav_msgs::Path msg);

        void msgLane2LocalLane(nav_msgs::Path msg_path, vector<UtilityNS::WayPoint>& path);

        double calcAngleAndCost(vector<UtilityNS::WayPoint>& path);

        void extractPartFromTrajectory(const vector<UtilityNS::WayPoint>& originalPath,
            const UtilityNS::WayPoint& currnetPos,
            const double& minDistance,
            const double& waypointDensity,
            vector<UtilityNS::WayPoint>& extractedPath);

        void fixPathDensity(vector<UtilityNS::WayPoint>& path, const double& pathDensity);
        nav_msgs::Path fixPathDensity(nav_msgs::Path path);

        void trajectoryToMarkers(const vector<vector<vector<UtilityNS::WayPoint>>>& paths, visualization_msgs::MarkerArray& markerArray);

        void generateRunoffTrajectory(const vector<vector<UtilityNS::WayPoint>>& referencePaths,
            const UtilityNS::WayPoint& carPos, const double& speed, const double& microPlanDistance,
            const double& carTipMargin, const double& rollInMargin, const double& rollInSpeedFactor,
            const double& pathDensity, const double& rollOutDensity, const int& rollOutNumber,
            const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance,
            vector<vector<vector<UtilityNS::WayPoint>>>& rollOutsPaths,
            vector<UtilityNS::WayPoint>& sampledPoints_debug);

        void calculateRollInTrajectories(const UtilityNS::WayPoint& carPos, const double& speed, const vector<UtilityNS::WayPoint>& originalCenter,
            int& start_index, int& end_index, vector<double>& end_laterals,
            vector<vector<UtilityNS::WayPoint>>& rollInPaths, const double& max_roll_distance,
            const double& carTipMargin, const double& rollInMargin, const double& rollInSpeedFactor,
            const double& pathDensity, const double& rollOutDensity, const int& rollOutNumber,
            const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance,
            vector<UtilityNS::WayPoint>& sampledPoints);

        void smoothPath(vector<UtilityNS::WayPoint>& path, double weight_data, double weight_smooth, double tolerance);

    public:
        RolloutGenerator();

        ~RolloutGenerator();

        nav_msgs::Path getCenterPath(vector<UtilityNS::WayPoint> pathIn);
        vector<nav_msgs::Path> getOtherpaths(vector<vector<UtilityNS::WayPoint>> pathsIn);
        nav_msgs::Path getRemainingPath(nav_msgs::Path pathIn, nav_msgs::Path centerPath);
        nav_msgs::Path calculatePathYaw(nav_msgs::Path pathIn);

        void run(tf::StampedTransform transform, nav_msgs::Path pathMsg, nav_msgs::Path& fixedGlobal, nav_msgs::Path& centerPath, nav_msgs::Path& remainingPath, vector<nav_msgs::Path>& alternativePaths);

        void initROS();

        tf::StampedTransform transform;
    };
} // namespace RolloutGeneratorNS

#endif //ROLLOUT_GENERATOR_H


namespace RolloutGeneratorNS {
RolloutGenerator::RolloutGenerator()
    : nh_private("")
{
    initROS();
    currentPose_flag = false;
}

RolloutGenerator::~RolloutGenerator()
{
}

void RolloutGenerator::initROS()
{
    nh_private.param<double>("roboat_planning/_samplingTipMargin", PlanningParams.carTipMargin, 1);
    nh_private.param<double>("roboat_planning/_samplingOutMargin", PlanningParams.rollInMargin, 1);
    nh_private.param<double>("roboat_planning/_samplingSpeedFactor", PlanningParams.rollInSpeedFactor, 0.25);

    nh_private.param<double>("roboat_planning/_pathResolution", PlanningParams.pathDensity, 0.1);
    nh_private.param<double>("roboat_planning/_maxPathDistance", PlanningParams.microPlanDistance, 10.0);

    nh_private.param<double>("roboat_planning/_rollOutDensity", PlanningParams.rollOutDensity, 0.1);
    nh_private.param<int>("roboat_planning/_rollOutNumber", PlanningParams.rollOutNumber, 20);

    nh_private.param<double>("roboat_planning/_smoothingDataWeight", PlanningParams.smoothingDataWeight, 0.45);
    nh_private.param<double>("roboat_planning/_smoothingSmoothWeight", PlanningParams.smoothingSmoothWeight, 0.4);
    nh_private.param<double>("roboat_planning/_smoothingToleranceError", PlanningParams.smoothingToleranceError, 0.05);

    nh_private.param<double>("roboat_planning/_speedProfileFactor", PlanningParams.speedProfileFactor, 1.2);

    pub_localTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("planning/op/rollouts_all", 1);
    pub_testLane = nh.advertise<visualization_msgs::Marker>("planning/op/rollouts_center", 1);

    pub_global_path = nh.advertise<nav_msgs::Path>("planning/op/global_path", 1);
    pub_center_path = nh.advertise<nav_msgs::Path>("planning/op/center_path", 1);
    pub_remaining_path = nh.advertise<nav_msgs::Path>("planning/op/remaining_path", 1);

    speed = 1;
}

/**
 * @description: 主循环函数
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::run(tf::StampedTransform transform, nav_msgs::Path pathMsg, nav_msgs::Path& fixedGlobal, nav_msgs::Path& centerPath, nav_msgs::Path& remainingPath, vector<nav_msgs::Path>& alternativePaths)
{
    current_pose.pos.x = transform.getOrigin().x();
    current_pose.pos.y = transform.getOrigin().y();
    current_pose.pos.z = transform.getOrigin().z();
    current_pose.pos.yaw = tf::getYaw(transform.getRotation());

    getGlobalPlannerPath_cb(pathMsg);

    if (globalPaths.size() == 0)
        return;

    globalPathSections.clear();
    for (size_t i = 0; i < globalPaths.size(); i++) {
        centralTrajectorySmoothed.clear();
        extractPartFromTrajectory(globalPaths[i], current_pose, PlanningParams.microPlanDistance,
            PlanningParams.pathDensity, centralTrajectorySmoothed);
        globalPathSections.push_back(centralTrajectorySmoothed);
    }
    vector<UtilityNS::WayPoint> sampled_points;
    generateRunoffTrajectory(globalPathSections,
        current_pose,
        speed,
        PlanningParams.microPlanDistance,
        PlanningParams.carTipMargin,
        PlanningParams.rollInMargin,
        PlanningParams.speedProfileFactor,
        PlanningParams.pathDensity,
        PlanningParams.rollOutDensity,
        PlanningParams.rollOutNumber,
        PlanningParams.smoothingDataWeight,
        PlanningParams.smoothingSmoothWeight,
        PlanningParams.smoothingToleranceError,
        rollOuts,
        sampled_points);

    // pub lcoal rollouts in rviz
    if (pub_localTrajectoriesRviz.getNumSubscribers() != 0)
    {
        visualization_msgs::MarkerArray marker_rollouts;
        trajectoryToMarkers(rollOuts, marker_rollouts);
        pub_localTrajectoriesRviz.publish(marker_rollouts);
    }

    // convert to ros Path message
    fixedGlobal      = fixPathDensity(pathMsg);
    centerPath       = getCenterPath(centralTrajectorySmoothed);
    remainingPath    = getRemainingPath(fixedGlobal, centerPath);
    alternativePaths = getOtherpaths(rollOuts[0]);

    // re-calculate orientation
    // fixedGlobal = calculatePathYaw(fixedGlobal);
    // centerPath = calculatePathYaw(centerPath);
    // remainingPath = calculatePathYaw(remainingPath);
    // for (int i = 0; i < alternativePaths.size(); ++i)
    //     alternativePaths[i] = calculatePathYaw(alternativePaths[i]);

    if (pub_global_path.getNumSubscribers() != 0)
        pub_global_path.publish(fixedGlobal);
    if (pub_center_path.getNumSubscribers() != 0)
        pub_center_path.publish(centerPath);
    if (pub_remaining_path.getNumSubscribers() != 0)
        pub_remaining_path.publish(remainingPath);
}

nav_msgs::Path RolloutGenerator::calculatePathYaw(nav_msgs::Path pathIn)
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

nav_msgs::Path RolloutGenerator::getRemainingPath(nav_msgs::Path pathIn, nav_msgs::Path centerPath)
{
    nav_msgs::Path pathOut = pathIn;

    geometry_msgs::Point lastPose = centerPath.poses.back().pose.position;

    for (int i = 0; i < pathIn.poses.size(); ++i)
    {
        geometry_msgs::Point curPose = pathIn.poses[i].pose.position;

        if (sqrt((lastPose.x-curPose.x)*(lastPose.x-curPose.x) + (lastPose.y-curPose.y)*(lastPose.y-curPose.y)) <= PlanningParams.pathDensity)
        {
            if (i + 1 <= pathOut.poses.size() -1)
                pathOut.poses.erase(pathOut.poses.begin(), pathOut.poses.begin() + i + 1);
            else
                pathOut.poses.erase(pathOut.poses.begin(), pathOut.poses.begin() + i);

            break;
        }
    }

    // if (pathOut.poses.size() == 1)
    //     pathOut.poses.clear();

    return pathOut;
}

nav_msgs::Path RolloutGenerator::getCenterPath(vector<UtilityNS::WayPoint> pathIn)
{
    nav_msgs::Path pathOut;
    pathOut.header.stamp = ros::Time();
    pathOut.header.frame_id = "map";

    for (int i = 0; i < pathIn.size(); ++i)
    {
        geometry_msgs::PoseStamped poseCur;
        poseCur.header.stamp = ros::Time();
        poseCur.header.frame_id = "map";

        poseCur.pose.position.x = pathIn[i].pos.x;
        poseCur.pose.position.y = pathIn[i].pos.y;
        poseCur.pose.position.z = pathIn[i].pos.z;

        poseCur.pose.orientation = tf::createQuaternionMsgFromYaw(pathIn[i].pos.yaw);

        pathOut.poses.push_back(poseCur);
    }

    return pathOut;
}

vector<nav_msgs::Path> RolloutGenerator::getOtherpaths(vector<vector<UtilityNS::WayPoint>> pathsIn)
{
    vector<nav_msgs::Path> pathVecOut;

    nav_msgs::Path pathOut;
    pathOut.header.stamp = ros::Time();
    pathOut.header.frame_id = "map";

    for (int i = 0; i < pathsIn.size(); ++i)
    {
        pathOut.poses.clear();
        for (int j = 0; j < pathsIn[i].size(); ++j)
        {
            geometry_msgs::PoseStamped poseCur;
            poseCur.header.stamp = ros::Time();
            poseCur.header.frame_id = "map";

            poseCur.pose.position.x = pathsIn[i][j].pos.x;
            poseCur.pose.position.y = pathsIn[i][j].pos.y;
            poseCur.pose.position.z = pathsIn[i][j].pos.z;

            poseCur.pose.orientation = tf::createQuaternionMsgFromYaw(pathsIn[i][j].pos.yaw);

            pathOut.poses.push_back(poseCur);
        }
        pathVecOut.push_back(pathOut);
    }

    return pathVecOut;
}

void RolloutGenerator::trajectoryToMarkers(const vector<vector<vector<UtilityNS::WayPoint>>>& paths, visualization_msgs::MarkerArray& markerArray)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "rollouts";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.01;
    lane_waypoint_marker.frame_locked = false;
    lane_waypoint_marker.color.r = 0.0; lane_waypoint_marker.color.g = 1.0; lane_waypoint_marker.color.b = 0.0;
    lane_waypoint_marker.color.a = 0.7;

    for (size_t i = 0; i < paths.size(); i++) {
        for (size_t k = 0; k < paths[i].size(); k++) {
            lane_waypoint_marker.points.clear();
            lane_waypoint_marker.id = i * 10 + k;
            for (size_t m = 0; m < paths[i][k].size(); m++) {
                geometry_msgs::Point wp;
                wp.x = paths[i][k][m].pos.x;
                wp.y = paths[i][k][m].pos.y;
                wp.z = paths[i][k][m].pos.z;
                lane_waypoint_marker.points.push_back(wp);
            }
            markerArray.markers.push_back(lane_waypoint_marker);
        }
    }
    // nodes visualization
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "map";
    markerNode.header.stamp = ros::Time();
    markerNode.ns = "nodes";
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.id = 999999;
    markerNode.scale.x = 0.02;
    markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
    markerNode.color.a = 0.7;

    for (size_t i = 0; i < paths.size(); i++) {
        for (size_t k = 0; k < paths[i].size(); k++) {
            for (size_t m = 0; m < paths[i][k].size(); m++) {
                geometry_msgs::Point wp;
                wp.x = paths[i][k][m].pos.x;
                wp.y = paths[i][k][m].pos.y;
                wp.z = paths[i][k][m].pos.z;
                markerNode.points.push_back(wp);
            }
        }
    }
    markerArray.markers.push_back(markerNode);
}
/**
 * @description:生成候选局部规划路径rollouts 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::generateRunoffTrajectory(const vector<vector<UtilityNS::WayPoint>>& referencePaths,
    const UtilityNS::WayPoint& carPos,
    const double& speed,
    const double& microPlanDistance,
    const double& carTipMargin,
    const double& rollInMargin,
    const double& rollInSpeedFactor,
    const double& pathDensity,
    const double& rollOutDensity,
    const int& rollOutNumber,
    const double& SmoothDataWeight,
    const double& SmoothWeight,
    const double& SmoothTolerance,
    vector<vector<vector<UtilityNS::WayPoint>>>& rollOutsPaths,
    vector<UtilityNS::WayPoint>& sampledPoints_debug)
{

    if (referencePaths.size() == 0)
        return;
    if (microPlanDistance <= 0)
        return;
    rollOutsPaths.clear();
    sampledPoints_debug.clear(); //for visualization only

    for (unsigned int i = 0; i < referencePaths.size(); i++) {
        vector<vector<UtilityNS::WayPoint>> local_rollOutPaths;
        int s_index = 0, e_index = 0;
        vector<double> e_distances;

        if (referencePaths.at(i).size() > 0) {
            calculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
                local_rollOutPaths, microPlanDistance, carTipMargin, rollInMargin,
                rollInSpeedFactor, pathDensity, rollOutDensity, rollOutNumber,
                SmoothDataWeight, SmoothWeight, SmoothTolerance, sampledPoints_debug);
        }
        rollOutsPaths.push_back(local_rollOutPaths);
    }
}
/**
 * @description: 由中心轨迹的采样点计算候选rollouts的采样点并平滑
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::calculateRollInTrajectories(const UtilityNS::WayPoint& carPos,
    const double& speed,
    const vector<UtilityNS::WayPoint>& originalCenter,
    int& start_index,
    int& end_index,
    vector<double>& end_laterals,
    vector<vector<UtilityNS::WayPoint>>& rollInPaths,
    const double& max_roll_distance,
    const double& carTipMargin,
    const double& rollInMargin,
    const double& rollInSpeedFactor,
    const double& pathDensity,
    const double& rollOutDensity,
    const int& rollOutNumber,
    const double& SmoothDataWeight,
    const double& SmoothWeight,
    const double& SmoothTolerance,
    vector<UtilityNS::WayPoint>& sampledPoints)
{
    UtilityNS::WayPoint p;

    int iLimitIndex = (carTipMargin / 0.3) / pathDensity;
    if (iLimitIndex >= originalCenter.size())
        iLimitIndex = originalCenter.size() - 1;

    //Get Closest Index
    UtilityNS::RelativeInfo info;
    UtilityNS::getRelativeInfo(originalCenter, carPos, info);

    double remaining_distance = 0;
    int close_index = info.iBack;
    for (unsigned int i = close_index; i < originalCenter.size() - 1; i++) {
        if (i > 0)
            remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
    }

    double initial_roll_in_distance = info.perp_distance; //GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);

    vector<UtilityNS::WayPoint> RollOutStratPath;

    //calculate the starting index
    double d_limit = 0;
    unsigned int far_index = close_index;

    //calculate end index
    double start_distance = rollInSpeedFactor * speed + rollInMargin;
    if (start_distance > remaining_distance)
        start_distance = remaining_distance;

    d_limit = 0;
    for (unsigned int i = close_index; i < originalCenter.size(); i++) {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);

        if (d_limit >= start_distance) {
            far_index = i;
            break;
        }
    }

    int centralTrajectoryIndex = rollOutNumber / 2;
    vector<double> end_distance_list;
    for (int i = 0; i < rollOutNumber + 1; i++) {
        double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);
        end_distance_list.push_back(end_roll_in_distance);
    }

    start_index = close_index;
    end_index = far_index; // end_index是第二个阶段结尾的点坐标
    end_laterals = end_distance_list;

    //calculate the actual calculation starting index
    d_limit = 0;
    unsigned int smoothing_start_index = start_index;
    unsigned int smoothing_end_index = end_index;

    for (unsigned int i = smoothing_start_index; i < originalCenter.size(); i++) {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_start_index++; // 这个是一个阶段结尾的点下标
    }

    d_limit = 0;
    for (unsigned int i = end_index; i < originalCenter.size(); i++) {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_end_index++;
    }
    // printf("%s %d %d %d \n", "----------------", int(smoothing_start_index), int(end_index), int(originalCenter.size()));

    int nSteps = end_index - smoothing_start_index;

    rollInPaths.clear();
    vector<double> inc_list;
    vector<double> inc_list_inc;
    for (int i = 0; i < rollOutNumber + 1; i++) {
        rollInPaths.push_back(vector<UtilityNS::WayPoint>());
        double diff = end_laterals.at(i) - initial_roll_in_distance;
        inc_list.push_back(diff / (double)nSteps);
        inc_list_inc.push_back(0);
    }

    // cout << initial_roll_in_distance << endl;

    vector<vector<UtilityNS::WayPoint>> execluded_from_smoothing;
    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        execluded_from_smoothing.push_back(vector<UtilityNS::WayPoint>());

    // start section
    //Insert First strait points within the tip of the car range
    for (unsigned int j = start_index; j < smoothing_start_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            if (j < iLimitIndex)
                execluded_from_smoothing.at(i).push_back(p);
            else
                rollInPaths.at(i).push_back(p);
        }
    }

    // transition section
    for (unsigned int j = smoothing_start_index; j < end_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            inc_list_inc[i] += inc_list[i];
            double d = inc_list_inc[i];
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2) - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2) - d * sin(p.pos.yaw + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);
        }
    }
    
    //Insert last strait points to make better smoothing
    for (unsigned int j = end_index; j < smoothing_end_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);


            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;
            rollInPaths.at(i).push_back(p);
        }
    }


    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

    // middle smooth section
    d_limit = 0;
    for (unsigned int j = smoothing_end_index; j < originalCenter.size(); j++) {
        if (j > 0)
            d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j - 1).pos);

        if (d_limit > max_roll_distance)
            break;
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollInPaths.size(); i++) {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);
        }
    }


    // tixiao's processing
    int path_length = rollInPaths[0].size();

    // the last section
    int seg_num_2 = smoothing_start_index - start_index;
    int j_start = path_length - seg_num_2;
    for (int j = path_length - seg_num_2; j < path_length; j++)
    {
        int origin_path_index = min(int(j + start_index), (int)originalCenter.size() - 1);
        p = originalCenter.at(origin_path_index);
        double original_speed = p.v;
        for (int i = 0; i < rollOutNumber + 1; i++)
        {
            p.pos.x = originalCenter.at(origin_path_index).pos.x;
            p.pos.y = originalCenter.at(origin_path_index).pos.y;

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths[i][j] = p;
        }
    }

    // the second to last section
    // reset offset vector
    inc_list.clear();
    inc_list_inc.clear();
    for (int i = 0; i < rollOutNumber + 1; i++) {
        double diff = end_laterals.at(i);
        inc_list.push_back(diff / (double)nSteps);
        inc_list_inc.push_back(0);
    }
    for (int j = smoothing_start_index; j < end_index; j++) {
        for (int i = 0; i < rollOutNumber + 1; i++) {
            inc_list_inc[i] += inc_list[i];
        }
    }

    int seg_num_1 = end_index - smoothing_start_index;
    if (seg_num_1 > 0)
    {
        int j_start = max(path_length - seg_num_2 - seg_num_1, int(smoothing_start_index - start_index));
        for (int j = j_start; j < path_length - seg_num_2; j++)
        {
            int origin_path_index = min(int(j + start_index), (int)originalCenter.size() - 1);
            p = originalCenter.at(origin_path_index);
            double original_speed = p.v;
            for (int i = 0; i < rollOutNumber + 1; i++)
            {
                inc_list_inc[i] -= inc_list[i];
                double d = inc_list_inc[i];
                p.pos.x = originalCenter.at(origin_path_index).pos.x - d * cos(p.pos.yaw + M_PI_2);
                p.pos.y = originalCenter.at(origin_path_index).pos.y - d * sin(p.pos.yaw + M_PI_2);

                if (i != centralTrajectoryIndex)
                    p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
                else
                    p.v = original_speed;

                rollInPaths[i][j] = p;
            }
        }
    }

    // smooth path
    for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
        smoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
    }
}

/**
 * @description: 平滑生成的曲线 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::smoothPath(vector<UtilityNS::WayPoint>& path, double weight_data,
    double weight_smooth, double tolerance)
{
    if (path.size() <= 2)
        return;

    const vector<UtilityNS::WayPoint>& path_in = path;
    vector<UtilityNS::WayPoint> smoothPath_out = path_in;

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    int size = path_in.size();

    while (change >= tolerance) {
        change = 0.0;
        for (int i = 1; i < size - 1; i++) {
            xtemp = smoothPath_out[i].pos.x;
            ytemp = smoothPath_out[i].pos.y;

            smoothPath_out[i].pos.x += weight_data * (path_in[i].pos.x - smoothPath_out[i].pos.x);
            smoothPath_out[i].pos.y += weight_data * (path_in[i].pos.y - smoothPath_out[i].pos.y);

            smoothPath_out[i].pos.x += weight_smooth * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x - (2.0 * smoothPath_out[i].pos.x));
            smoothPath_out[i].pos.y += weight_smooth * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y - (2.0 * smoothPath_out[i].pos.y));

            change += fabs(xtemp - smoothPath_out[i].pos.x);
            change += fabs(ytemp - smoothPath_out[i].pos.y);
        }
        nIterations++;
    }
    path = smoothPath_out;
}

// nav_msgs::Path smoothPath(nav_msgs::Path path)
// {
//     if (path.poses.size() <= 2)
//         return path;

//     double weight_data = 0.45;
//     double weight_smooth = 0.4;
//     double tolerance = 0.05;

//     nav_msgs::Path smoothPath_out = path;

//     double change = tolerance;
//     double xtemp, ytemp;
//     int nIterations = 0;

//     int size = path.poses.size();

//     while (change >= tolerance) {
//         change = 0.0;
//         for (int i = 1; i < size - 1; i++) {
//             xtemp = smoothPath_out.poses[i].pose.position.x;
//             ytemp = smoothPath_out.poses[i].pose.position.y;

//             smoothPath_out.poses[i].pose.position.x += weight_data * (path.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
//             smoothPath_out.poses[i].pose.position.y += weight_data * (path.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);

//             smoothPath_out.poses[i].pose.position.x += weight_smooth * (smoothPath_out.poses[i-1].pose.position.x + smoothPath_out.poses[i+1].pose.position.x - (2.0 * smoothPath_out.poses[i].pose.position.x));
//             smoothPath_out.poses[i].pose.position.y += weight_smooth * (smoothPath_out.poses[i-1].pose.position.y + smoothPath_out.poses[i+1].pose.position.y - (2.0 * smoothPath_out.poses[i].pose.position.y));

//             change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
//             change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
//         }
//         nIterations++;
//     }

//     return smoothPath_out;
// }


/**
 * @description: 从全局路径截取最大局部规划距离 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::extractPartFromTrajectory(const vector<UtilityNS::WayPoint>& originalPath,
    const UtilityNS::WayPoint& currentPos,
    const double& minDistance,
    const double& waypointDensity,
    vector<UtilityNS::WayPoint>& extractedPath)
{
    if (originalPath.size() < 2)
        return;
    extractedPath.clear();
    int close_index = UtilityNS::getNextClosePointIndex(originalPath, currentPos);
    // printf("%d\n", close_index);

    double dis = 0;
    if (close_index >= originalPath.size() - 1)
        close_index = originalPath.size() - 2;

    for (int i = close_index; i >= 0; i--) {
        extractedPath.insert(extractedPath.begin(), originalPath[i]);
        if (i < originalPath.size())
            dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
        // printf("%f\n", dis);
        if (dis > 2)
            break;
    }
    dis = 0;
    for (int i = close_index + 1; i < (int)originalPath.size(); i++) {
        extractedPath.push_back(originalPath[i]);
        if (i > 0)
            dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
        if (dis > minDistance)
            break;
    }
    if (extractedPath.size() < 2) {
        cout << endl
                  << "[loacal_planner_node] Extracted Rollout Path is too Small, Size = " << extractedPath.size() << endl;
        return;
    }
    UtilityNS::visualLaneInRviz(extractedPath, pub_testLane);
    fixPathDensity(extractedPath, waypointDensity);
    calcAngleAndCost(extractedPath);
}

void RolloutGenerator::fixPathDensity(vector<UtilityNS::WayPoint>& path, const double& pathDensity)
{
    if (path.size() == 0 || pathDensity == 0)
        return;
    double dis = 0, ang = 0;
    double margin = pathDensity * 0.01;
    double remaining = 0;
    int nPoints = 0;
    vector<UtilityNS::WayPoint> fixedPath;
    fixedPath.push_back(path[0]);
    size_t start = 0, next = 1;
    while (next < path.size()) {
        dis += hypot(path[next].pos.x - path[next - 1].pos.x, path[next].pos.y - path[next - 1].pos.y) + remaining;
        ang = atan2(path[next].pos.y - path[start].pos.y, path[next].pos.x - path[start].pos.x);

        if (dis < pathDensity - margin) {
            next++;
            remaining = 0;
        } else if (dis > (pathDensity + margin)) {
            UtilityNS::WayPoint point_start = path[start];
            nPoints = dis / pathDensity;
            for (int j = 0; j < nPoints; j++) {
                point_start.pos.x = point_start.pos.x + pathDensity * cos(ang);
                point_start.pos.y = point_start.pos.y + pathDensity * sin(ang);
                fixedPath.push_back(point_start);
            }
            remaining = dis - nPoints * pathDensity;
            start++;
            path[start].pos = point_start.pos;
            dis = 0;
            next++;
        } else {
            dis = 0;
            remaining = 0;
            fixedPath.push_back(path[next]);
            next++;
            start = next - 1;
        }
    }
    path = fixedPath;
}

nav_msgs::Path RolloutGenerator::fixPathDensity(nav_msgs::Path path)
{
    double _pathResolution = PlanningParams.pathDensity;
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


void RolloutGenerator::getGlobalPlannerPath_cb(nav_msgs::Path msg)
{
    if (msg.poses.size() > 0) {
        globalPaths.clear();
        vector<UtilityNS::WayPoint> single_path;
        msgLane2LocalLane(msg, single_path);
        calcAngleAndCost(single_path);
        globalPaths.push_back(single_path);
    }
}

void RolloutGenerator::msgLane2LocalLane(nav_msgs::Path msg_path, vector<UtilityNS::WayPoint>& path)
{
    path.clear();
    for (size_t i = 0; i < msg_path.poses.size(); i++) {
        UtilityNS::WayPoint wp;
        wp.pos.x = msg_path.poses[i].pose.position.x;
        wp.pos.y = msg_path.poses[i].pose.position.y;
        wp.pos.z = msg_path.poses[i].pose.position.z;
        wp.pos.yaw = tf::getYaw(msg_path.poses[i].pose.orientation);
        wp.v = 1;//msg_path.waypoints.at(i).speed_limit;
        // wp.laneId = msg_path.waypoints.at(i).lane_id;
        path.push_back(wp);
    }
}

double RolloutGenerator::calcAngleAndCost(vector<UtilityNS::WayPoint>& path)
{
    if (path.size() < 2)
        return 0;
    if (path.size() == 2) {
        path[0].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = 0;
        path[1].pos.yaw = path[0].pos.yaw;
        path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
        return path[1].cost;
    }

    path[0].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
    path[0].cost = 0;

    for (int j = 1; j < path.size() - 1; j++) {
        path[j].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
    }

    int j = (int)path.size() - 1;
    path[j].pos.yaw = path[j - 1].pos.yaw;
    path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);

    return path[j].cost;
}
} // namespace RolloutGeneratorNS