#include <pluginlib/class_list_macros.h>
#include "planner/lex_planner.h"
#include <unistd.h>

//register this planner as a BaseLEXPlanner plugin
PLUGINLIB_EXPORT_CLASS(lex_planner::LEXPlanner, nav_core::BaseGlobalPlanner)
 
	
namespace lex_planner {

	LEXPlanner::LEXPlanner (){}

	LEXPlanner::LEXPlanner(const std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, costmap_ros);
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize the Plugin
	void LEXPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		// initialize subscriptions
		subPath = nh.subscribe("planning/planning/execute_path", 5, &LEXPlanner::pathHandler, this);
		// subPath = nh.subscribe("planning/server/path_blueprint_smooth", 5, &LEXPlanner::pathHandler, this);
		// visualize twist command
		subTwistCommand1 = nh.subscribe<nav_msgs::Path>("/move_base/TrajectoryPlannerROS/local_plan", 5, &LEXPlanner::twistCommandHandler, this);
		subTwistCommand2 = nh.subscribe<nav_msgs::Path>("/move_base/DWAPlannerROS/local_plan", 5, &LEXPlanner::twistCommandHandler, this);
		// Publisher
        pubTwistCommand = nh.advertise<nav_msgs::Path>("/twist_command", 5);
	}

	// visualize twist command
	void LEXPlanner::twistCommandHandler(const nav_msgs::Path::ConstPtr& pathMsg){

		try{ listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ return; }

        nav_msgs::Path outTwist = *pathMsg;

        for (int i = 0; i < outTwist.poses.size(); ++i)
            outTwist.poses[i].pose.position.z = transform.getOrigin().z() + 1.0;

        pubTwistCommand.publish(outTwist);
    }

    // receive path from prm global planner
	void LEXPlanner::pathHandler(const nav_msgs::Path::ConstPtr& pathMsg){
		// std::lock_guard<std::mutex> lock(mtx);
		// if the planner couldn't find a feasible path, pose size should be 0
		globalPath = *pathMsg;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////
	bool LEXPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan)
	{
		// 2. if the planner couldn't find a feasible path, pose size should be 0
		if (globalPath.poses.size() == 0){
			return false;
		}
		ROS_INFO("A Valid Path Received!");

		// 3. Extract Path
		geometry_msgs::PoseStamped this_pos = goal;
		for (int i = 0; i < globalPath.poses.size(); ++i){
			this_pos = globalPath.poses[i];
			plan.push_back(this_pos);
		}

		plan.back().pose.orientation = goal.pose.orientation;

		// globalPath.poses.clear();

		return true; 
	}

};