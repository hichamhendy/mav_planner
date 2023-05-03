/*
Path Planner is designed to follow the pattern of Singelton
*/
#pragma once

#define __PACKAGE_NAME__ "MAV Planner"

#include <tf2_eigen/tf2_eigen.h>

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <dynamic_reconfigure/server.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
// #include <ompl/geometric/PathGeometric.h>

#include <ompl/config.h>
#include <ompl/tools/config/MagicConstants.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

#include <boost/thread.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include <manager_msgs/CommandTarget.h>

#include <Eigen/Dense>

// #include <fcl/config.h>
// #include <fcl/fcl.h>
// #include <fcl/geometry/collision_geometry.h>
// #include <fcl/geometry/octree/octree.h>

namespace ob = ompl::base; // This namespace contains sampling based planning routines shared by both planning under geometric constraints (geometric) and planning under differential constraints (dynamic) More...
namespace og = ompl::geometric; // This namespace contains code that is specific to planning under geometric constraints. 


/* // Memory leak fix (not necessary)
octomap::OcTree* tree_oct_ = NULL;
octomap::AbstractOcTree* msg_octree_ = NULL; */


namespace nut
{

class Planner
{
private:
	ros::NodeHandle nh_; // private node
	ros::Time start_time_;
  	ros::Time last_wp_time_;
	ros::Subscriber octree_sub_;
	ros::Subscriber ekf2_sub_;
	ros::Subscriber goal_sub_;
	ros::Publisher path_pub_;
	ros::Publisher waypoint_pub_;
	ros::Publisher path_array_pub_;
	ros::Publisher waypoint_marker_pub_;
	ros::ServiceServer goal_setting_server_;

	double mapupdate_dt_;

	mutable std::mutex octomap_mutex_, collide_lock;  // mutex for thread safe implementation (asynchronous callbacks are allowed)
	boost::mutex update_cell_sets_lock_, plan_mutex_, setStart_lock_;

	/* OMPL data */
	// Notice all must be shared pointers which is in ob already defined. 
	// Revise -> https://ompl.kavrakilab.org/geometricPlanningSE3.html
	ob::StateSpacePtr space_3D_; // identify the space we are planning in: SE(3) in which planning can be performed revise -> https://ompl.kavrakilab.org/classompl_1_1base_1_1StateSpace.html#afd88dea5b056dae47158f22edb21e562
	ob::SpaceInformationPtr si_SE3_; 	// construct an instance of  space information from this state space
	ob::ProblemDefinitionPtr problem_def_; 	// problem instance pointer


	// goal state
	double former_goal_[3];

	// og::PathGeometric* path_bsplining_ = NULL; // https://ompl.kavrakilab.org/classompl_1_1geometric_1_1PathGeometric.html#details
	std::shared_ptr<og::PathGeometric> path_bsplining_;

	// contains the geometry and the transform information.
	std::shared_ptr<fcl::CollisionGeometry> copter_; // interest object
	std::shared_ptr<fcl::CollisionGeometry> tree_;  // configuration space

	// std::shared_ptr<fcl::CollisionObject<double>> copter_; // interest object
	// std::shared_ptr<fcl::CollisionObject<double>> tree_;  // configuration space 

	octomap::OcTree* tree_oct_ = NULL;  // boost::optional<octomap::OcTree> octo_tree_;
	octomap::AbstractOcTree* msg_octree_ = NULL; 

	bool initialize_, goal_reached_, replan_flag_, goal_reachable_;

	size_t no_waypoints;

	// vector to save the bspline in
	std::vector<Eigen::Vector3d> bspline_pos_, waypoints_; // by the way Eigen::VectorXd  works as well.
	std::vector<double> bspline_attit_;
	std::vector<std::pair<Eigen::Vector3d, double>> pose_set_; 
	std::vector<std::tuple<double, double, double>>  path_bspline_;

	std::thread goal_setting_thread_;

	Eigen::Vector3d current_position_, goal_;

	double elevation_goal_;

	static Planner * planner_instance_;
    static std::mutex mutex_;

	/**
	 * @brief
	*/
	void updateOctotree(std::shared_ptr<fcl::CollisionGeometry> binary_msg);

	/**
	 * @brief collision check function
	 * @return is a state in SE3 occupied or not
	*/
	bool isStateOccupied(const ob::State *state);

	/***
	 * @brief the function should provide a validation for the quad motion
	 * @note this function assumes state_1 is valid
	 * @return is the suggested motion valid regarding the kinodynamics
	*/
	bool isMotionValid(const ob::State *state_1, const ob::State *state_2);

	/**
	 * @brief This method returns an optimization objective which corresponds to optimizing path length.
	 *  in configuration space of computed paths. (https://ompl.kavrakilab.org/classompl_1_1base_1_1PathLengthOptimizationObjective.html)
	 * @note revise  https://ompl.kavrakilab.org/classompl_1_1base_1_1OptimizationObjective.html
	 * @return a structure representing the optimization objective to use
	// for optimal motion planning.
	*/
	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& space_3D_info);

	/**
	 * @brief This method returns an objective which
		attempts to minimize th cost of the function.
	 * @note revise https://ompl.kavrakilab.org/classompl_1_1base_1_1OptimizationObjective.html
	 * @param state space information
	 * @return a structure representing the optimization objective to use
	// for optimal motion planning. 
	*/
	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

	/**
	 * @brief In some cases you might be interested in optimal planning under more than one objective. 
	 * For instance, we might want to specify some balance between path clearance and path length
	 * This results in an optimization objective where path cost is equivalent to summing up each of the individual objectives' path costs.
	 * @note guidlines https://ompl.kavrakilab.org/optimizationObjectivesTutorial.html
	 * 
	 */
	ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si);

	/**
	 * @brief Maximize minimum clearance
	 * @note guidlines -> https://ompl.kavrakilab.org/optimizationObjectivesTutorial.html
	 */
	ob::OptimizationObjectivePtr getMaxMinClearanceObjective(const ob::SpaceInformationPtr& si);

protected:
	/**
	 * @brief Constructs the object
	 * @note made protected (also private possible) to prevent other objects from using new operator
	 * 
	 */
	Planner(const ros::NodeHandle& nh);

	/**
	 * @brief Destroy the Planner object
	 * 
	 */
	~Planner();

public:
	typedef std::shared_ptr<Planner> Smart_Ptr;


	/**
	 * @brief Singletons should not be cloneable.
	 *
	 * @param other 
	 */
	Planner(Planner& other) = delete;

	/**
	 * @brief Singletons should not be assignable.
	 * 
	 */
	void operator=(const Planner&) = delete;

	/**
	 * @brief Get the Instance object to construct
	 * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
	 * 
	 * @param value 
	 * @return Planner* 
	 */
	static Planner *getInstance(ros::NodeHandle& nh);

	/**
	 * @brief Initialization of planning problem
	*/
	void initPlaning();

	/**
	 * @brief
	*/
	void setStart(double x, double y, double z);

	/**
	 * @brief
	*/
	void setGoal(double x, double y, double z);

	/**
	 * @brief
	*/
	void startCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

	/**
	 * @brief
	*/
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

	/**
	 * @brief needed to continuous reset of planner and hence and making a receding horizon planner
	 * @note THIS MUST BE DEFINED A SEPARATE THREAD THAT CANNOT BE TOUCHED so that it can be guranteed
	 * that once the enviroment changes and consquently the tree, the replanning process has a new start point
	 * and ready to replan. Thus, the call back here needs to contain a reinitialization flag to allow
	 * under conditions the replanning process 
	 * 
	*/
	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

	/**
	 * @brief loading octree from binary
	 * @note only a new stream call the update and subsquently intiate a replan process
	 * The message consists of: the complete maximum-likelihood occupancy map as compact OctoMap binary stream, encoding free and occupied space.
	*/
	void octomapBinaryCallback(const octomap_msgs::Octomap::ConstPtr& binary_msg);

		/**
	 * @brief loading octree from full
	 * @note only a new stream call the update and subsquently intiate a replan process
	 * The message consists of: the complete maximum-likelihood occupancy map as compact OctoMap binary stream, encoding free and occupied space.
	*/
	void octomapFullCallback(const octomap_msgs::Octomap::ConstPtr& binary_msg);

	/**
	 * @brief 
	 * @note: this service is made to solve the problem of segmentation fault 
	 */
	bool goalSettingServiceCallback(manager_msgs::CommandTarget::Request& req,manager_msgs::CommandTarget::Response& res);

	/**
	 * @brief
	*/
	void replan();

	/**
	 * @brief Minimum snap trajectory 
	 * @note https://ieeexplore.ieee.org/document/5980409
	 * 
	 */
	void minSnapTraj(const trajectory_msgs::MultiDOFJointTrajectory& msg);

	/**
	 * @brief the function sets the algorithms needed for finding the path
	 * @note guidlines -> https://ompl.kavrakilab.org/genericPlanning.html
	 *  @return to be called
	*/
	void plan(); 

	/**
	 * @brief This function takes in the path to pass it to the commander
	 * 
	 * @param path 
	 * @return std::pair<Eigen::Vector3d, double>  pos and yaw
	 */
	std::vector<std::pair<Eigen::Vector3d, double>> getPathToFly();
	std::vector<std::tuple<double, double, double>> getBSpline();

	/**
	 * @brief Get the Waypoints
	 * 
	 * @return std::vector<Eigen::Vector3d> 
	 */
	std::vector<Eigen::Vector3d>  getWaypoints() const;

	/**
	 * @brief utility function
	 * 
	 */
	double ToEulerAngles(const Eigen::Quaternionf q);

	/**
	 * @brief check if a goal has been reached or not yet.
	 * 
	 */
	bool isReached();

	/**
	 * @brief check if a goal is reachable and call RRT.
	 * 
	 */
	bool isReachable(ob::State *state);

	/**
	 * @brief this function is made to inform the flight manager in advance about replanning so that it stops executing whilst the solving time
	 * 
	 * @return true if a new path has been calculated due to occupancy checks
	 * @return otherwise
	 */
	bool getReplaned();

	/**
	 * @brief Get the Goal object in case the flight Manager needed it 
	 * 
	 * @return Eigen::Vector3d 
	 */
	Eigen::Vector3d getGoal();

	/**
	 * @brief // I obsereved in large areas the inbetween points could be occupied but that can't initiate a replan since the bspline points couldn't 
	 * itself be occupied but definitely the inbetween distances
	 * 
	 */
	bool resamplePathOccupied(const std::size_t start_idx, const double max_path_length = 0.75);
};


inline geometry_msgs::Pose EigenAffine3dToGeometryPose(const Eigen::Affine3d& transform)
{
  const Eigen::Vector3d trans = transform.translation();
  const Eigen::Quaterniond quat(transform.rotation());
  geometry_msgs::Pose geom_pose;
  geom_pose.position.x = trans.x();
  geom_pose.position.y = trans.y();
  geom_pose.position.z = trans.z();
  geom_pose.orientation.w = quat.w();
  geom_pose.orientation.x = quat.x();
  geom_pose.orientation.y = quat.y();
  geom_pose.orientation.z = quat.z();
  return geom_pose;
}


}
