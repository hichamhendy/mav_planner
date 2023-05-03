#include "mav_planner/path_planner.h"
#include "mav_planner/tailored_objectives.h"

using namespace nut;


/**
* Static methods should be defined outside the class.
*/
Planner* Planner::planner_instance_ {nullptr}; 
std::mutex Planner::mutex_;


Planner::Planner(const ros::NodeHandle& nh): nh_(nh)
{
	initialize_ = false;
	goal_reached_ = false;
	replan_flag_ = false;

	copter_ = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.35, 0.35, 0.25)); // copter dimension
	
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1))); // tree resolution
	tree_ = std::shared_ptr<fcl::CollisionGeometry>(tree);
		
	space_3D_ = std::make_shared<ob::SE3StateSpace>();
	
	// set the constraints/bounds for the R^3 part on SE(3) https://ompl.kavrakilab.org/constrainedPlanning.html#autotoc_md45
	ob::RealVectorBounds bounds(3); 

 	bounds.setLow(0,-10);
	bounds.setHigh(0,10);
	bounds.setLow(1,-10);
	bounds.setHigh(1,10);
	bounds.setLow(2,0.5);
	bounds.setHigh(2,2);

	space_3D_->as<ob::SE3StateSpace>()->setBounds(bounds); // Cast this instance to a desired type

	// construct an instance of  space information from this state space
	// si_SE3_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_3D_));  // The base class for space information. This contains all the information about the space planning is done in. 
	si_SE3_ = std::make_shared<ob::SpaceInformation>(space_3D_);

	// create a start state
	ob::ScopedState<ob::SE3StateSpace> start_(space_3D_);
	start_->setXYZ(0,0,0);
	start_->as<ob::SO3StateSpace::StateType>(1)->setIdentity();	
	// start.random();

	// create a goal state 
	ob::ScopedState<ob::SE3StateSpace> goal_(space_3D_);
	goal_->setXYZ(0,0,0);
	former_goal_[0] = 0;
	former_goal_[1] = 0;
	former_goal_[2] = 0;
	goal_->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// goal.random();
		
	// Set the instance of the state validity checker to decide whether a given state in apecific static is valid to use. 
	// Parallelly implementation assume this validity checker is thread safe. 
	si_SE3_->setStateValidityChecker(std::bind(&Planner::isStateOccupied, this, std::placeholders::_1 )); // generates a forwarding call wrapper for isStateOccupied, an occupied cell is NOT valid

	// Set the instance of the motion validity checker to use. 
	// Also: Parallel implementations of planners assume this validity checker is thread safe.
	// si_SE3_->setMotionValidator (std::bind(&Planner::isMotionValid, this, std::placeholders::_1 )); // to be implemented but wouldn't be needed if I gonna smooth the curve

	// create a problem instance
	problem_def_ = std::make_shared<ob::ProblemDefinition>(si_SE3_);

	// set the start and goal states
	problem_def_->setStartAndGoalStates(start_, goal_);

	// set Optimizattion objective
	// problem_def_->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si_SE3_)); // hereby, the cost function should be formalized
	problem_def_->setOptimizationObjective(getBalancedObjective(si_SE3_));

	//path_bsplining_ = std::make_shared<og::PathGeometric*>();
	
	octree_sub_ = nh_.subscribe<octomap_msgs::Octomap>("/rtabmap/octomap_binary", 1, boost::bind(&Planner::octomapBinaryCallback, this, _1)); 
	ekf2_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, boost::bind(&Planner::odomCallback, this, _1)); // gonna use /rtabmap/odom instead of ekf2 /mavros/local_position/odom (temporairly)
	goal_sub_ =  nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&Planner::goalCallback, this, _1)); // to be downed

	path_pub_ = nh_.advertise<visualization_msgs::Marker>( "/mav_planner/path", 1);
	path_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "/mav_planner/path_array", 1);
	waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/mav_planner/rrt_waypoints", 1);
	waypoint_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/mav_planner/waypoints", 1); // latch on false by default

	goal_setting_server_ = nh_.advertiseService("/goal_setting", &Planner::goalSettingServiceCallback, this);

	mapupdate_dt_ = 0.2;

	ROS_INFO_STREAM("==================================================================================");
	ROS_INFO_STREAM("Path planner object Constructed.");
	ROS_INFO_STREAM("==================================================================================");
}


Planner* Planner::getInstance(ros::NodeHandle& nh)
{
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */

	std::lock_guard<std::mutex> lock(mutex_);
    if(planner_instance_ == nullptr)
	{
        planner_instance_ = new Planner(nh);
    }
    return planner_instance_;
}


Planner::~Planner()
{
	if (tree_oct_)
	{
		// delete tree_oct_;
		// tree_oct_ = NULL;
		tree_oct_->clear();
  	}
	// delete path_bsplining_;

	// if(goal_setting_thread_.joinable())
	// 	goal_setting_thread_.join();
}


void Planner::octomapBinaryCallback(const octomap_msgs::Octomap::ConstPtr& binary_msg)
{
	// ROS_INFO("Message is readable and its Resolution is %f", binary_msg->resolution);
	
	// ROS_INFO("OctoMap memory usage: %2.3f MB", octree_->memoryUsage() / 1000000.0);

	if (tree_oct_ != NULL) 
	{
		tree_oct_->clear();
		//ROS_INFO("DEBUG: Tree DEL");
  	}


	ros::Time current = ros::Time::now();
  	// Update map at a fixed rate. This is useful on setting replanning rates for the planner.
	if ((current - last_wp_time_).toSec() < mapupdate_dt_) 
	{
		return;
	}
	last_wp_time_ = ros::Time::now();

	// use the basic form of octree for less coputational load and compact representation (also fcl needs that http://docs.ros.org/en/fuerte/api/fcl/html/classfcl_1_1OcTree.html#a6594321e94b100bcdd97c1be33aa5814)
	msg_octree_ = octomap_msgs::msgToMap(*binary_msg); // binaryMsgToMap works here also

	// notice msgToMap: Convert an octomap representation to a new octree (full probabilities or binary). You will need to free the memory. Return NULL on error.
	// on the other side fullMapToMsgData: serialization of an octree into binary data e.g. for messages and services. Full probability version (stores complete state of tree, .ot file format). 
	// The data will be much smaller if you call octomap.toMaxLikelihood() and octomap.prune() before. swich the topic of the octo server to /octomap_full

 	if (msg_octree_)
	{
        //Create object of type octomap::OcTree*
		// ROS_INFO("casting initiated");
		// std::unique_lock<std::mutex> lock(octomap_mutex_);  //  A unique_lock can be created without immediately locking, can unlock at any point in its existence, and can transfer ownership of the lock from one instance to another.
		std::lock_guard<std::mutex> lock(octomap_mutex_); // A lock_guard always holds a lock from its construction to its destruction. 
		// boost::mutex::scoped_lock _(update_cell_sets_lock_);
        // tree_oct_ = dynamic_cast<octomap::OcTree*>(msg_octree_);  // dynamic casting from octomap::AbstractOcTree to octomap::OcTree // notich to ColorOcTree wored!!!
		tree_oct_ = (octomap::OcTree*)msg_octree_; // Memory leak fix

		// substantial reduction
		// tree_oct_->prune();
		// Updates the occupancy of all inner nodes to reflect their children's occupancy. // it's necessary if the OccupancyOctTreeBase is called with lazy_eval = false (to be checked)
		// tree_oct_->updateInnerOccupancy();
		if(tree_oct_ == NULL)
		{
			ROS_WARN("Casting failed, still NULL !I'm out");
			return;
		}
			
	} 
	


 	if (tree_oct_)
    {
        // ROS_INFO("%s: Octo-Map received %zu nodes, %f m of resolution", ros::this_node::getName().c_str(), tree_oct_->size(), tree_oct_->getResolution());
        // fcl::OcTree* tree = NULL;
		// convert the octomap::octree to fcl::octree fcl_octree object
		// tree = (new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct_))); // I am giving a the tree in form of a shared ptr to guarntee the shared ownership
		std::shared_ptr<fcl::OcTree> tree {new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct_))};
		// Update the octree used for collision checking
		// std::thread(&Planner::updateOctotree, std::ref(tree));
		updateOctotree(std::shared_ptr<fcl::CollisionGeometry>(tree));
		// it's so essential to replan in case of new updates (discovries in the environment)
		if(!goal_reached_)
			replan(); 
			// std::thread(&Planner::replan, this).detach(); //  leave it to run on its own
		// delete tree;
		//delete tree_oct_;
		// delete msg_octree_;
    } 
	else
	{
		ROS_ERROR("Error by reading OcTree from stream. Casting failed");
		// delete msg_octree_;
	} 
}

void Planner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if(msg->pose.position.z < 0.0)
    	return;

	ROS_INFO("[%s] A target has been sent!", __PACKAGE_NAME__);
	// goal_orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,msg->pose.orientation.y, msg->pose.orientation.z);
	// isRechable();
	ROS_DEBUG_STREAM("goalCallback: " << *msg);
	setGoal(msg->pose.position.x, msg->pose.position.y, elevation_goal_);
}


bool Planner::goalSettingServiceCallback(manager_msgs::CommandTarget::Request& req,manager_msgs::CommandTarget::Response& res)
{
	Eigen::Vector3d position;
  	tf2::fromMsg(req.destination, position);

	ROS_INFO("[%s] A target has been sent!", __PACKAGE_NAME__);
	ROS_DEBUG_STREAM("goalServiceCallback: " << req.destination);
	setGoal(position.x(), position.y(), position.z());

	res.success = 1;
	return true;
}


void Planner::initPlaning()
{
	if(!initialize_)
		std::cout << "Planner Initialized" << std::endl;
	initialize_ = true;
}


void Planner::setStart(double x, double y, double z)
{
	ob::RealVectorBounds bounds(3); 
	bounds.setLow(0,-10 + x);
	bounds.setHigh(0,10 + x);
	bounds.setLow(1,-10 + y);
	bounds.setHigh(1,10 + y);
	bounds.setLow(2,0.5);
	bounds.setHigh(2,2);
	space_3D_->as<ob::SE3StateSpace>()->setBounds(bounds); // Cast this instance to a desired type

	ob::ScopedState<ob::SE3StateSpace> start(space_3D_);
	start->setXYZ(x,y,z);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	problem_def_->clearStartStates();
	problem_def_->addStartState(start);
}


void Planner::setGoal(double x, double y, double z)
{
	const Eigen::Vector3d check_former_goal_ (former_goal_[0], former_goal_[1], former_goal_[2]);
    const Eigen::Vector3d check_actual_goal_ (x, y, z);
	// check if the new goal is outside a sphere of 0.4 to save compuational load of planning
	if((check_former_goal_ - check_actual_goal_).norm() > 0.5) // add another check of how far the quad actuall is from the target
	{
		// replan_flag_ = true;
		ob::ScopedState<ob::SE3StateSpace> goal(space_3D_);
		goal->setXYZ(x,y,z);
		former_goal_[0] = x;
		former_goal_[1] = y;
		former_goal_[2] = z;
		goal_ << x, y, z;
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		problem_def_->clearGoal();
		problem_def_->setGoalState(goal);
		if(initialize_) 
			plan(); // this must not be the only authority to initiate the planning process.

		// ob::State *state =  space_3D_->allocState();
		// goal_reachable_ = isReachable(state);
		// space_3D_->as<ob::SO3StateSpace::StateType>()->values = goal->values;

		goal_reached_ = false;
	}
	else
		ROS_WARN("[%s] Doesn't really worth the load of plannung. It's just %f", __PACKAGE_NAME__, sqrt(abs(former_goal_[0] - x) + abs(former_goal_[1] - y) + abs(former_goal_[2] - z)));
}


bool Planner::isReached()
{
	if ((current_position_ - goal_).norm() < 0.5 && current_position_.norm() >= 0.1 && goal_.norm() >= 0.1)
	{
		ROS_WARN("[%s] Goal vicinity has bee reached, Replanning not Required", __PACKAGE_NAME__);
		goal_reached_ = true;
		return goal_reached_;
	}
	goal_reached_ = false;
	return goal_reached_;
}


bool Planner::isReachable(ob::State *state)
{	
	if(isStateOccupied(state)) // Check if the goal state is free at all maybe the user clicked wrong
	{	
		ROS_INFO("[%s] Goal point set to: %f, %f, %f is AT THE MOMENT unoccupied.  I am heading to ", __PACKAGE_NAME__, goal_.x(), goal_.y(), goal_.z());
		if(initialize_) // hold on till all get set
			plan(); // this must not be the only authority to initiate the planning process.
		return true;
	}
	else
	{
		ROS_ERROR("[%s] Goal point set to: %f, %f, %f is invalid due to faliure in free space check", __PACKAGE_NAME__, goal_.x(), goal_.y(), goal_.z());
		return false;
	}
}


void Planner::updateOctotree(std::shared_ptr<fcl::CollisionGeometry> map)
{
	std::lock_guard<std::mutex> lock(collide_lock);
	try
	{
		tree_ = map;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
}


void Planner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	ROS_DEBUG_STREAM("OdomCallback: " << *msg);
	if( std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y) || std::isnan(msg->pose.pose.position.z))
        return;
	// an observation: If EKF2 spins around, it causes invalid goal setting. It mus be assured that SLAM and EKF2 doesn't deviate to avoid that.
	setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z); // update the start point whilst moving based on EKF2
	elevation_goal_ = msg->pose.pose.position.z;
	
	current_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
	// By rasing the flag, I make the way free to a receding horizon planning, not like before global planning.  
	initPlaning();
	// isReached();
}


void Planner::startCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	setStart(msg->point.x, msg->point.y, msg->point.z);
	initPlaning();
}


Eigen::Vector3d Planner::getGoal()
{
	return goal_;
}


void Planner::replan()
{
	// odomCb and startPlanningCb (exceptopnal) are allowed to raise the planning flag
	if(path_bsplining_ != NULL && initialize_) // check if nothing turned down the reinitialzation flag and path_smoother ptr isn't empty yet which all will happen at the first call
	{
		ROS_INFO("[%s] Spline total points: %ld", __PACKAGE_NAME__, path_bsplining_->getStateCount ());
		if(path_bsplining_->getStateCount () <= 2) // 2 points bspline would be bad to follow. Reinitiate 
		{
			plan();
			ROS_INFO("[%s] Bspline  of 2 points;hence, I'm replanning", __PACKAGE_NAME__);
		}
		else
		{
			for (std::size_t idx = 0; idx < path_bsplining_->getStateCount () - 1; idx++)
			{
				if(!replan_flag_)
					replan_flag_ = !isStateOccupied(path_bsplining_->getState(idx));// || resamplePathOccupied(idx); // a recheck to enforce the receding horizon planning, the tree parallel updated and consuently the state space
				else
					break;

			}
			if(replan_flag_)
			{
				plan(); // initiate the planning to consider the tree after the a volumetric change
			}
			// else
				// ROS_INFO("[%s] Path is clear; Replanning not required", __PACKAGE_NAME__);
		}
	}
}


void Planner::plan()
{
	boost::lock_guard<boost::mutex> loc(plan_mutex_);
	
	// create a planner for the defined space -> https://ompl.kavrakilab.org/classompl_1_1base_1_1Planner.html
	// ob::PlannerPtr plan(new og::InformedRRTstar(si_SE3_)); // variant  multilevel::QRRT
	ob::PlannerPtr plan = std::make_shared<og::InformedRRTstar>(si_SE3_);

	// set the problem we are trying to solve for the planner The problem needs to be set before calling solve(). 
	plan->setProblemDefinition(problem_def_);

	// perform setup steps for the planner (extra configuration steps). This must be called before solving. 
	plan->setup();

	// std::cout << "\n\n***** Planning Space: "<< std::endl;
	// si_SE3_->printSettings(std::cout);

	// std::cout << "\n\n***** Problem definition: "<< std::endl;
	// problem_def_->print(std::cout);

	// attempt to solve the problem within 2 second of planning time
	ob::PlannerStatus solved = plan->solve(2);

	if (solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state) and inquire about the found path
		ROS_INFO("[%s] Found solution!", __PACKAGE_NAME__);
		ob::PathPtr path = problem_def_->getSolutionPath();
		og::PathGeometric* pth = problem_def_->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
	        path->print(std::cout); 	        // print the path to screen

		trajectory_msgs::MultiDOFJointTrajectory msg; // flight manager should be utilizing that
		trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
		visualization_msgs::MarkerArray waypoint_vis;

		for(auto & mk: waypoint_vis.markers) 
        	mk.action = visualization_msgs::Marker::DELETE;
		waypoint_marker_pub_.publish(waypoint_vis);

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "map";
		msg.joint_names.clear();
		msg.points.clear();
		msg.joint_names.push_back("planner_waypoints");
		visualization_msgs::Marker mk;
		mk.header.frame_id = "map";
		mk.header.stamp = ros::Time(); // ros::Time::now(); // revise
		mk.ns = "rrt/waypoints";
		mk.type = visualization_msgs::Marker::CUBE;
		mk.action = visualization_msgs::Marker::ADD;
		mk.lifetime = ros::Duration(0.0);
		mk.frame_locked = false;
		const Eigen::Affine3d base_transform = Eigen::Affine3d::Identity();
		mk.pose = EigenAffine3dToGeometryPose(base_transform);
		mk.pose.orientation.x = 0.0;
		mk.pose.orientation.y = 0.0;
		mk.pose.orientation.z = 0.0;
		mk.pose.orientation.w = 1.0;
		mk.color.a = 1.0;
		mk.color.r = 1.0;
		mk.color.g = 0.0;
		mk.color.b = 0.0; 


			
		for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

	            	// extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            	// extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

			point_msg.time_from_start.fromSec(ros::Time::now().toSec());
			point_msg.transforms.resize(1);

			point_msg.transforms[0].translation.x=  pos->values[0];
			point_msg.transforms[0].translation.y = pos->values[1];
			point_msg.transforms[0].translation.z = pos->values[2];

			point_msg.transforms[0].rotation.x = rot->x;
			point_msg.transforms[0].rotation.y = rot->y;
			point_msg.transforms[0].rotation.z = rot->z;
			point_msg.transforms[0].rotation.w = rot->w;

			mk.pose.position.x = pos->values[0]; 
			mk.pose.position.y = pos->values[1]; 
			mk.pose.position.z = pos->values[2];  

			mk.scale.x = 0.1;
			mk.scale.y = 0.1;
			mk.scale.z = 0.1; 
        
        	waypoint_vis.markers.push_back(mk);

			waypoints_.push_back(Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]));

			msg.points.push_back(point_msg);
		}

		waypoint_pub_.publish(msg);
		waypoint_marker_pub_.publish(waypoint_vis);


	        //Path smoothing using bspline

		og::PathSimplifier* pathBSpline = new og::PathSimplifier(si_SE3_); // Create an instance for a specified space information. 
		// path_bsplining_ = NULL;
		// path_bsplining_ = new og::PathGeometric(dynamic_cast<const og::PathGeometric&> (*problem_def_->getSolutionPath())); // regular casting (og::PathGeometric&)  vs dynamic casting: dynamic_cast<const og::PathGeometric&> still troubling!!
		path_bsplining_ = std::make_shared<og::PathGeometric>(dynamic_cast<const og::PathGeometric&> (*problem_def_->getSolutionPath())); // examine auto downcastedPtr = std::dynamic_pointer_cast<Derived>(basePtr)
		pathBSpline->smoothBSpline(*path_bsplining_,3); // smoothing attempt with max attempts try of 3, given a path, attempt to smooth it (the validity of the path is maintained). 
		// std::cout << "Smoothed Path: " << std::endl;
		// path_bsplining_->print(std::cout);

		path_bspline_.clear();
		//Publish path as markers

		visualization_msgs::Marker marker;
		visualization_msgs::MarkerArray path_vis; 
		marker.action = visualization_msgs::Marker::DELETEALL;
		path_pub_.publish(marker);

		path_vis.markers.clear();
		pose_set_.clear();
		

		for (std::size_t idx = 0; idx < path_bsplining_->getStateCount (); idx++)
		{
	        	// cast the abstract state type to the type we expect
			const ob::SE3StateSpace::StateType *se3state = path_bsplining_->getState(idx)->as<ob::SE3StateSpace::StateType>();

	        	// extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            	// extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			// const Eigen::Quaternionf* quat(rot->w, rot->x, rot->y, rot->z);
				
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "bspline/path";
			marker.id = idx;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pos->values[0];
			marker.pose.position.y = pos->values[1];
			marker.pose.position.z = pos->values[2];
			marker.pose.orientation.x = rot->x;
			marker.pose.orientation.y = rot->y;
			marker.pose.orientation.z = rot->z;
			marker.pose.orientation.w = rot->w;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			path_pub_.publish(marker);
			// ros::Duration(0.1).sleep();
			// std::cout << "Published marker: " << idx << std::endl;  
			path_vis.markers.push_back(marker);

			pose_set_.push_back( std::make_pair(Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]),
				ToEulerAngles(Eigen::Quaternionf(rot->w, rot->x, rot->y, rot->z))));
			path_bspline_.push_back(std::tuple<double, double, double>(pos->values[0], pos->values[1], pos->values[2]));
			no_waypoints = idx;
		}
			
		// Clear memory
		problem_def_->clearSolutionPaths();
		replan_flag_ = false;

		path_array_pub_.publish(path_vis);
		delete pathBSpline;

	}
	else
		ROS_WARN("[%s] No Solution found. Try another target point", __PACKAGE_NAME__);

	// std::this_thread::sleep_for( std::chrono::seconds(1) );
}


bool Planner::isStateOccupied(const ob::State *state)
{
	// std::lock_guard<std::mutex> lock(collide_lock); // a try to fix segment fault problem

	const ob::SE3StateSpace::StateType *SE3_state_ = state->as<ob::SE3StateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = SE3_state_->as<ob::RealVectorStateSpace::StateType>(0);

	// extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = SE3_state_->as<ob::SO3StateSpace::StateType>(1);

	collide_lock.lock();
	fcl::CollisionObject treeObj(tree_);  // https://flexible-collision-library.github.io/d3/d70/classfcl_1_1CollisionObject.html
	fcl::CollisionObject interestObject(copter_);
	collide_lock.unlock();

	// check validity of state defined by pos & rot. Before starting the proximity computation, we need first to set the geometry and transform for the objects involving in computation. 
	fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
	interestObject.setTransform(rotation, translation);

	/*
	Once the objects are set, we can perform the proximity computation between them.
	 All the proximity queries in FCL follow a common pipeline: first, set the query 
	 request data structure and then run the query function by using request as the input. 
	 The result is returned in a query result data structure. 
	 For example, for collision checking, we first set the CollisionRequest data structure, 
	 and then run the collision function
	*/

	// set the collision request structure, here we just use the default setting
	fcl::CollisionRequest requestType(1,false,1,false);  // CollisionRequest (size_t num_max_contacts_=1, bool enable_contact_=false, size_t num_max_cost_sources_=1, bool enable_cost_=false)
	// result will be returned via the collision result structure
	fcl::CollisionResult collisionResult; // return binary collision result 
	// perform collision test
	
	fcl::collide(&interestObject, &treeObj, requestType, collisionResult); 
	

	return(!collisionResult.isCollision()); // https://flexible-collision-library.github.io/dc/d6f/structfcl_1_1CollisionResult.html#a2b00f46c4c676d081918d3a6a8b47b40
}


bool Planner::getReplaned()
{
	return replan_flag_;
}


std::vector<std::pair<Eigen::Vector3d, double>> Planner::getPathToFly()
{
	return pose_set_;
}


std::vector<std::tuple<double, double, double>> Planner::getBSpline()
{
	return path_bspline_;
}


bool Planner::resamplePathOccupied(const std::size_t start_idx, const double max_path_length)
{

	    // cast the abstract state type to the type we expect
		const ob::SE3StateSpace::StateType *se3state1 = path_bsplining_->getState(start_idx)->as<ob::SE3StateSpace::StateType>();
		const ob::RealVectorStateSpace::StateType *pos1 = se3state1->as<ob::RealVectorStateSpace::StateType>(0);
		const ob::SE3StateSpace::StateType *se3state2 = path_bsplining_->getState(start_idx + 1)->as<ob::SE3StateSpace::StateType>();
		const ob::RealVectorStateSpace::StateType *pos2 = se3state2->as<ob::RealVectorStateSpace::StateType>(0);
		
		auto p1 = Eigen::Vector3d(pos1->values[0],  pos1->values[1], pos1->values[2]);	
		const auto p2 = Eigen::Vector3d(pos2->values[0],  pos2->values[1], pos2->values[2]);
		while ((p2 - p1).norm() > max_path_length)
		{
			p1 = p1 + (p2 - p1).normalized() * 0.5 * max_path_length;

			// ob::ScopedState<ob::SE3StateSpace> inspection_p(space_3D_);
			// inspection_p->setXYZ(p1.x(), p1.y(), p1.z());
			// ob::State *state =  space_3D_->allocState();

			fcl::CollisionObject treeObj(tree_);  // https://flexible-collision-library.github.io/d3/d70/classfcl_1_1CollisionObject.html
			fcl::CollisionObject interestObject(copter_);

			fcl::Vec3f translation(p1.x(), p1.y(),p1.z());
			fcl::Quaternion3f rotation(1, 0, 0, 0);
			interestObject.setTransform(rotation, translation);

			fcl::CollisionRequest requestType(1,false,1,false);  // CollisionRequest (size_t num_max_contacts_=1, bool enable_contact_=false, size_t num_max_cost_sources_=1, bool enable_cost_=false)
			fcl::CollisionResult collisionResult; // return binary collision result 

			fcl::collide(&interestObject, &treeObj, requestType, collisionResult); 
			
			if(collisionResult.isCollision())
				return 1;
		}
	
	return 0;
}


double Planner::ToEulerAngles(const Eigen::Quaternionf q) 
{
    Eigen::Vector3f angles;    //yaw pitch roll

    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

/*     // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp); */

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[0] = std::atan2(siny_cosp, cosy_cosp);
    return angles[0];
}

ob::OptimizationObjectivePtr Planner::getThresholdPathLengthObj(const ob::SpaceInformationPtr &si)
{
	// under development 
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	// Set the cost threshold for objective satisfaction. When a path is found with a cost better than the cost threshold, the objective is considered satisfied.	
	obj->setCostThreshold(ob::Cost(1.51)); // https://ompl.kavrakilab.org/optimalPlanningTutorial.html
	obj->print(std::cout);
	obj->getDescription();
	return obj;
}


ob::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr &si)
{
	// stable
	ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
	lengthObj->setCostToGoHeuristic(&ob::goalRegionCostToGo); // The definition of a function which returns an admissible estimate of the optimal path cost from a given state to a goal. 
	lengthObj->print(std::cout);
	lengthObj->getDescription();
	return lengthObj;
}


std::vector<Eigen::Vector3d>  Planner::getWaypoints() const
{
	return waypoints_;
}


ob::OptimizationObjectivePtr Planner::getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new MaximizeMinClearance(si));
 
	// To objectives to be optimised: path length and clearance (t distance between the path and an obstacle).
    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
	lengthObj->setCostToGoHeuristic(&ob::goalRegionCostToGo);

	// we weight the length with a factor of 2.0 and the clearance with a factor of 1.0 to try to balance more in favor of minimizing path length in planning. 
    opt->addObjective(lengthObj, 1.0);
    opt->addObjective(clearObj, 1.0);

	// return 2.0 * lengthObj + clearObj;	// the same as before
 
    return ob::OptimizationObjectivePtr(opt);
}



ob::OptimizationObjectivePtr Planner::getMaxMinClearanceObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr minMaxObj(new MaximizeMinClearance(si));
	// minMaxObj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return minMaxObj;
}



ob::Cost MaximizeMinClearance::MaximizeMinClearance::stateCost(const ob::State* s) const
{
    return ob::Cost(this->si_->getStateValidityChecker()->clearance(s));
}


bool MaximizeMinClearance::isCostBetterThan(ob::Cost c1, ob::Cost c2) const
{
    return c1.value() > c2.value() + ompl::magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION; 	//+ ompl::magic::BETTER_PATH_COST_MARGIN;
}


ob::Cost MaximizeMinClearance::combineCosts(ob::Cost c1, ob::Cost c2) const
{
    if (c1.value() < c2.value())
        return c1;
    else
        return c2;
}


ob::Cost MaximizeMinClearance::motionCost(const ob::State *s1, const ob::State *s2) const
{
    return this->combineCosts(this->stateCost(s1), this->stateCost(s2));
}


ob::Cost MaximizeMinClearance::identityCost() const
{
    return ob::Cost(std::numeric_limits<double>::infinity());
}


ob::Cost MaximizeMinClearance::infiniteCost() const
{
    return ob::Cost(-std::numeric_limits<double>::infinity());
}
