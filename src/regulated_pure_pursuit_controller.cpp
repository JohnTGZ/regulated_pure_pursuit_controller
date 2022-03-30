#include "regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

// PLUGINLIB_DECLARE_CLASS has been changed to PLUGINLIB_EXPORT_CLASS in ROS Noetic
// Changing all tf::TransformListener* to tf2_ros::Buffer*
PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav_core::BaseLocalPlanner)

namespace regulated_pure_pursuit_controller
{

    RegulatedPurePursuitController::RegulatedPurePursuitController() : initialized_(false), odom_helper_("odom"), goal_reached_("false")
    { }    

    void RegulatedPurePursuitController::initialize(std::string name, tf2_ros::Buffer *tf, 
                                            costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!isInitialized() ){
            ros::NodeHandle pnh_("~/" + name);

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            costmap_ros_->getRobotPose(current_pose_);

            costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

            initParams(pnh_);
            initPubSubSrv(pnh_);

            odom_helper_.setOdomTopic( odom_topic_ );

            initialized_ = true;
            ROS_INFO("PurePursuitPlanner Initialized");
        }
        else {
            ROS_WARN("RegulatedPurePursuitController has already been initialized, doing nothing.");
        }

    }

    void RegulatedPurePursuitController::initPubSubSrv(ros::NodeHandle& nh){
        global_path_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
        local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
        carrot_pub_ = nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);
        carrot_arc_pub_ = nh.advertise<nav_msgs::Path>("lookahead_collision_arc", 1);
        
        local_cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    void RegulatedPurePursuitController::initParams(ros::NodeHandle& nh){
        nh.param<std::string>("odom_topic", odom_topic_, "odom");

        nh.param<double>("max_robot_pose_search_dist", max_robot_pose_search_dist_, getCostmapMaxExtent());

        //Lookahead
        nh.param<double>("lookahead_time", lookahead_time_, 1.5);
        nh.param<double>("lookahead_dist", lookahead_dist_, 0.25);
        nh.param<bool>("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_, false);
        nh.param<double>("min_lookahead_dist", min_lookahead_dist_, 0.3);
        nh.param<double>("max_lookahead_dist", max_lookahead_dist_, 0.9);

        //Rotate to heading param
        nh.param<bool>("use_rotate_to_heading", use_rotate_to_heading_, false);
        nh.param<double>("rotate_to_heading_min_angle", rotate_to_heading_min_angle_, 0.785);
        nh.param<double>("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_, 1.8);
        nh.param<double>("max_angular_accel", max_angular_accel_, 1.5);

        //Reversing
        nh.param<bool>("allow_reversing", allow_reversing_, false);
        if (use_rotate_to_heading_ && allow_reversing_) {
            ROS_WARN("Disabling reversing. Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. By default setting use_rotate_to_heading true");
            allow_reversing_ = false;
        }

        //Speed
        nh.param<double>("desired_linear_vel", desired_linear_vel_, 0.5);
        nh.param<double>("max_angular_vel", max_angular_vel_, 1.5);
        nh.param<double>("min_approach_linear_velocity", min_approach_linear_velocity_, 0.05);

        //Regulated linear velocity scaling
        nh.param<bool>("use_regulated_linear_velocity_scaling", use_regulated_linear_velocity_scaling_, false);
        nh.param<double>("regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_, 0.9);
        nh.param<double>("regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_, 0.25);
        
        //Inflation cost scaling (Limit velocity by proximity to obstacles)
        nh.param<bool>("use_cost_regulated_linear_velocity_scaling", use_cost_regulated_linear_velocity_scaling_, true);
        nh.param<double>("inflation_cost_scaling_factor", inflation_cost_scaling_factor_, 3.0);
        nh.param<double>("cost_scaling_dist", cost_scaling_dist_, 0.6);
        nh.param<double>("cost_scaling_gain", cost_scaling_gain_, 1.0);
        if (inflation_cost_scaling_factor_ <= 0.0){
            ROS_WARN("The value inflation_cost_scaling_factor is incorrectly set, "
                "it should be >0. Disabling cost regulated linear velocity scaling.");
            use_cost_regulated_linear_velocity_scaling_ = false;
        }

        //Collision avoidance
        nh.param<double>("max_allowed_time_to_collision_up_to_carrot", max_allowed_time_to_collision_up_to_carrot_, 1.0);
        
        nh.param<double>("goal_dist_tol", goal_dist_tol_, 0.25);

        double control_frequency;
        nh.param<double>("control_frequency", control_frequency, 20);
        control_duration_ = 1.0 / control_frequency;

        double transform_tolerance;
        nh.param<double>("transform_tolerance", transform_tolerance, 0.1);
        transform_tolerance_ = ros::Duration(transform_tolerance);




        //Ddynamic Reconfigure

        ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(nh));
        ddr_->registerVariable<double>("lookahead_time", &this->lookahead_time_, "", 0.0, 20.0);
        ddr_->registerVariable<double>("lookahead_dist", &this->lookahead_time_, "", 0.0, 20.0);

        ddr_->registerVariable<bool>("use_velocity_scaled_lookahead_dist", &this->use_velocity_scaled_lookahead_dist_);
        ddr_->registerVariable<double>("min_lookahead_dist", &this->min_lookahead_dist_, "", 0.0, 5.0);
        ddr_->registerVariable<double>("max_lookahead_dist", &this->max_lookahead_dist_, "", 0.0, 10.0);

        //Rotate to heading param
        ddr_->registerVariable<bool>("use_rotate_to_heading", &this->use_rotate_to_heading_);
        ddr_->registerVariable<double>("rotate_to_heading_min_angle", &this->rotate_to_heading_min_angle_, "", 0.0, 15.0);
        ddr_->registerVariable<double>("rotate_to_heading_angular_vel", &this->rotate_to_heading_angular_vel_, "", 0.0, 15.0);
        ddr_->registerVariable<double>("max_angular_accel", &this->max_angular_accel_, "", 0.0, 15.0);

        //Speed
        ddr_->registerVariable<double>("desired_linear_vel", &this->desired_linear_vel_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("max_angular_vel", &this->max_angular_vel_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("min_approach_linear_velocity", &this->min_approach_linear_velocity_, "", 0.0, 10.0);

        //Regulated linear velocity scaling
        ddr_->registerVariable<bool>("use_regulated_linear_velocity_scaling", &this->use_regulated_linear_velocity_scaling_);
        ddr_->registerVariable<double>("regulated_linear_scaling_min_radius", &this->regulated_linear_scaling_min_radius_, "", 0.0, 5.0);
        ddr_->registerVariable<double>("regulated_linear_scaling_min_speed", &this->regulated_linear_scaling_min_speed_, "", 0.0, 5.0);
        
        //Inflation cost scaling (Limit velocity by proximity to obstacles)
        ddr_->registerVariable<bool>("use_cost_regulated_linear_velocity_scaling", &this->use_cost_regulated_linear_velocity_scaling_);
        ddr_->registerVariable<double>("inflation_cost_scaling_factor", &this->inflation_cost_scaling_factor_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("cost_scaling_dist", &this->cost_scaling_dist_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("cost_scaling_gain", &this->cost_scaling_gain_, "", 0.0, 10.0);

        //Collision avoidance
        ddr_->registerVariable<double>("max_allowed_time_to_collision_up_to_carrot", &this->max_allowed_time_to_collision_up_to_carrot_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("goal_dist_tol", &this->goal_dist_tol_, "", 0.0, 4.0);

        ddr_->publishServicesTopics();
        
    }

    bool RegulatedPurePursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
            return false;
        }

        global_plan_.poses.clear();
        //Convert the global plan to nav_msgs::Path as the original regulated pure pursuit uses this message type
        createPathMsg(orig_global_plan, global_plan_);

        goal_reached_ = false;

        return true;
    }

    bool RegulatedPurePursuitController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if(!initialized_)
        {
            ROS_ERROR("RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
            return false;
        }

        goal_reached_ = false;

        double linear_vel, angular_vel;
        double sign = 1.0;
        if (allow_reversing_) {
            sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
        }

        //Get current pose of robot
        geometry_msgs::PoseStamped pose; 
        costmap_ros_->getRobotPose(pose);
        ROS_INFO("Got robot pose with frame_id: %s", pose.header.frame_id.c_str());
        ROS_INFO("Global Plan frame_id: %s", global_plan_.header.frame_id.c_str());
        

        //Get the current speed of the robot
        geometry_msgs::Twist speed; 
        getRobotVel(speed);

        // Transform path to robot base frame
        nav_msgs::Path transformed_plan = transformGlobalPlan(pose);

        //Dynamically adjust look ahead distance based on the speed
        double lookahead_dist = getLookAheadDistance(speed);

        //Get lookahead point and publish for visualization
        geometry_msgs::PoseStamped carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
        carrot_pub_.publish(createCarrotMsg(carrot_pose));

        //Carrot distance squared
        const double carrot_dist2 =
            (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
            (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

        // Find curvature of circle (k = 1 / R)
        double curvature = 0.0;
        if (carrot_dist2 > 0.001) {
            curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
        }

        //check if global plan is reached
        checkGoalReached(pose);


        linear_vel = desired_linear_vel_;
        // Make sure we're in compliance with basic constraints
        double angle_to_heading;

        //IF robot should use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_
        if (shouldRotateToGoalHeading(carrot_pose)) {
            double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
            rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
        } 
        //IF robot should use_rotate_to_heading_ && angle_to_heading > rotate_to_heading_min_angle_
        else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
            rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
        } 
        //Travel forward and accordinging to the curvature
        else {
            //Constrain linear velocity
            applyConstraints(
                std::fabs(lookahead_dist - sqrt(carrot_dist2)),
                lookahead_dist, curvature, speed,
                costAtPose(pose.pose.position.x, pose.pose.position.y), linear_vel, sign);
            // Apply curvature to angular velocity after constraining linear velocity
            angular_vel = linear_vel * curvature;
            //Ensure that angular_vel does not exceed user-defined amount
            angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        }

        //Collision checking on this velocity heading
        const double & carrot_dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
        if (isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
            ROS_WARN("RegulatedPurePursuitController detected collision ahead!");
        }

        // populate and return message
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;

        //For debugging
        local_cmd_vel_pub_.publish(cmd_vel);

        return true;
    }

    bool RegulatedPurePursuitController::isGoalReached()
    {
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose from costmap");
            return false;
        }

        if (goal_reached_){
            ROS_INFO("GOAL Reached!");
            return true;
        }
        return false;

    }

    bool RegulatedPurePursuitController::shouldRotateToPath(
        const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path)
    {
        // Whether we should rotate robot to rough path heading
        angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
        return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
    }

    bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
        const geometry_msgs::PoseStamped & carrot_pose)
    {
        // Whether we should rotate robot to goal heading
        double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
        return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
    }

    void RegulatedPurePursuitController::rotateToHeading(
        double & linear_vel, double & angular_vel,
        const double & angle_to_path, const geometry_msgs::Twist & curr_speed)
    {
        // Rotate in place using max angular velocity / acceleration possible
        linear_vel = 0.0;
        const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
        angular_vel = sign * rotate_to_heading_angular_vel_;

        const double & dt = control_duration_;
        const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
        const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
        angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
    }


    void RegulatedPurePursuitController::applyConstraints(
        const double & dist_error, const double & lookahead_dist,
        const double & curvature, const geometry_msgs::Twist & /*curr_speed*/,
        const double & pose_cost, double & linear_vel, double & sign)
    {
        double curvature_vel = linear_vel;
        double cost_vel = linear_vel;
        double approach_vel = linear_vel;

        // limit the linear velocity by curvature
        const double radius = fabs(1.0 / curvature);
        const double & min_rad = regulated_linear_scaling_min_radius_;
        if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
            curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
        }

        // limit the linear velocity by proximity to obstacles
        if (use_cost_regulated_linear_velocity_scaling_ &&
            pose_cost != static_cast<double>(costmap_2d::NO_INFORMATION) &&
            pose_cost != static_cast<double>(costmap_2d::FREE_SPACE))
        {
            const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
            const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
            std::log(pose_cost / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

            if (min_distance_to_obstacle < cost_scaling_dist_) {
                cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
            }
        }

        // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
        linear_vel = std::min(cost_vel, curvature_vel);
        linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

        // if the actual lookahead distance is shorter than requested, that means we're at the
        // end of the path. We'll scale linear velocity by error to slow to a smooth stop.
        // This expression is eq. to 
        //      (1) holding time to goal, t, constant using the theoretical
        //          lookahead distance and proposed velocity and 
        //      (2) using t with the actual lookahead
        //          distance to scale the velocity (e.g. t = lookahead / velocity, v = carrot / t).
        if (dist_error > 2.0 * costmap_->getResolution()) {
            double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
            double unbounded_vel = approach_vel * velocity_scaling;
            if (unbounded_vel < min_approach_linear_velocity_) {
                approach_vel = min_approach_linear_velocity_;
            } else {
                approach_vel *= velocity_scaling;
            }

            // Use the lowest velocity between approach and other constraints, if all overlapping
            linear_vel = std::min(linear_vel, approach_vel);
        }

        // Limit linear velocities to be valid
        linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
        linear_vel = sign * linear_vel;
    }


    /**
     * Calculation methods 
     */
    
    //Get lookahead point on the global plan
    geometry_msgs::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
        const double & lookahead_dist, const nav_msgs::Path & transformed_plan)
    {
        // Find the first pose which is at a distance greater than the lookahead distance
        auto goal_pose_it = std::find_if(
            transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) 
        {
            return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
        });

        // If the number of poses is not far enough, take the last pose
        if (goal_pose_it == transformed_plan.poses.end()) {
            goal_pose_it = std::prev(transformed_plan.poses.end());
        }

        return *goal_pose_it;
    }


    double RegulatedPurePursuitController::getLookAheadDistance(
        const geometry_msgs::Twist & speed)
    {
        // If using velocity-scaled look ahead distances, find and clamp the dist
        // Else, use the static look ahead distance
        double lookahead_dist = lookahead_dist_;
        if (use_velocity_scaled_lookahead_dist_) {
            lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
            lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
        }


        return lookahead_dist;
    }


    //Transform the global plan to the frame of the robot
    nav_msgs::Path RegulatedPurePursuitController::transformGlobalPlan(
        const geometry_msgs::PoseStamped & pose){
        if (global_plan_.poses.empty()) {
            ROS_WARN("Local Planner received plan with zero length");
        }


        // let's get the pose of the robot in the frame of the plan
        geometry_msgs::PoseStamped robot_pose;
        if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
            ROS_ERROR("Unable to transform robot pose into global plan's frame");
        }

        // We'll discard points on the plan that are outside the local costmap
        double max_costmap_extent = getCostmapMaxExtent();

        auto closest_pose_upper_bound =
            nav2_util::geometry_utils::first_after_integrated_distance(
                global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

        // First find the closest pose on the path to the robot
        // bounded by when the path turns around (if it does) so we don't get a pose from a later
        // portion of the path
        auto transformation_begin =
            nav2_util::geometry_utils::min_by(
                global_plan_.poses.begin(), closest_pose_upper_bound,
                [&robot_pose](const geometry_msgs::PoseStamped & ps) 
                {
                    return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
                }
            );

        // Find points up to max_transform_dist so we only transform them.
        auto transformation_end = std::find_if(
            transformation_begin, global_plan_.poses.end(),
            [&](const auto & pose) {
                return nav2_util::geometry_utils::euclidean_distance(pose, robot_pose) > max_costmap_extent;
            });


        // Lambda to transform a PoseStamped from global frame to local
        auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
            geometry_msgs::PoseStamped stamped_pose, transformed_pose;
            stamped_pose.header.frame_id = global_plan_.header.frame_id;
            stamped_pose.header.stamp = robot_pose.header.stamp;

            stamped_pose.pose = global_plan_pose.pose;

            transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
            return transformed_pose;
        };


        // Transform the near part of the global plan into the robot's frame of reference.
        nav_msgs::Path transformed_plan;

        std::transform(
            transformation_begin, transformation_end,
            std::back_inserter(transformed_plan.poses),
            transformGlobalPoseToLocal);

        transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
        transformed_plan.header.stamp = robot_pose.header.stamp;

        // Remove the portion of the global plan that we've already passed so we don't
        // process it on the next iteration (this is called path pruning)
        global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
        global_path_pub_.publish(transformed_plan);

        if (transformed_plan.poses.empty()) {
            ROS_ERROR("Resulting plan has 0 poses in it.");
        }

        return transformed_plan;
    }


    bool RegulatedPurePursuitController::transformPose(
        const std::string frame,
        const geometry_msgs::PoseStamped & in_pose,
        geometry_msgs::PoseStamped & out_pose) const
    {
        if (in_pose.header.frame_id == frame) {
            out_pose = in_pose;
            return true;
        }

        try {
            tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
            out_pose.header.frame_id = frame;
            return true;
        } 
        catch (tf2::TransformException & ex) {
            ROS_ERROR("Exception in transformPose: %s", ex.what());
        }
        return false;
    }


    double RegulatedPurePursuitController::costAtPose(const double & x, const double & y)
    {
        unsigned int mx, my;

        if (!costmap_->worldToMap(x, y, mx, my)) {
            ROS_ERROR("RegulatedPurePursuitController: Dimensions of the costmap are too small "
                        "to encapsulate the robot footprint at current speeds!");
        }

        unsigned char cost = costmap_->getCost(mx, my);
        return static_cast<double>(cost);
    }


    bool RegulatedPurePursuitController::inCollision(
        const double & x,
        const double & y,
        const double & theta)
    {
        unsigned int mx, my;

        if (!costmap_->worldToMap(x, y, mx, my)) {
            ROS_WARN_THROTTLE(1.0, "The dimensions of the costmap is too small to successfully check for "
            "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
            "increase your costmap size.");
            return false;
        }

        double footprint_cost = costmap_model_->footprintCost(
            x, y, theta, costmap_ros_->getRobotFootprint());

        if (footprint_cost == static_cast<double>(costmap_2d::NO_INFORMATION) &&
            costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
        {
            return false;
        }

        // if occupied or unknown and not to traverse unknown space
        return footprint_cost >= static_cast<double>(costmap_2d::LETHAL_OBSTACLE);
    }


    bool RegulatedPurePursuitController::isCollisionImminent(
        const geometry_msgs::PoseStamped & robot_pose,
        const double & linear_vel, const double & angular_vel,
        const double & carrot_dist)
    {
        // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
        // odom frame and the carrot_pose is in robot base frame.

        // check current point is OK
        if (inCollision(
                robot_pose.pose.position.x, robot_pose.pose.position.y,
                tf2::getYaw(robot_pose.pose.orientation)))
        {
            return true;
        }

        // visualization messages
        nav_msgs::Path arc_pts_msg;
        arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
        arc_pts_msg.header.stamp = robot_pose.header.stamp;
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
        pose_msg.header.stamp = arc_pts_msg.header.stamp;

        double projection_time = 0.0;
        if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
            // rotating to heading at goal or toward path
            // Equation finds the angular distance required for the largest
            // part of the robot radius to move to another costmap cell:
            // theta_min = 2.0 * sin ((res/2) / r_max)
            // via isosceles triangle r_max-r_max-resolution,
            // dividing by angular_velocity gives us a timestep.
            double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
            projection_time =
            2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
        } else {
            // Normal path tracking
            projection_time = costmap_->getResolution() / fabs(linear_vel);
        }

        const geometry_msgs::Point & robot_xy = robot_pose.pose.position;
        geometry_msgs::Pose2D curr_pose;
        curr_pose.x = robot_pose.pose.position.x;
        curr_pose.y = robot_pose.pose.position.y;
        curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

        // only forward simulate within time requested
        int i = 1;
        while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_) {
            i++;

            // apply velocity at curr_pose over distance
            curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
            curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
            curr_pose.theta += projection_time * angular_vel;

            // check if past carrot pose, where no longer a thoughtfully valid command
            if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist) {
                break;
            }

            // store it for visualization
            pose_msg.pose.position.x = curr_pose.x;
            pose_msg.pose.position.y = curr_pose.y;
            pose_msg.pose.position.z = 0.01;
            arc_pts_msg.poses.push_back(pose_msg);

            // check for collision at the projected pose
            if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
                carrot_arc_pub_.publish(arc_pts_msg);
                return true;
            }
        }

        carrot_arc_pub_.publish(arc_pts_msg);

        return false;
    }


    void RegulatedPurePursuitController::checkGoalReached(geometry_msgs::PoseStamped& pose){

        double dx = global_plan_.poses.back().pose.position.x - pose.pose.position.x;
        double dy = global_plan_.poses.back().pose.position.y - pose.pose.position.y;
        if (std::fabs(std::sqrt(dx * dx + dy * dy)) < goal_dist_tol_){
            goal_reached_ = true;
        }

    }

    /**
     * Helper methods
     */


    void RegulatedPurePursuitController::createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path){

        path.header = plan[0].header;
        for (int i = 0; i < plan.size(); i++){
            path.poses.push_back(plan[i]);
        }
    }

    geometry_msgs::PointStamped RegulatedPurePursuitController::createCarrotMsg( const geometry_msgs::PoseStamped & carrot_pose)
    {
        geometry_msgs::PointStamped carrot_msg ;
        carrot_msg.header = carrot_pose.header;
        carrot_msg.point.x = carrot_pose.pose.position.x;
        carrot_msg.point.y = carrot_pose.pose.position.y;
        carrot_msg.point.z = 0.01;  // publish right over map to stand out
        return carrot_msg;
    }

    //Get the size of the costmap
    double RegulatedPurePursuitController::getCostmapMaxExtent() const
    {
        const double max_costmap_dim_meters = std::max(
            costmap_->getSizeInMetersX(), costmap_->getSizeInMetersX());

        return max_costmap_dim_meters / 2.0;
    }

    void RegulatedPurePursuitController::getRobotVel(geometry_msgs::Twist& speed){
        nav_msgs::Odometry robot_odom;

        odom_helper_.getOdom(robot_odom);

        speed.linear.x = robot_odom.twist.twist.linear.x;
        speed.angular.z = robot_odom.twist.twist.angular.z;
    }


}