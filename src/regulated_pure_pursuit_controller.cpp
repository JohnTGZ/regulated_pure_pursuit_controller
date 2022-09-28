// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.h"
// #include "regulated_pure_pursuit/helper.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

// PLUGINLIB_DECLARE_CLASS has been changed to PLUGINLIB_EXPORT_CLASS in ROS Noetic
PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, mbf_costmap_core::CostmapController)
PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav_core::BaseLocalPlanner)

namespace regulated_pure_pursuit_controller
{

    RegulatedPurePursuitController::RegulatedPurePursuitController() : initialized_(false), odom_helper_("odom"), goal_reached_(false) {}

    // tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void RegulatedPurePursuitController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!isInitialized())
        {
            ros::NodeHandle pnh_("~/" + name);

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            global_frame_ = costmap_ros_->getGlobalFrameID();
            robot_base_frame_ = costmap_ros_->getBaseFrameID();

            costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

            initParams(pnh_);
            initPubSubSrv(pnh_);

            odom_helper_.setOdomTopic(odom_topic_);

            initialized_ = true;
            ROS_INFO("PurePursuitPlanner Initialized");
        }
        else
        {
            ROS_WARN("RegulatedPurePursuitController has already been initialized, doing nothing.");
        }
    }

    void RegulatedPurePursuitController::initPubSubSrv(ros::NodeHandle &nh)
    {
        global_path_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
        local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
        carrot_pub_ = nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);
        carrot_arc_pub_ = nh.advertise<nav_msgs::Path>("lookahead_collision_arc", 1);
        kink_pub_ = nh.advertise<geometry_msgs::PointStamped>("kink_point", 1);
    }

    void RegulatedPurePursuitController::initParams(ros::NodeHandle &nh)
    {
        nh.param<std::string>("odom_topic", odom_topic_, "odom");
        nh.param<double>("max_robot_pose_search_dist", max_robot_pose_search_dist_, getCostmapMaxExtent());
        nh.param<int>("min_global_plan_complete_size", min_global_plan_complete_size_, 20);
        nh.param<double>("global_plan_prune_distance", global_plan_prune_distance_, 1.0);

        // Lookahead
        nh.param<double>("lookahead_time", lookahead_time_, 1.5);
        nh.param<double>("lookahead_dist", lookahead_dist_, 0.25);
        nh.param<bool>("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_, false);
        nh.param<double>("min_lookahead_dist", min_lookahead_dist_, 0.3);
        nh.param<double>("max_lookahead_dist", max_lookahead_dist_, 0.9);

        // Used to keep track of the cost 
        nh.param<int>("deep_history_num", deep_history_num_, 50);

        // Rotate to heading param
        nh.param<bool>("use_rotate_to_heading", use_rotate_to_heading_, false);
        nh.param<double>("rotate_to_heading_min_angle", rotate_to_heading_min_angle_, 0.785);
        nh.param<double>("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_, 1.8);
        nh.param<double>("max_angular_accel", max_angular_accel_, 1.5);

        // Kinked parameters
        nh.param<double>("kink_angle_thresh", kink_angle_thresh_, 130);

        // Reversing
        nh.param<bool>("allow_reversing", allow_reversing_, false);
        if (use_rotate_to_heading_ && allow_reversing_)
        {
            ROS_WARN("Disabling reversing. Both use_rotate_to_heading and allow_reversing "
                     "parameter cannot be set to true. By default setting use_rotate_to_heading true");
            allow_reversing_ = false;
        }

        ros::NodeHandle normal_namespace;
        normal_namespace.param<double>("diff_drive_controller/linear/x/max_velocity", diff_drive_lin_val_, 0.2);
        nh.param<bool>("use_diff_drive_params_max_lin_vel", use_diff_drive_params_max_lin_vel_, false);

        ROS_WARN("[regulated] : The parameters in regulated pure pursuit are as follows: %d, %f", use_diff_drive_params_max_lin_vel_, diff_drive_lin_val_);

        // Speed
        nh.param<double>("desired_linear_vel", desired_linear_vel_, 0.5);
        nh.param<double>("max_angular_vel", max_angular_vel_, 1.5);
        nh.param<double>("min_approach_linear_velocity", min_approach_linear_velocity_, 0.05);

        // Regulated linear velocity scaling
        nh.param<bool>("use_regulated_linear_velocity_scaling", use_regulated_linear_velocity_scaling_, false);
        nh.param<double>("regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_, 0.9);
        nh.param<double>("regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_, 0.25);

        // Inflation cost scaling (Limit velocity by proximity to obstacles)
        nh.param<bool>("use_cost_regulated_linear_velocity_scaling", use_cost_regulated_linear_velocity_scaling_, true);
        nh.param<double>("inflation_cost_scaling_factor", inflation_cost_scaling_factor_, 3.0);
        nh.param<double>("cost_scaling_dist", cost_scaling_dist_, 0.6);
        nh.param<double>("cost_scaling_gain", cost_scaling_gain_, 1.0);
        if (inflation_cost_scaling_factor_ <= 0.0)
        {
            ROS_WARN("The value inflation_cost_scaling_factor is incorrectly set, "
                     "it should be >0. Disabling cost regulated linear velocity scaling.");
            use_cost_regulated_linear_velocity_scaling_ = false;
        }

        // Collision avoidance
        nh.param<double>("max_allowed_time_to_collision_up_to_carrot", max_allowed_time_to_collision_up_to_carrot_, 1.0);
        nh.param<double>("goal_dist_tol", goal_dist_tol_, 0.25);

        double control_frequency;
        nh.param<double>("control_frequency", control_frequency, 20);
        control_duration_ = 1.0 / control_frequency;

        double transform_tolerance;
        nh.param<double>("transform_tolerance", transform_tolerance, 0.1);
        transform_tolerance_ = ros::Duration(transform_tolerance);

        nh.param<bool>("get_alternate_lookahead_dist", get_alternate_lookahead_dist_, false);
        nh.param<bool>("always_prioritise_alternate_lookahead", always_prioritise_alternate_lookahead_, false);
        nh.param<double>("lookahead_adjustment_y_tol", lookahead_adjustment_y_tol_, 0.1);

        if (!get_alternate_lookahead_dist_ && always_prioritise_alternate_lookahead_)
        {
            ROS_WARN("[Regulated Pure Pursuit] : Unable to not get alternate lookahead dist, but prioritise the alternate lookahead");
        }

        // Ddynamic Reconfigure
        ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(nh));
        ddr_->registerVariable<double>("lookahead_time", &this->lookahead_time_, "", 0.0, 20.0);
        ddr_->registerVariable<double>("lookahead_dist", &this->lookahead_dist_, "", 0.0, 20.0);

        ddr_->registerVariable<int>("deep_history_num", &this->deep_history_num_, "The number of clear cost in the previous readings", 0, 100);

        ddr_->registerVariable<bool>("use_velocity_scaled_lookahead_dist", &this->use_velocity_scaled_lookahead_dist_);
        ddr_->registerVariable<double>("min_lookahead_dist", &this->min_lookahead_dist_, "", 0.0, 5.0);
        ddr_->registerVariable<double>("max_lookahead_dist", &this->max_lookahead_dist_, "", 0.0, 10.0);

        // Rotate to heading param
        ddr_->registerVariable<bool>("use_rotate_to_heading", &this->use_rotate_to_heading_);
        ddr_->registerVariable<double>("rotate_to_heading_min_angle", &this->rotate_to_heading_min_angle_, "", 0.0, 15.0);
        ddr_->registerVariable<double>("rotate_to_heading_angular_vel", &this->rotate_to_heading_angular_vel_, "", 0.0, 15.0);
        ddr_->registerVariable<double>("max_angular_accel", &this->max_angular_accel_, "", 0.0, 15.0);

        // Speed
        ddr_->registerVariable<double>("desired_linear_vel", &this->desired_linear_vel_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("max_angular_vel", &this->max_angular_vel_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("min_approach_linear_velocity", &this->min_approach_linear_velocity_, "", 0.0, 10.0);

        // Regulated linear velocity scaling
        ddr_->registerVariable<bool>("use_regulated_linear_velocity_scaling", &this->use_regulated_linear_velocity_scaling_);
        ddr_->registerVariable<double>("regulated_linear_scaling_min_radius", &this->regulated_linear_scaling_min_radius_, "", 0.0, 5.0);
        ddr_->registerVariable<double>("regulated_linear_scaling_min_speed", &this->regulated_linear_scaling_min_speed_, "", 0.0, 5.0);

        // Inflation cost scaling (Limit velocity by proximity to obstacles)
        ddr_->registerVariable<bool>("use_cost_regulated_linear_velocity_scaling", &this->use_cost_regulated_linear_velocity_scaling_);
        ddr_->registerVariable<double>("inflation_cost_scaling_factor", &this->inflation_cost_scaling_factor_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("cost_scaling_dist", &this->cost_scaling_dist_, "", 0.0, 10.0);
        ddr_->registerVariable<double>("cost_scaling_gain", &this->cost_scaling_gain_, "", 0.0, 10.0);

        ddr_->registerVariable<bool>("get_alternate_lookahead_dist", &this->get_alternate_lookahead_dist_, "", false);
        ddr_->registerVariable<bool>("always_prioritise_alternate_lookahead", &this->always_prioritise_alternate_lookahead_, "", false);
        ddr_->registerVariable<double>("lookahead_adjustment_y_tol", &this->lookahead_adjustment_y_tol_, "", 0.0, 1.0);


        // Collision avoidance
        ddr_->registerVariable<double>("max_allowed_time_to_collision_up_to_carrot", &this->max_allowed_time_to_collision_up_to_carrot_, "", 0.0, 20.0);
        ddr_->registerVariable<double>("goal_dist_tol", &this->goal_dist_tol_, "", 0.0, 4.0);
        ddr_->registerVariable<double>("kink_angle_thresh", &this->kink_angle_thresh_, "", 0.0, 180.0);

        ddr_->publishServicesTopics();
    }

    bool RegulatedPurePursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // Store the global plan that was passed in, into a global container
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        goal_reached_ = false;
        return true;
    }

    uint32_t RegulatedPurePursuitController::computeVelocityCommands(const geometry_msgs::PoseStamped &pose, const geometry_msgs::TwistStamped &velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message)
    {
        if (!initialized_)
        {
            ROS_ERROR("RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
            message = "RegulatedPurePursuitController has not been initialized";
            return mbf_msgs::ExePathResult::NOT_INITIALIZED;
        }

        // Update the header of the cmd_vel message
        updateHeaderOfCmdVel(cmd_vel);

        // Set the goal reached to false
        goal_reached_ = false;

        // Get current pose of robot in
        geometry_msgs::PoseStamped robot_pose;
        costmap_ros_->getRobotPose(robot_pose);

        // Get robot velocity
        geometry_msgs::Twist speed;
        getRobotVel(speed);

        // prune global plan to cut off parts of the past (spatially before the robot)
        pruneGlobalPlan(*tf_, robot_pose, global_plan_, 1.0);

        // Transform global plan to the frame of interest (w.r.t. the local costmap)
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        int goal_idx;
        geometry_msgs::TransformStamped tf_plan_to_robot_frame;
        if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, robot_base_frame_, max_lookahead_dist_, transformed_plan, &goal_idx, &tf_plan_to_robot_frame))
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            message = "Could not transform the global plan to the frame of the controller";
            return mbf_msgs::ExePathResult::INTERNAL_ERROR;
        }

        // check if global goal is reached
        geometry_msgs::PoseStamped global_goal;
        tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_robot_frame);
        double dx_2 = global_goal.pose.position.x * global_goal.pose.position.x;
        double dy_2 = global_goal.pose.position.y * global_goal.pose.position.y;

        if (fabs(std::sqrt(dx_2 + dy_2)) < goal_dist_tol_ && global_plan_.size() <= min_global_plan_complete_size_)
        {
            goal_reached_ = true;
            return mbf_msgs::ExePathResult::SUCCESS;
        }

        // Return false if the transformed global plan is empty
        if (transformed_plan.empty())
        {
            ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
            message = "Transformed plan is empty";
            return mbf_msgs::ExePathResult::INVALID_PATH;
        }

        // Dynamically adjust look ahead distance based on the speed
        double lookahead_dist = getLookAheadDistance(speed);

        // Get lookahead point and publish for visualization
        geometry_msgs::PoseStamped carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
        if (fabs(carrot_pose.pose.position.y) < lookahead_adjustment_y_tol_ && fabs(carrot_pose.pose.position.x) > 0.0)
        {
        }
        else
        {
            // ROS_ERROR("During turns restrict the lookahead");
            // Set the lookahead distance to a minimum if the pose is far left or right
            double shortened_lookahead_dist = min_lookahead_dist_;
            carrot_pose = getLookAheadPoint(shortened_lookahead_dist, transformed_plan);
        }

        geometry_msgs::PointStamped kink_message;
        if (get_alternate_lookahead_dist_ && getAlternateKinkLookAheadDistance(transformed_plan, kink_message))
        {
            double kinked_dist = std::hypot(kink_message.point.x, kink_message.point.y);
            double carrot_dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
            if (always_prioritise_alternate_lookahead_ || kinked_dist < carrot_dist || kinkedIsPosWhileCarrotIsNeg(kink_message, carrot_pose))
            {
                carrot_pose.pose.position.x = kink_message.point.x;
                carrot_pose.pose.position.y = kink_message.point.y;
                carrot_pose.pose.position.z = kink_message.point.z;
            }
        }
        carrot_pub_.publish(createCarrotMsg(carrot_pose));

        // Carrot distance squared
        const double carrot_dist2 = (carrot_pose.pose.position.x * carrot_pose.pose.position.x) + (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

        // Find curvature of circle (k = 1 / R)
        double curvature = 0.0;
        if (carrot_dist2 > 0.001)
        {
            curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
        }

        // Set reversal
        double sign = 1.0;
        if (allow_reversing_)
        {
            sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
        }

        double linear_vel, angular_vel;
        linear_vel = desired_linear_vel_;

        // Make sure we're in compliance with basic constraints
        double angle_to_heading;

        if (shouldRotateToGoalHeading(carrot_pose))
        {
            // If robot should use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_
            double angle_to_goal = tf2::getYaw(transformed_plan.back().pose.orientation);
            rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
        }
        else if (shouldRotateToPath(carrot_pose, angle_to_heading))
        {
            // If robot should use_rotate_to_heading_ && angle_to_heading > rotate_to_heading_min_angle_
            rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
        }
        else
        {
            // Travel forward and accordinging to the curvature
            // ROS_INFO("Simply respecting the curvature of the path");
            // Constrain linear velocity
            applyConstraints(std::fabs(lookahead_dist - sqrt(carrot_dist2)), lookahead_dist, curvature, speed, costAtPose(robot_pose.pose.position.x, robot_pose.pose.position.y), linear_vel, sign);

            // Apply curvature to angular velocity after constraining linear velocity
            angular_vel = linear_vel * curvature;

            // Ensure that angular_vel does not exceed user-defined amount
            angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        }

        // ROS_ERROR("The angle of the path is: %f", std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x));

        // Collision checking on this velocity heading
        //  TODO: Update this collision checking with the diff drive params minimum and maximum parameter speeds
        const double &carrot_dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
        if (isCollisionImminent(robot_pose, linear_vel, angular_vel, carrot_dist))
        {
            ROS_WARN_THROTTLE(5, "[RegulatedPurePursuitController] : Detected collision ahead!");
            linear_vel = 0.0;
            angular_vel = 0.0;
        }

        // populate and return message
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return mbf_msgs::ExePathResult::SUCCESS;
    }

    bool RegulatedPurePursuitController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        std::string dummy_message;
        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
        uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
        cmd_vel = cmd_vel_stamped.twist;
        return outcome == mbf_msgs::ExePathResult::SUCCESS;
    }

    bool RegulatedPurePursuitController::isGoalReached()
    {
        if (goal_reached_)
        {
            ROS_INFO("[RegulatedPurePursuitController] Goal Reached!");
            return true;
        }
        return false;
    }

    bool RegulatedPurePursuitController::shouldRotateToPath(const geometry_msgs::PoseStamped &carrot_pose, double &angle_to_path)
    {
        // Whether we should rotate robot to rough path heading
        angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
        bool answer = (use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_);
        // ROS_INFO("[Regulated Pure Pursuit] : The answer of should RotateToPath is: %d, because of use_rotate_to_heading: %d and the angle to path: %f, and the min_angle: %f", answer, use_rotate_to_heading_, fabs(angle_to_path), rotate_to_heading_min_angle_);
        return answer;
    }

    bool RegulatedPurePursuitController::shouldRotateToGoalHeading(const geometry_msgs::PoseStamped &carrot_pose)
    {
        // Whether we should rotate robot to goal heading
        double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
        bool answer = (use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_);
        // ROS_INFO("[Regulated Pure Pursuit] : The answer of should shouldRotateToGoalHeading is: %d, because of use_rotate_to_heading: %d and the distance to goal: %f, and the goal distance: %f", answer, use_rotate_to_heading_, fabs(dist_to_goal), goal_dist_tol_);
        return answer;
    }

    void RegulatedPurePursuitController::rotateToHeading(double &linear_vel, double &angular_vel, const double &angle_to_path, const geometry_msgs::Twist &curr_speed)
    {
        // Rotate in place using max angular velocity / acceleration possible
        linear_vel = 0.0;
        const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
        angular_vel = sign * rotate_to_heading_angular_vel_;

        const double &dt = control_duration_;
        const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
        const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
        angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
    }

    void RegulatedPurePursuitController::applyConstraints(const double &dist_error, const double &lookahead_dist, const double &curvature, const geometry_msgs::Twist & /*curr_speed*/, const double &pose_cost, double &linear_vel, double &sign)
    {
        double curvature_vel = linear_vel;
        double cost_vel = linear_vel;
        double approach_vel = linear_vel;

        // limit the linear velocity by curvature
        const double radius = fabs(1.0 / curvature);
        const double &min_rad = regulated_linear_scaling_min_radius_;

        if (use_regulated_linear_velocity_scaling_ && radius < min_rad)
        {
            curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
        }

        // limit the linear velocity by proximity to obstacles
        if (use_cost_regulated_linear_velocity_scaling_ &&
            pose_cost != static_cast<double>(costmap_2d::NO_INFORMATION) &&
            pose_cost != static_cast<double>(costmap_2d::FREE_SPACE))
        {
            const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
            const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
                                                        std::log(pose_cost / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) +
                                                    inscribed_radius;

            if (min_distance_to_obstacle < cost_scaling_dist_)
            {
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
        if (dist_error > 2.0 * costmap_->getResolution())
        {
            double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
            double unbounded_vel = approach_vel * velocity_scaling;
            if (unbounded_vel < min_approach_linear_velocity_)
            {
                approach_vel = min_approach_linear_velocity_;
            }
            else
            {
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

    // Get lookahead point on the global plan
    geometry_msgs::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(const double &lookahead_dist, const std::vector<geometry_msgs::PoseStamped> &transformed_plan)
    {
        // Find the first pose which is at a distance greater than the lookahead distance
        auto goal_pose_it = std::find_if(transformed_plan.begin(), transformed_plan.end(), [&](const auto &ps)
                                         { return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist; });

        // If the number of poses is not far enough, take the last pose
        if (goal_pose_it == transformed_plan.end())
        {
            goal_pose_it = std::prev(transformed_plan.end());
        }

        return *goal_pose_it;
    }

    bool RegulatedPurePursuitController::getAlternateKinkLookAheadDistance(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, geometry_msgs::PointStamped &kink_message)
    {
        // This is for debugging purposes
        nav_msgs::Path dummy_path; 

        int number_of_odd_elements = 13;
        int half = number_of_odd_elements / 2;
        int full = half * 2;

        // Check the first point thru third last point to find a kink
        for (unsigned int i = 0; i < transformed_plan.size() - number_of_odd_elements - 1; i++)
        {
            dummy_path.poses.push_back(transformed_plan[i]);

            // Obtain the angle of the path
            RegulatedPurePursuitHelper regulated_helper_instance(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, transformed_plan[i + half].pose.position.x, transformed_plan[i + half].pose.position.y, transformed_plan[i + full].pose.position.x, transformed_plan[i + full].pose.position.y);
            double angle_in_degrees = regulated_helper_instance.inverseCosineVectorinDegrees();
            
            if (angle_in_degrees < kink_angle_thresh_)
            {
                // publish a pose
                kink_message.header.stamp = ros::Time::now();
                kink_message.header.frame_id = transformed_plan[i + half].header.frame_id;
                kink_message.point.x = transformed_plan[i + half].pose.position.x;
                kink_message.point.y = transformed_plan[i + half].pose.position.y;
                kink_message.point.z = 0.0;
                kink_pub_.publish(kink_message);
                return true;
            }
        }

        dummy_path.header.frame_id = "base_link";
        dummy_path.header.stamp = ros::Time::now();
        global_path_pub_.publish(dummy_path);
        return false;
    }

    double RegulatedPurePursuitController::getLookAheadDistance(const geometry_msgs::Twist &speed)
    {
        // If using velocity-scaled look ahead distances, find and clamp the dist
        // Else, use the static look ahead distance
        double lookahead_dist = lookahead_dist_;
        if (use_velocity_scaled_lookahead_dist_)
        {
            if (use_diff_drive_params_max_lin_vel_)
            {
                if (fabs(speed.linear.x) >= diff_drive_lin_val_ - 0.05)
                {
                    // ROS_ERROR("Activating maximum lookahead since the msaximum speed is crossed");
                    lookahead_dist = max_lookahead_dist_;
                    return lookahead_dist;
                }
            }
            lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
            lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
        }
        return lookahead_dist;
    }

    bool RegulatedPurePursuitController::transformGlobalPlan(const tf2_ros::Buffer &tf, 
                                                            const std::vector<geometry_msgs::PoseStamped> &global_plan, 
                                                            const geometry_msgs::PoseStamped &global_pose, 
                                                            const costmap_2d::Costmap2D &costmap, 
                                                            const std::string &robot_base_frame, 
                                                            double max_plan_length, 
                                                            std::vector<geometry_msgs::PoseStamped> &transformed_plan, 
                                                            int *current_goal_idx, 
                                                            geometry_msgs::TransformStamped *tf_plan_to_robot_frame)
    {
        // this method is a slightly modified version of base_local_planner/goal_functions.h
        const geometry_msgs::PoseStamped &plan_pose = global_plan[0];
        transformed_plan.clear();

        try
        {
            if (global_plan_.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // if (!tf.canTransform(robot_base_frame, plan_pose.header.frame_id, plan_pose.header.stamp, transform_tolerance_ )){
            //     return false;
            // }
            // get plan_to_robot_transform from plan frame to robot frame
            geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(robot_base_frame, global_pose.header.stamp,
                                                                                         plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                         plan_pose.header.frame_id, transform_tolerance_);

            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

            // we'll discard points on the plan that are outside the local costmap
            double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                             costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacles that are
                                    // located on the border of the local costmap

            int i = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = 1e10;

            // we need to loop to a point on the plan that is within a certain distance of the robot
            for (int j = 0; j < (int)global_plan.size(); ++j)
            {
                double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
                double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
                if (new_sq_dist > sq_dist_threshold)
                    break; // force stop if we have reached the costmap border

                if (new_sq_dist < sq_dist) // find closest distance
                {
                    sq_dist = new_sq_dist;
                    i = j;
                }
            }

            geometry_msgs::PoseStamped newer_pose;

            double plan_length = 0; // check cumulative Euclidean distance along the plan

            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() &&
                   sq_dist <= sq_dist_threshold &&
                   (max_plan_length <= 0 || plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped &pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                // Calculate distance to previous pose
                if (i > 0 && max_plan_length > 0)
                {
                    plan_length += std::sqrt(std::pow(global_plan[i].pose.position.x - global_plan[i - 1].pose.position.x, 2) +
                                             std::pow(global_plan[i].pose.position.y - global_plan[i - 1].pose.position.y, 2));
                }

                ++i;
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
            }

            // Return the transformation from the global plan to the global planning frame if desired
            if (tf_plan_to_robot_frame)
                *tf_plan_to_robot_frame = plan_to_robot_transform;
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Robot Frame: %s Plan Frame size %d: %s\n", robot_base_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }

    bool RegulatedPurePursuitController::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
    {
        if (global_plan.empty())
            return true;

        try
        {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform);

            double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end())
            {
                double dx = robot.pose.position.x - it->pose.position.x;
                double dy = robot.pose.position.y - it->pose.position.y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < dist_thresh_sq)
                {
                    erase_end = it;
                    break;
                }
                ++it;
            }
            if (erase_end == global_plan.end())
                return false;

            if (erase_end != global_plan.begin())
                global_plan.erase(global_plan.begin(), erase_end);
        }
        catch (const tf::TransformException &ex)
        {
            ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }

        return true;
    }

    double RegulatedPurePursuitController::costAtPose(const double &x, const double &y)
    {
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my))
        {
            ROS_ERROR("RegulatedPurePursuitController: Dimensions of the costmap are too small "
                      "to encapsulate the robot footprint at current speeds!");
        }

        // 0 to 255
        unsigned char cost = costmap_->getCost(mx, my);

        // Convert the unsigned char into a double
        return static_cast<double>(cost);
    }

    bool RegulatedPurePursuitController::inCollision(const double &x, const double &y, const double &theta)
    {
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my))
        {
            ROS_WARN_THROTTLE(1.0, "The dimensions of the costmap is too small to successfully check for "
                                   "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
                                   "increase your costmap size.");
            return false;
        }

        // The footprint here is the local robot footprint
        double footprint_cost = costmap_model_->footprintCost(x, y, theta, costmap_ros_->getRobotFootprint());
        if (footprint_cost == static_cast<double>(costmap_2d::NO_INFORMATION) && costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
        {
            return false;
        }

        int count = 0;
        for (int i = 0; i < footprint_cost_deep_history_.size(); i++)
        {
            if (footprint_cost_deep_history_[i] == -1.00)
            {
                count += 1;
            }
        }

        if (footprint_cost_deep_history_.size() >= deep_history_num_)
        {
            // In order to maintain a deep history, remove the first entry and update it with the new entry
            footprint_cost_deep_history_.erase(footprint_cost_deep_history_.begin());
            footprint_cost_deep_history_.push_back(footprint_cost);
        }
        else
        {
            footprint_cost_deep_history_.push_back(footprint_cost);
        }

        bool answer = (footprint_cost >= static_cast<double>(costmap_2d::LETHAL_OBSTACLE) || count > 0);
        return answer;
    }

    bool RegulatedPurePursuitController::isCollisionImminent(const geometry_msgs::PoseStamped &robot_pose, const double &linear_vel, const double &angular_vel, const double &carrot_dist)
    {
        // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
        // odom frame and the carrot_pose is in robot base frame.

        // check current point is OK
        if (inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y, tf2::getYaw(robot_pose.pose.orientation)))
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
        if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01)
        {
            // rotating to heading at goal or toward path
            // Equation finds the angular distance required for the largest
            // part of the robot radius to move to another costmap cell:
            // theta_min = 2.0 * sin ((res/2) / r_max)
            // via isosceles triangle r_max-r_max-resolution,
            // dividing by angular_velocity gives us a timestep.
            double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
            projection_time = 2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
        }
        else
        {
            // Normal path tracking
            projection_time = costmap_->getResolution() / fabs(linear_vel);
        }

        const geometry_msgs::Point &robot_xy = robot_pose.pose.position;
        geometry_msgs::Pose2D curr_pose;
        curr_pose.x = robot_pose.pose.position.x;
        curr_pose.y = robot_pose.pose.position.y;
        curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

        // only forward simulate within time requested
        int i = 1;
        while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_)
        {
            i++;

            // apply velocity at curr_pose over distance
            curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
            curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
            curr_pose.theta += projection_time * angular_vel;

            // check if past carrot pose, where no longer a thoughtfully valid command
            if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist)
            {
                break;
            }

            // store it for visualization
            pose_msg.pose.position.x = curr_pose.x;
            pose_msg.pose.position.y = curr_pose.y;
            pose_msg.pose.position.z = 0.01;
            arc_pts_msg.poses.push_back(pose_msg);

            // check for collision at the projected pose
            if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta))
            {
                carrot_arc_pub_.publish(arc_pts_msg);
                return true;
            }
        }

        carrot_arc_pub_.publish(arc_pts_msg);

        return false;
    }

    // Create path message based a given PoseStamped plan
    void RegulatedPurePursuitController::createPathMsg(const std::vector<geometry_msgs::PoseStamped> &plan, nav_msgs::Path &path)
    {
        path.header = plan[0].header;
        for (int i = 0; i < plan.size(); i++)
        {
            path.poses.push_back(plan[i]);
        }
    }

    // Create a carrot
    geometry_msgs::PointStamped RegulatedPurePursuitController::createCarrotMsg(const geometry_msgs::PoseStamped &carrot_pose)
    {
        geometry_msgs::PointStamped carrot_msg;
        carrot_msg.header = carrot_pose.header;
        carrot_msg.point.x = carrot_pose.pose.position.x;
        carrot_msg.point.y = carrot_pose.pose.position.y;
        carrot_msg.point.z = 0.01; // publish right over map to stand out
        return carrot_msg;
    }

    // Get the size of the costmap and divide it by 2
    double RegulatedPurePursuitController::getCostmapMaxExtent() const
    {
        const double max_costmap_dim_meters = std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersX());
        return max_costmap_dim_meters / 2.0;
    }

    void RegulatedPurePursuitController::getRobotVel(geometry_msgs::Twist &speed)
    {
        nav_msgs::Odometry robot_odom;
        odom_helper_.getOdom(robot_odom);
        speed.linear.x = robot_odom.twist.twist.linear.x;
        speed.angular.z = robot_odom.twist.twist.angular.z;
    }

    void RegulatedPurePursuitController::updateHeaderOfCmdVel(geometry_msgs::TwistStamped &cmd_vel)
    {
        // Defined as static so the memory address of this variable is maintained even when the function goes out of scope
        static uint32_t seq = 0;
        cmd_vel.header.seq = seq++;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = robot_base_frame_;
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    }

    bool RegulatedPurePursuitController::kinkedIsPosWhileCarrotIsNeg(const geometry_msgs::PointStamped& message_one, const geometry_msgs::PoseStamped& message_two)
    {
        if ((message_two.pose.position.x < 0.0 && message_one.point.x) > 0.0)
        {
            return true;
        }

        return false;
    }
}
