#ifndef REGULATED_PURE_PURSUIT_CONTROLLER_H
#define REGULATED_PURE_PURSUIT_CONTROLLER_H

#include <algorithm>

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include <tf2/utils.h>
#include <tf/tf.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <regulated_pure_pursuit_controller/geometry_utils.h>


namespace regulated_pure_pursuit_controller{
  class RegulatedPurePursuitController : public nav_core::BaseLocalPlanner{

  public:

    RegulatedPurePursuitController();

    ~RegulatedPurePursuitController(){
      delete costmap_model_;
    };

    /**
     * Initialization methods
     */

    void initParams(ros::NodeHandle& nh);

    void initPubSubSrv(ros::NodeHandle& nh);


    void initialize(std::string name, tf2_ros::Buffer* tf,
        costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * Inherited virtual methods from BaseLocalPlanner
     */

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

    bool checkGoalReached();

    const bool& isInitialized(){
      return initialized_;
    }

    /**
     * Calculation methods
     */

    bool shouldRotateToPath( const geometry_msgs::PoseStamped & carrot_pose, 
                              double & angle_to_path);

    bool shouldRotateToGoalHeading( const geometry_msgs::PoseStamped & carrot_pose);

    void rotateToHeading( double & linear_vel, double & angular_vel,
                          const double & angle_to_path, 
                          const geometry_msgs::Twist & curr_speed);

    void applyConstraints(
        const double & dist_error, const double & lookahead_dist,
        const double & curvature, const geometry_msgs::Twist & /*curr_speed*/,
        const double & pose_cost, double & linear_vel, double & sign);

    geometry_msgs::PoseStamped getLookAheadPoint(
      const double & lookahead_dist, const nav_msgs::Path & transformed_plan);
    
    double getLookAheadDistance( const geometry_msgs::Twist & speed);

    nav_msgs::Path transformGlobalPlan(const geometry_msgs::PoseStamped & pose);

    bool transformPose( const std::string frame, const geometry_msgs::PoseStamped & in_pose,
                        geometry_msgs::PoseStamped & out_pose) const;

    double costAtPose(const double & x, const double & y);

    bool inCollision( const double & x, const double & y,
                      const double & theta);

    bool isCollisionImminent(
        const geometry_msgs::PoseStamped & robot_pose,
        const double & linear_vel, const double & angular_vel,
        const double & carrot_dist);

    /**
     * @brief Check if goal is reached by pose within a tolerance of goal_dist_tol_
     * 
     * @param pose Pose to check against
     */
    void checkGoalReached(geometry_msgs::PoseStamped& pose);

    /**
     * Helper methods
     */

    /**
     * @brief Get the maximum extent of costmap (from costmap center along its edges to the edge)
     * 
     * @return double 
     */
    double getCostmapMaxExtent() const;

    double euclidean_distance(const geometry_msgs::PoseStamped& p1,
                              const geometry_msgs::PoseStamped& p2);

    void createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path);

    
    geometry_msgs::PointStamped createCarrotMsg( const geometry_msgs::PoseStamped & carrot_pose);

    void getRobotVel(geometry_msgs::Twist& speed);

    void setSpeedLimit(
        const double & speed_limit,
        const bool & percentage);

    private:
      bool initialized_{false}; //indication of whether program has initialized

      /**
       * User-defined params
       */
      std::string odom_topic_{"odom"};

      double max_robot_pose_search_dist_;

      //Lookahead
      double lookahead_time_;
      double lookahead_dist_;
      double min_lookahead_dist_, max_lookahead_dist_;
      bool use_velocity_scaled_lookahead_dist_;

      //Rotate to heading
      bool use_rotate_to_heading_;
      double rotate_to_heading_min_angle_;
      double rotate_to_heading_angular_vel_;
      double max_angular_accel_;

      //Regulated linear velocity scaling
      bool use_regulated_linear_velocity_scaling_;
      double desired_linear_vel_;
      double min_approach_linear_velocity_;
      double regulated_linear_scaling_min_radius_;
      double regulated_linear_scaling_min_speed_;


      //Inflation cost scaling (Limit velocity by proximity to obstacles)
      double use_cost_regulated_linear_velocity_scaling_;
      double inflation_cost_scaling_factor_;
      double cost_scaling_dist_;
      double cost_scaling_gain_;

      //Collision avoidance
      double max_allowed_time_to_collision_up_to_carrot_;

      //Tolerances
      double goal_dist_tol_{0.2};
      ros::Duration transform_tolerance_;
      //Control frequency
      double control_duration_;

      /**
       * Pointer to other ROS Objects
       */
      tf2_ros::Buffer* tf_;
      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      base_local_planner::CostmapModel* costmap_model_; //For retrieving robot footprint cost
      geometry_msgs::PoseStamped current_pose_;
      base_local_planner::OdometryHelperRos odom_helper_;

      // for visualisation, publishers of global and local plan
      /**
       * @brief 
       * 
       */
      ros::Publisher global_path_pub_, local_plan_pub_;
      ros::Publisher carrot_pub_;
      ros::Publisher carrot_arc_pub_;

      ros::Publisher local_cmd_vel_pub_;

      /**
       * Run-time variables
       */
      // std::vector<geometry_msgs::PoseStamped> global_plan_;
      nav_msgs::Path global_plan_;
      bool goal_reached_;


  };
};

#endif //REGULATED_PURE_PURSUIT_CONTROLLER_H