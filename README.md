# regulated_pure_pursuit_controller

This is a ROS1 port of the [ROS2 Local Planner plugin](https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html). They are mostly the same, however the source code may differ due to the lack of similar API/functions within ROS1. Of course, that being said, we should all prepare to move to ROS2, yet a significant proportion of existing robots still utilise the ROS1 ecosystem, and since there is a lack of good pure pursuit planners out there, this port could prove to be a viable local planner replacement.

The Parameters are the same, please refer to the [Nav2 Regulated Pure Pursuit Controller](https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller) for more details

# Disclaimer
In no way did I write the original algorithm/source code, this originally developed by [Shrijit Singh](https://www.linkedin.com/in/shrijitsingh99/) and [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at Samsung Research as part of the Nav2 working group.

---

# Dependencies

1. [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure)
2. [Move Base Flex](https://github.com/magazino/move_base_flex)


# Deviations from Nav2 Package

## `computeVelocityCommands(geometry_msgs::Twist &cmd_vel)`
In ROS2, the corresponding method is 

`computeVelocityCommands( const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * goal_checker)`.

Where the robot velocity(`speed`) is already supplied to the method, and the goal_checker already replacing the need for a `isGoalReached()` method.

In the ported version, we have to use the ROS1 `isGoalReached()` method to check for goals, and the robot velocity is obtained through the `base_local_planner::OdometryHelperRos` API.


## `setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)`

ROS2 uses `setPlan(const nav_msgs::msg::Path & path)`, so we have to convert the global plan to a nav_msgs::path message type for further processing.

## transformGlobalPlan

The original transformGlobalPlan from the Nav2 package when ported directly faced issues with extrapolation into the future when looking up the transform between `/odom` and `/map` frame. Therefore, the transformGlobalPlan method from TEB Local Planner has been adapted for use here as it provides a more reliable and faster way of transforming the global plan into the base frame of the robot.


## Added a parameter `max_angular_vel` to clamp the output angular velocity to a user-defined value.
