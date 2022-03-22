# regulated_pure_pursuit_controller

This is a ROS1 port of the [ROS2 Local Planner plugin](https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html). They are conceptually the same, however the source code may differ due to the lack of similar API/functions within ROS1. Of course, that being said, we should all prepare to move to ROS2, yet a significant proportion of existing robots still utilise the ROS1 ecosystem, and since there is a lack of good pure pursuit planner sout there, this port could prove to be a viable local planner replacement.

The Parameters are the same, please refer to the (Nav2 Regulated Pure Pursuit Controller)[https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller] for more details