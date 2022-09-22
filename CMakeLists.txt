cmake_minimum_required(VERSION 2.8.3)
project(regulated_pure_pursuit_controller)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules
        base_local_planner
        costmap_2d
        nav_core
        nav_msgs
        pluginlib
        ddynamic_reconfigure
        tf
        mbf_msgs
        mbf_costmap_core
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES regulated_pure_pursuit_controller
        CATKIN_DEPENDS
        pluginlib
        roscpp
        base_local_planner
        costmap_2d
        nav_core
        nav_msgs
        pluginlib
        ddynamic_reconfigure
        tf
        mbf_msgs
        mbf_costmap_core
)

# The first argument in add_library is the name of the library
add_library(regulated_pure_pursuit_controller
        src/regulated_pure_pursuit_controller.cpp
        src/helper.cpp
)

target_link_libraries(regulated_pure_pursuit_controller 
                        ${catkin_LIBRARIES})
add_dependencies(regulated_pure_pursuit_controller nav_msgs_gencpp)

install(TARGETS regulated_pure_pursuit_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES regulated_pure_pursuit_controller.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/regulated_pure_pursuit_tests.cpp)
    target_link_libraries(${PROJECT_NAME}-test regulated_pure_pursuit_controller ${catkin_LIBRARIES} ${catkin_LIBRARIES})
 endif()
