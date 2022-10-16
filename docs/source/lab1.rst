

Lab 1: ROS publisher and subscriber
========

Overview
#######

In this laboratory we will create a system to collect data from a lidar to make the Turtlebot move in such a way as to avoid obstacles.

*While following the step-by-step tutorials, please take your time to think about what you are doing and what happens in each step, with the help of Google if necessary.*

Creating a Publisher
#####
”Node” is the ROS term for an executable that is connected to the ROS network. Here we’ll create a
publisher node which will continually broadcast a message.
Let create a file in the package that you created for this course:
::
  $ touch publisher.cpp
In this file will write the code related to the publisher's node.
::
  #include "ros/ros.h"
  #include "geometry_msgs/Twist.h"  // For geometry_msgs::Twist

  /**
   * This tutorial demonstrates simple sending of messages over the ROS system.
   */
   int main(int argc, char **argv) {
     // Initialize the ROS system and become a node.
     ros::init(argc, argv, "publisher");
     ros::NodeHandle nh;

     // Create a publisher object.
     ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);

     // Loop at 10Hz until the node is shut down.
     ros::Rate rate(10);
     while(ros::ok()) {
       // Create and fill in the message.  The other four
       // fields, which are ignored by turtlesim, default to 0.
       geometry_msgs::Twist msg;
       msg.linear.x = double(0.03);
       msg.angular.z = double(0.03);

       // Publish the message.
       pub.publish(msg);

       // Send a message to rosout with the details.
       ROS_INFO_STREAM("Sending velocity command:"
         << " linear=" << msg.linear.x
         << " angular=" << msg.angular.z);

       ros::spinOnce();

       // Wait until it's time for another iteration.
       rate.sleep();
     }
   }

Creating a Subscriber
####

As before create from commandline a new file, named *subscriber.cpp*.
Here's the template file you can use:
::
  #include <ros/ros.h> 
  #include <sensor_msgs/LaserScan.h>

  /**
   * This tutorial demonstrates simple receipt of messages over the ROS system.
   */
  void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    ROS_INFO("LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
  }

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback); 
    ros::spin(); 
    return 0;
  }
    
Build the code
#####

You used *catkin_create_pkg* in a previous tutorial which created a package.xml and a CMakeLists.txt file for you.
The generated CMakeLists.txt should look like this (with modifications from the Creating Msgs and Srvs tutorial and unused comments and examples removed): 
::
  cmake_minimum_required(VERSION 2.8.3)
  project(turtlebot_package)

  ## Find catkin and any catkin packages
  find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs genmsg)

  ## Generate added messages and services
  generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

  ## Declare a catkin package
  catkin_package()

  ## Build publisher and subscriber
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(subscriber src/subscriber.cpp)
  target_link_libraries(subscriber ${catkin_LIBRARIES})
  add_dependencies(subscriber turtlebot_package_generate_messages_cpp)

  add_executable(publisher src/publisher.cpp)
  target_link_libraries(publisher ${catkin_LIBRARIES})
  add_dependencies(publisher turtlebot_package_generate_messages_cpp)

Examining the Simple Publisher and Subscriber
#####

Now you can do :bash:`catkin_make` to compile everything.

Make sure that a roscore is up and running launching :bash:`roscore`, :bash:`turtlebot_package subscriber` and then
::
  cd turtlebot_ws/
  source devel/setup.bash
  rosrun turtlebot_package publisher

Writing a obstacle avoidance script
#####

Now it's your turn, we would like you to write to own node. This node should be able to move the robot around with a linear velocity of 0.2 m/s and avoid obstacle by turning on itself with an angular velocity of 0.15 m/s. To get access to the obstacle you can use the node you wrote previously that gives you the data of the Hokuyo Laser scan mounted on the turtlebot.

