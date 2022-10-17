

Lab 1: ROS publisher and subscriber
###########

Overview
**********

In this laboratory we will create a system to collect data from a lidar to make the Turtlebot move in such a way as to avoid obstacles.

.. info:: 
  While following the step-by-step tutorials, please take your time to think about what you are doing and what happens in each step, with the help of Google if necessary.
  
Creating a ros package
**********

Software in ROS is organized in packages. A package might contain ROS nodes, a ROS-independent library, a dataset, configuration files, a third-party piece of software, or anything else that logically constitutes a useful module. 
Let then create a new ROS package.
First of all we open a terminal and source it:

.. code-block:: 

  $ cd turtlebot_ws
  $ source devel/setup.bash
  
Now we create the package in \textbf{src} folder:

.. code-block:: 

  $ cd src
  $ catkin_create_pkg turtlebot_package std_msgs roscpp sensor_msgs geometry_msgs
  
:code:`std_msgs` :code:`roscpp` :code:`sensor_msgs` :code:`geometry_msgs` are the dependencies you will need in this package for later. 

Creating a Publisher
**********

”Node” is the ROS term for an executable that is connected to the ROS network. Here we’ll create a
publisher node which will continually broadcast a message.
Let create a file in the package that you created for this course:

.. code-block:: 

  $ cd turtlebot_ws/
  $ source devel/setup.bash
  $ rosrun turtlebot_package publisher
  
In this file we will write the code related to the publisher's node:

.. code-block:: cpp

    #include "ros/ros.h"
    #include "type_msgs/Name.h"  // To fill this do a "rostopic info topicname"

    /**
     * This tutorial demonstrates simple sending of messages over the ROS system.
     */
     int main(int argc, char **argv) {
       // Initialize the ROS system and become a node.
       ros::init(argc, argv, "publisher");
       ros::NodeHandle nh;

       // Create a publisher object.
       ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/do a rostopic list to know which topic to use", 1000);

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

The code explained
========
First of all we want to send velocity commands to the turtlebot, so let's find out on which topic we have to publisher. In order to do that let's use the following command:


.. code-block:: cpp

    rostopic list

You should have a lot of topic names printed. The one that allows us to send velocity commands is "cmd_vel_mux/input/navi", we will reuse it later in the code. Now let's find out the type of this topic using:

.. code-block:: cpp

    rostopic info cmd_vel_mux/input/navi
 
 We find that this topic is of type geometry_msgs/Twist, we will have to add it in the includes. You can search online for more infos about it.

.. code-block:: cpp

    #include "ros/ros.h"

ros/ros.h is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system. 

.. code-block:: cpp

    #include <geometry_msgs/Twist.h>

This includes the geometry\_msgs/Twist.h message, which resides in the geometry\_msgs package which provides messages for common geometric primitives such as points, vectors, and poses. For more informations go to <http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html>


.. code-block:: cpp

    ros::init(argc, argv, "publisher")

Initialize ROS. This is also where we specify the name of our node. Node names must be unique in a running system. Here we use


.. code-block:: cpp

    ros::NodeHandle nh

Create a handle to this process node. The first NodeHandle created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using. 


.. code-block:: cpp

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000)

Tell the master that we are going to be publishing a message of type :code:`geometry_msgs/Twist` on the topic :code:`/cmd_vel_mux/input/navi`. This lets the master tell any nodes listening on :code:`/cmd_vel` that we are going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones. 


.. code-block:: cpp

   ros::Rate loop_rate(10)

A :code:`ros::Rate` object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to :code:`Rate::sleep()`, and sleep for the correct amount of time.
.. note::
    In this case we tell it we want to run at 10Hz.

.. code-block:: cpp

    geometry_msgs::Twist msg;
    msg.linear.x = double(0.3);
    msg.angular.z = double(0.1);

We create a message of type Twist that we fill with informations. Here 0.03 m/s for x and 0.03 m/s for the angular velocity. The other four fields of Twist for the linear and angular velocity are are ignored by turtlesim, and set to 0 by default. 

.. code-block:: cpp

   pub.publish(msg)

Now we actually broadcast the message to anyone who is connected. 

.. code-block:: cpp

   ROS_INFO_STREAM("Sending random velocity command:"
   << " linear=" << msg.linear.x
   << " angular=" << msg.angular.z);

:code:`ROS_INFO` and friends are our replacement for :code:`printf/cout`.

.. code-block:: cpp

   ros::spinOnce()

Calling :code:`ros::spinOnce()` here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have :code:`ros::spinOnce()` here, your callbacks would never get called. So, add it for good measure.

Creating a Subscriber
**********
Here we want to access the data from the laser. We again have to understand which topic it is and the type of it. Once you have found these infos you do :code:`rostopic echo /name_of_the_topic`.

Let's create from command line a new file, named *subscriber.cpp*.
Here's the template file you can use:

.. code-block:: cpp

  #include "ros/ros.h" 
  #include "type_msgs/Name.h"  // To fill this do a "rostopic info topicname"

  /**
   * This tutorial demonstrates simple receipt of messages over the ROS system.
   */
  void chatterCallback(const type_msgs::Name::ConstPtr& msg)
  {
    ROS_INFO("LaserScan (val1,val2)=(%f,%f)", msg->ranges[xxx], msg->ranges[xxx]);
  }

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("do a rostopic list to know which topic to use", 1000, chatterCallback); 
    ros::spin(); 
    return 0;
  }


The code explained
============================

.. code-block:: cpp

  #include <sensor_msgs/LaserScan.h>
  
We include the message LaseScan from sensor\_msgs package.

.. code-block:: cpp

  void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    ROS_INFO("LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
  }
  
This is the callback function that will get called when a new message has arrived on the :code:`/scan ` topic. 

.. code-block:: cpp

  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback); 
  
Subscribe to the /scan topic with the master. ROS will call the :code:`chatterCallback()` function whenever a new message arrives. The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive.

.. code-block:: cpp

  ros::spin()
  
Enters a loop, calling message callbacks as fast as possible.

Build the code
**********

You used *catkin_create_pkg* in a previous tutorial which created a package.xml and a CMakeLists.txt file for you.
The generated CMakeLists.txt should look like this (with modifications from the Creating Msgs and Srvs tutorial and unused comments and examples removed):

.. code-block:: pygments.lexers.make.CMakeLexer

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
**********
First you do a :code:`catkin_make` to compile everything.

Then open a terminal: 

.. code-block::
  
   $ cd turtlebot_ws/
   $ source devel/setup.bash
   $ roslaunch turtlebot_bringup minimal.launch


In an other terminal: 
!! Be carefull here because your robot should start moving !!

.. code-block::
  
   $ cd turtlebot_ws/
   $ source devel/setup.bash
   $ rosrun turtlebot_package publisher

