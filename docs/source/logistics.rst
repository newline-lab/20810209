Logistics
========

Set up the Turlebot workspace
--------

Let's create and build a catkin workspace with all the turtlebot materials inside:

.. code-block:: pygments.lexers.shell.BashSessionLexer

   $ mkdir -p ~/turtlebot_ws/src
   $ git clone --recursive https://github.com/newline-lab/turtlebot2-noetic.git
   $ cd ..
   $ catkin_make

Due to missing packages, it will for sure it will surely happen that errors will come out. Let's go trought the printed logs and try to understande which packages are needed to (c)make it work.

.. toggle-header::
    :header: ****
   Download the :download:`install_ws.sh <files/install_ws.sh>` , extract and run is as executable:
   .. code-block:: pygments.lexers.shell.BashSessionLexer

      sudo chmod +x ws_install.sh
      ./ws_install.sh

Set up the Turlebot workspace
--------

Let's launch the turtlebot bringup:

.. code-block:: pygments.lexers.shell.BashSessionLexer
   
   $ roslaunch turtlebot_bringup minimal.launch

In an other terminal we can open the keyboard teleop to be able to move the robot around with the keyboard:

.. code-block:: pygments.lexers.shell.BashSessionLexer
   
   $ roslaunch turtlebot_teleop keyboard_teleop.launch

