mkdir turtlebot_ws
cd turtlebot_ws
mkdir src
cd src
git clone --recursive https://github.com/newline-lab/turtlebot2-noetic.git
sudo apt install ros-noetic-ecl-core
sudo apt-get install python-catkin-tools
sudo apt-get install ros-noetic-kobuki*
sudo apt-get install libuvc-dev
sudo apt-get install ros-noetic-joy
sudo apt-get install ros-noetic-arbotix*
sudo apt-get install ros-noetic-vision-opencv 
sudo apt-get install pyqt5-dev-tools
sudo apt-get install libsensors4-dev
sudo apt-get install ros-noetic-urg-node
sudo chmod a+rw /dev/ttyACM0
cd ..
catkin_make
