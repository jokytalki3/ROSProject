# test

waffle_pi in Gazebo with three cubes in red, green and yellow colour.

1. Run roscore
   -roscore

2. Launch project world in Gazebo
   -export TURTLEBOT3_MODEL=waffle_pi
   -roslaunch robot_project robotProject.launch

3. Execute RViz, can see camera view
   -export TURTLEBOT3_MODEL=waffle_pi
   -roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

4. Control using keyboard
   -export TURTLEBOT3_MODEL=waffle_pi
   -roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

5. run "python follower_opencv.py"
   It will pass image message to opencv object
