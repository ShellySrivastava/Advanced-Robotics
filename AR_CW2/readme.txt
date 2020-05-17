Name: Shelly Srivastava Student Id: 190385633

To use the package 'AR_week8_test'
1) Copy the package AR_week8_test to the source folder in catkin workspace (~/ws_moveit/src)
2) Set up the environment. Run the following commands in the terminal
	cd ~/ws_moveit/src
	git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
	git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
3) Source the catkin workspace
	source ~/ws_moveit/devel/setup.bash
4) Compile the package using the following command
	cd ~/ws_moveit
	catkin_make
5) Run roscore
	roscore
6) Run the follwoing commands
	roslaunch panda_moveit_config demo.launch
	rosrun AR_week8_test square_size_generator.py
	rosrun AR_week8_test move_panda_square.py
	rosrun rqt_plot rqt_plot