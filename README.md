kdl_wrapper
===========
Overview
---------------------------------------------
Wraps the kdl and kdl_parser packages for generating KDL kinematic chains from URDF
by taking as inputs the IDs of the root and tip of the kinematic chain of a robot manipulator.

It also initializes the following KDL solvers:
 - Inverse velocity solver (WDLS)
 - Forward joint to jacobian solver
 - Forward joint to pose solver
 - Forward joint to acceleration solver

Requires the kdl_acc_solver package to be installed in the catkin workspace:
  
		git clone https://github.com/kth-ros-pkg/kdl_acc_solver.git


Example code
---------------------------------------------

You can run the example code under src/pr2_kdl_wrapper_example.cpp by doing the following:

1. Make sure you have the pr2_common metapackage:
     
		sudo apt-get install ros-<rosdistro>-pr2-common

2. Upload the PR2 URDF: 

		roslaunch pr2_description upload_pr2.launch kinect:=TRUE

3. Run the pr2_kdl_wrapper_example :

		rosrun kdl_wrapper pr2_kdl_wrapper_example
