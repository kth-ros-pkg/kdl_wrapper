kdl_wrapper
===========

Wraps the kdl and kdl_parser packages for generating KDL kinematic chains from URDF
by taking as inputs the IDs of the root and tip of the kinematic chain.

It also initializes the following KDL solvers:
 - Inverse velocity solver (WDLS)
 - Forward joint to jacobian solver
 - Forward joint to pose solver
 - Forward joint to acceleration solver
