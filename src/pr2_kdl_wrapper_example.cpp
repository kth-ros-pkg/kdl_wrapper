/*
 * pr2_kdl_wrapper_example.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: Francisco Vina
 */

/* Copyright (c) 2013, Francisco Vina, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <kdl_wrapper/kdl_wrapper.h>


int main(int argc, char *argv[])
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "pr2_kdl_wrapper_example");


    KDLWrapper pr2_kdl_wrapper;

    if(!pr2_kdl_wrapper.init("torso_lift_link", "r_gripper_tool_frame"))
    {
        ROS_ERROR("Error initiliazing pr2_kdl_wrapper");
    }

    KDL::JntArray q_in(7);
    q_in(0) = 0.0;
    q_in(1) = M_PI/2;
    q_in(2) = 0.0;
    q_in(3) = M_PI/4;
    q_in(4) = 0.0;
    q_in(5) = 0.0;
    q_in(6) = M_PI/3;


    KDL::Twist v_in;
    v_in.vel = KDL::Vector(0.0, 0.2, 0.2);
    v_in.rot = KDL::Vector(0.0, 0.0, 0.0);

    KDL::JntArray q_dot_out;
    ROS_INFO("Calculating inverse kinematics");
    pr2_kdl_wrapper.ik_solver_vel->setLambda(0.3);
    pr2_kdl_wrapper.ik_solver_vel->CartToJnt(q_in, v_in, q_dot_out);

    ROS_INFO("Output q_dot: (%f, %f, %f, %f, %f, %f, %f)",
             q_dot_out(0),
             q_dot_out(1),
             q_dot_out(2),
             q_dot_out(3),
             q_dot_out(4),
             q_dot_out(5),
             q_dot_out(6));

    return 0;
}
