/*
 * kdl_wrapper.cpp
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

#include <kdl_wrapper/kdl_wrapper.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

KDLWrapper::KDLWrapper()
{
    n = ros::NodeHandle("~");

    ik_solver_vel = NULL;
    jnt_jac_solver = NULL;
    fk_solver_pos = NULL;
    fk_solver_acc = NULL;

    m_initialized = false;
}

KDLWrapper::~KDLWrapper()
{
    m_initialized = false;
    delete ik_solver_vel;
    delete jnt_jac_solver;
    delete fk_solver_pos;
    delete fk_solver_acc;
}

bool KDLWrapper::init(const std::string &chain_root, const std::string &chain_tip)
{
    if(m_initialized)
    {
        ROS_ERROR("Already initialized");
        return false;
    }

    KDL::Tree tree;
    if(!getTreeFromURDF(tree))
    {
        return false;
    }

    if(!tree.getChain(chain_root, chain_tip, m_chain))
    {
        ROS_ERROR("Error getting KDL chain");
        return false;
    }
    ROS_DEBUG("Number of segments: %d", m_chain.getNrOfSegments());
    ROS_DEBUG("Number of joints in chain: %d", m_chain.getNrOfJoints());

    //	m_InvVelSolver.reset( new KDL::ChainIkSolverVel_pinv(m_ArmChain, 0.0001, 300));
    ik_solver_vel = new KDL::ChainIkSolverVel_wdls(m_chain, 0.01);
    ik_solver_vel->setLambda(0.3);

    jnt_jac_solver = new KDL::ChainJntToJacSolver(m_chain);
    fk_solver_pos = new KDL::ChainFkSolverPos_recursive(m_chain);
    fk_solver_vel = new KDL::ChainFkSolverVel_recursive(m_chain);
    fk_solver_acc = new KDL::ChainFkSolverAcc_recursive(m_chain);

    ROS_DEBUG("Successfully initialized KDL chain");

    m_initialized = true;
    return true;
}

bool KDLWrapper::isInitialized()
{
    return m_initialized;
}

KDL::Chain KDLWrapper::getKDLChain()
{
    return m_chain;
}

bool KDLWrapper::getTreeFromURDF(KDL::Tree &tree)
{

    /// Get robot_description from ROS parameter server
    std::string param_name = "robot_description";
    std::string full_param_name;
    std::string xml_string;

    n.searchParam(param_name, full_param_name);
    if (n.hasParam(full_param_name))
    {
        n.getParam(full_param_name.c_str(), xml_string);
    }

    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
        n.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
        n.shutdown();
        return false;
    }
    ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

    /// Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        n.shutdown();
        return false;
    }
    ROS_DEBUG("Successfully parsed urdf file");

    if (!kdl_parser::treeFromUrdfModel(model, tree)){
        ROS_ERROR("Failed to construct kdl tree");
        n.shutdown();
        return false;
    }

    return true;

}
