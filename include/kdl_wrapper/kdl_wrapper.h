/*
 * kdl_wrapper.h
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

#ifndef KDL_WRAPPER_H_
#define KDL_WRAPPER_H_

#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolveracc_recursive.hpp>

class KDLWrapper{
public:

    ros::NodeHandle n;
    KDL::ChainIkSolverVel_wdls *ik_solver_vel;
    KDL::ChainJntToJacSolver *jnt_jac_solver;
    KDL::ChainFkSolverPos *fk_solver_pos;
    KDL::ChainFkSolverAcc *fk_solver_acc;

    KDLWrapper();
    virtual ~KDLWrapper();


    // initialize the KDL chain and the kinematic solvers
    // chain_root: ID of the root link of the kinematic chain
    // chain_tip: ID of the tip link of the kinematic chain
    // returns true if initialized correctly
    bool init(const std::string &chain_root, const std::string &chain_tip);

    // returns true if the object was initialized correctly
    bool isInitialized();

    // returns the KDL chain
    KDL::Chain getKDLChain();

private:

    bool m_initialized;
    KDL::Chain m_chain;

    bool getTreeFromURDF(KDL::Tree &tree);


};


#endif /* KDL_WRAPPER_H_ */
