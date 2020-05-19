/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>,
 *                       Enrico Mingo Hoffman <enrico.mingo@iit.it>,
 *                       Pouya Mohammadi <https://github.com/Pouya-moh>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <Eigen/Dense>
#include <iostream>

#include <mutex>

// Robot Manipulator Interface includes
#include "../robots/robot_manipulator_if.hpp"

// Gazebo-Specific includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ROS
#include <sensor_msgs/JointState.h>

// ROS KDL PARSER includes
#include <kdl_parser/kdl_parser.hpp>

// // KDL includes
// #include <kdl/tree.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl/chainfksolvervel_recursive.hpp>
// #include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainjnttojacdotsolver.hpp>

// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace cosima
{
    class RobotManipulatorGazebo : public RobotManipulatorIF
    {
    public:
        RobotManipulatorGazebo(const std::string &name, const unsigned int &model_id, RTT::TaskContext *tc);

        bool configure();
        void sense();
        void act();

        bool setControlMode(const std::string &controlMode);
        bool setActiveKinematicChain(const std::vector<std::string> &jointNames);
        void readFromOrocos();
        void writeToOrocos();

        // bool loadModel(const std::string &model_urdf);

    private:
        gazebo::physics::Joint_V vec_active_joints;

        gazebo::physics::ModelPtr model;

        gazebo::event::ConnectionPtr world_begin;
        gazebo::event::ConnectionPtr world_end;

        gazebo::physics::JointControllerPtr gazebo_position_joint_controller;
        bool initGazeboJointController();

        void WorldUpdateBegin();
        void WorldUpdateEnd();

        // // KDL Parser
        // KDL::Tree kdl_tree;
        // KDL::Chain kdl_chain;

        // std::vector<std::string> joint_names; // TODO

        // Helpers

        // double *zero_forces;
        // double *max_forces;
        // double *zero_accelerations;
        // double *target_positions;

        // Sense
        Eigen::VectorXd q;
        Eigen::VectorXd qd;
        Eigen::VectorXd tau;

        // double *gc;
        // double *M;

        // Act
        Eigen::VectorXd cmd_trq;
        Eigen::VectorXd cmd_pos;

        // Mutex
        std::mutex sync_read_mtx;
        std::mutex sync_write_mtx;
    };

} // namespace cosima