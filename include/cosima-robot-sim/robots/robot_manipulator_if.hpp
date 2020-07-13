/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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

#include <bullet/LinearMath/btAlignedObjectArray.h>
#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>

// ROS
#include <sensor_msgs/JointState.h>

// ROS KDL PARSER includes
#include <kdl_parser/kdl_parser.hpp>

// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>

namespace cosima
{

    enum InterfaceType
    {
        Bullet = 0,
        Gazebo = 1
    };

    enum ControlModes
    {
        JointPosCtrl = 0,
        JointTrqCtrl = 1,
        JointGravComp = 2
    };

    class RobotManipulatorIF
    {
    public:
        RobotManipulatorIF(const std::string &name, RTT::TaskContext *tc);

        virtual bool configure();

        virtual void sense() = 0;
        virtual void act() = 0;

        virtual bool setControlMode(const std::string &controlMode) = 0;
        // virtual bool setActiveKinematicChain(const std::vector<std::string> &jointNames) = 0;
        virtual void readFromOrocos() = 0;
        virtual void writeToOrocos() = 0;

        bool defineKinematicChain(const std::string &urdf, const std::string &chain_root_link_name, const std::string &chain_tip_link_name);

        virtual bool setBasePosition(const double& x, const double& y, const double& z) = 0;

        virtual InterfaceType getInterfaceType() = 0;

    protected:
        RTT::TaskContext *tc;
        std::string robot_name;

        std::string robot_urdf;
        std::string robot_chain_base_link;
        std::string robot_chain_tip_link;

        int num_joints;
        std::map<std::string, int> map_joint_names_2_indices;
        std::vector<unsigned int> vec_joint_indices;

        ControlModes active_control_mode;
        ControlModes requested_control_mode;

        std::string getName();

        // Orocos Ports
        RTT::InputPort<Eigen::VectorXd> in_JointPositionCtrl_cmd;
        Eigen::VectorXd in_JointPositionCtrl_cmd_var;
        RTT::FlowStatus in_JointPositionCtrl_cmd_flow;

        RTT::InputPort<Eigen::VectorXd> in_JointTorqueCtrl_cmd;
        Eigen::VectorXd in_JointTorqueCtrl_cmd_var;
        RTT::FlowStatus in_JointTorqueCtrl_cmd_flow;

        RTT::OutputPort<Eigen::VectorXd> out_gc_fdb;
        Eigen::VectorXd out_gc_fdb_var;

        RTT::OutputPort<Eigen::MatrixXd> out_inertia_fdb;
        Eigen::MatrixXd out_inertia_fdb_var;

        RTT::OutputPort<sensor_msgs::JointState> out_jointstate_fdb;
        sensor_msgs::JointState out_jointstate_fdb_var;

        // KDL
        KDL::Tree kdl_tree;
        KDL::Chain kdl_chain;
    };

} // namespace cosima