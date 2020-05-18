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

#include "../include/rtt-bullet-embedded/api/b3_capi_wrapper_no_gui.hpp"

// ROS
#include <sensor_msgs/JointState.h>

namespace cosima
{

enum ControlModes { JointPosCtrl = 0, JointTrqCtrl = 1, JointGravComp = 2 };

class RobotManipulator
{
public:
    RobotManipulator(const std::string &name, const unsigned int &model_id, std::shared_ptr<b3CApiWrapperNoGui> sim, RTT::TaskContext* tc);

    bool configure();
    // void update();
    void sense();
    void act();
    
    bool setControlMode(std::string controlMode);
    bool setActiveKinematicChain(const std::vector<std::string> &jointNames);
    void readFromOrocos();
    void writeToOrocos();

private:
    std::shared_ptr<b3CApiWrapperNoGui> sim;
    RTT::TaskContext* tc;
    int robot_id;
    std::string robot_name;

    int num_joints;
    int *joint_indices;
    std::map<std::string, int> map_joint_names_2_indices;
    std::vector<int> vec_joint_indices;

    // Helpers
    double *zero_forces;
    double *max_forces;
    double *zero_accelerations;
    double *target_positions;

    ControlModes active_control_mode;
    ControlModes requested_control_mode;

    std::string getName();

    // Sense
    double *q;
    double *qd;
    double *gc;
    double *M;

    // Act
    double *cmd_trq;
    double *cmd_pos;

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
};

} // namespace cosima