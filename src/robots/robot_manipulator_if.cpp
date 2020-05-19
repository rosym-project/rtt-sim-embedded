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

#include "../include/cosima-robot-sim/robots/robot_manipulator_if.hpp"
#include <rtt/Logger.hpp>

#include <unistd.h>

#define PRELOG(X, T) (RTT::log(RTT::X) << "[" << (T->getName()) << ":" << this->robot_name << "] ")

using namespace cosima;
using namespace RTT;

RobotManipulatorIF::RobotManipulatorIF(const std::string &name, RTT::TaskContext *tc)
{
    // Initialize
    this->robot_name = name;
    this->active_control_mode = ControlModes::JointGravComp;
    this->requested_control_mode = ControlModes::JointPosCtrl;

    this->tc = tc;

    this->num_joints = 0;
}

bool RobotManipulatorIF::configure()
{
    if ((vec_joint_indices.size() <= 0) || (!this->tc))
    {
        return false;
    }
    this->num_joints = vec_joint_indices.size();

    // Add OROCOS RTT ports
    if (tc->getPort("in_JointPositionCtrl_cmd"))
    {
        tc->ports()->removePort("in_JointPositionCtrl_cmd");
    }
    in_JointPositionCtrl_cmd_var = Eigen::VectorXd::Zero(this->num_joints);
    in_JointPositionCtrl_cmd.setName("in_" + this->robot_name + "_JointPositionCtrl_cmd");
    in_JointPositionCtrl_cmd.doc("Input port for reading joint position commands");
    tc->ports()->addPort(in_JointPositionCtrl_cmd);
    in_JointPositionCtrl_cmd_flow = RTT::NoData;

    if (tc->getPort("in_JointTorqueCtrl_cmd"))
    {
        tc->ports()->removePort("in_JointTorqueCtrl_cmd");
    }
    in_JointTorqueCtrl_cmd_var = Eigen::VectorXd::Zero(this->num_joints);
    in_JointTorqueCtrl_cmd.setName("in_" + this->robot_name + "_JointTorqueCtrl_cmd");
    in_JointTorqueCtrl_cmd.doc("Input port for reading joint torque commands");
    tc->ports()->addPort(in_JointTorqueCtrl_cmd);
    in_JointTorqueCtrl_cmd_flow = RTT::NoData;

    if (tc->getPort("out_gc_fdb"))
    {
        tc->ports()->removePort("out_gc_fdb");
    }
    out_gc_fdb_var = Eigen::VectorXd::Zero(this->num_joints);
    out_gc_fdb.setName("out_" + this->robot_name + "_gc_fdb");
    out_gc_fdb.doc("Output port for sending joint space gravity and coriolis");
    out_gc_fdb.setDataSample(out_gc_fdb_var);
    tc->ports()->addPort(out_gc_fdb);

    if (tc->getPort("out_inertia_fdb"))
    {
        tc->ports()->removePort("out_inertia_fdb");
    }
    out_inertia_fdb_var = Eigen::MatrixXd::Zero(this->num_joints, this->num_joints);
    out_inertia_fdb.setName("out_" + this->robot_name + "_inertia_fdb");
    out_inertia_fdb.doc("Output port for sending joint space inertia matrix");
    out_inertia_fdb.setDataSample(out_inertia_fdb_var);
    tc->ports()->addPort(out_inertia_fdb);

    if (tc->getPort("out_jointstate_fdb"))
    {
        tc->ports()->removePort("out_jointstate_fdb");
    }
    out_jointstate_fdb_var.position.resize(this->num_joints);
    out_jointstate_fdb_var.velocity.resize(this->num_joints);
    out_jointstate_fdb_var.effort.resize(this->num_joints);
    for (unsigned int i = 0; i < this->num_joints; i++)
    {
        out_jointstate_fdb_var.position[i] = 0.0;
        out_jointstate_fdb_var.velocity[i] = 0.0;
        out_jointstate_fdb_var.effort[i] = 0.0;
    }
    out_jointstate_fdb.setName("out_" + this->robot_name + "_jointstate_fdb");
    out_jointstate_fdb.doc("Output port for sending joint space state");
    out_jointstate_fdb.setDataSample(out_jointstate_fdb_var);
    tc->ports()->addPort(out_jointstate_fdb);

    // Last action from the configuration side
    this->setControlMode("JointPositionCtrl");
    return true;
}
