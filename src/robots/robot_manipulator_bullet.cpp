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

#include "../include/cosima-robot-sim/robots/robot_manipulator_bullet.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#include <unistd.h>

#define PRELOG(X, T) (RTT::log(RTT::X) << "[" << (T->getName()) << ":" << this->robot_name << "] ")

using namespace cosima;
using namespace RTT;

RobotManipulatorBullet::RobotManipulatorBullet(const std::string &name, const unsigned int &model_id, std::shared_ptr<b3CApiWrapperNoGui> sim, RTT::TaskContext *tc) : RobotManipulatorIF(name, tc)
{
    this->sim = sim;
    this->robot_id = model_id;
}

void RobotManipulatorBullet::sense()
{
    ////////////////////////////////////////////////////
    ///////// Get Joint States from Simulation /////////
    ////////////////////////////////////////////////////

    // b3JointStates2 _joint_states;
    // sim->getJointStates(this->robot_id, _joint_states);
    for (unsigned int j = 0; j < this->num_joints; j++)
    {
        b3JointSensorState state;
        sim->getJointState(this->robot_id, joint_indices[j], &state);
        this->q[j] = state.m_jointPosition;
        this->qd[j] = state.m_jointVelocity;
        this->zero_accelerations[j] = 0.0;

        // Convert to eigen data types
        this->out_jointstate_fdb_var.position[j] = this->q[j];
        this->out_jointstate_fdb_var.velocity[j] = this->qd[j];
    }

    //////////////////////////////////////////////
    ///////// Calculate Inverse Dynamics /////////
    //////////////////////////////////////////////
    sim->calculateInverseDynamics(this->robot_id, this->q, this->qd, this->zero_accelerations, this->gc);
    for (unsigned int j = 0; j < this->num_joints; j++)
    {
        this->out_gc_fdb_var(j) = this->gc[j];
    }

    ///////////////////////////////////////////////
    ///////// Calculate JS Inertia Matrix /////////
    ///////////////////////////////////////////////

    // TODO flag? 0 or 1?
    sim->calculateMassMatrix(this->robot_id, this->q, this->num_joints, this->M, 0);
    for (unsigned int u = 0; u < this->num_joints; u++)
    {
        for (unsigned int v = 0; v < this->num_joints; v++)
        {
            this->out_inertia_fdb_var(u, v) = this->M[u * this->num_joints + v];
        }
    }
}

void RobotManipulatorBullet::writeToOrocos()
{
    this->out_jointstate_fdb.write(this->out_jointstate_fdb_var);
    this->out_gc_fdb.write(this->out_gc_fdb_var);
    // RTT::log(RTT::Error) << "Bullet out_gc_fdb_var = " << out_gc_fdb_var << RTT::endlog();
    this->out_inertia_fdb.write(this->out_inertia_fdb_var);
}

void RobotManipulatorBullet::readFromOrocos()
{
    this->in_JointPositionCtrl_cmd_flow = this->in_JointPositionCtrl_cmd.read(this->in_JointPositionCtrl_cmd_var);
    if (this->in_JointPositionCtrl_cmd_flow != RTT::NoData)
    {
        for (unsigned int i = 0; i < this->num_joints; i++)
        {
            this->cmd_pos[i] = this->in_JointPositionCtrl_cmd_var(i);
        }
    }
    this->in_JointTorqueCtrl_cmd_flow = this->in_JointTorqueCtrl_cmd.read(this->in_JointTorqueCtrl_cmd_var);
    if (this->in_JointTorqueCtrl_cmd_flow != RTT::NoData)
    {
        for (unsigned int i = 0; i < this->num_joints; i++)
        {
            this->cmd_trq[i] = this->in_JointTorqueCtrl_cmd_var(i);
        }
    }
}

void RobotManipulatorBullet::act()
{
    //////////////////////////////////////////////
    ///////// Handle Control Mode switch /////////
    //////////////////////////////////////////////
    if (this->requested_control_mode != this->active_control_mode)
    {
        if ((this->requested_control_mode == ControlModes::JointTrqCtrl) || (this->requested_control_mode == ControlModes::JointGravComp))
        {
            // Only if we had a torque-unrelated control mode active
            if ((this->active_control_mode != ControlModes::JointTrqCtrl) && (this->active_control_mode != ControlModes::JointGravComp))
            {
                // Unlocking the breaks
                b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_VELOCITY, this->num_joints);
                mode_params.m_jointIndices = this->joint_indices;
                mode_params.m_forces = this->zero_forces;
                PRELOG(Error, this->tc) << "Releasing the breaks" << RTT::endlog();
                sim->setJointMotorControlArray(this->robot_id, mode_params);

                if (this->requested_control_mode == ControlModes::JointTrqCtrl)
                {
                    PRELOG(Error, this->tc) << "Switching to JointTrqCtrl" << RTT::endlog();
                }
                else if (this->requested_control_mode == ControlModes::JointGravComp)
                {
                    PRELOG(Error, this->tc) << "Switching to JointGravComp" << RTT::endlog();
                }
            }
            // Else just set the new control mode for the next phase
        }
        else if (this->requested_control_mode == ControlModes::JointPosCtrl)
        {
            // handle control modes (initial control mode = PD Position)
            // b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_PD, this->num_joints);
            b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_POSITION_VELOCITY_PD, this->num_joints);
            mode_params.m_jointIndices = this->joint_indices;
            mode_params.m_forces = this->max_forces;
            // Use current positions to avoid initial jumps
            mode_params.m_targetPositions = this->q;
            PRELOG(Error, this->tc) << "Switching to JointPosCtrl" << RTT::endlog();
            sim->setJointMotorControlArray(this->robot_id, mode_params);
        }

        this->active_control_mode = this->requested_control_mode;
    }

    ///////////////////////////////////////////////////////////////
    ///////// Send Commands according to the Control Mode /////////
    ///////////////////////////////////////////////////////////////
    if (this->active_control_mode == ControlModes::JointGravComp)
    {
        b3RobotSimulatorJointMotorArrayArgs _mode_params_trq(CONTROL_MODE_TORQUE, this->num_joints);
        _mode_params_trq.m_jointIndices = this->joint_indices;
        _mode_params_trq.m_forces = gc;
        sim->setJointMotorControlArray(this->robot_id, _mode_params_trq);
    }
    else if (this->active_control_mode == ControlModes::JointPosCtrl)
    {
        b3RobotSimulatorJointMotorArrayArgs _mode_params(CONTROL_MODE_POSITION_VELOCITY_PD, this->num_joints);
        _mode_params.m_jointIndices = this->joint_indices;
        _mode_params.m_forces = this->max_forces;
        ///////////////////////////////////////////////////////////////////////////////
        // TODO THINK ABOUT THE SEMANTIC REGARDING SENDING THE COMMANDS TO THE ROBOT //
        ///////////////////////////////////////////////////////////////////////////////
        if (this->in_JointPositionCtrl_cmd_flow != RTT::NewData)
        {
            _mode_params.m_targetPositions = this->q;
        }
        else
        {
            _mode_params.m_targetPositions = this->cmd_pos;
        }
        sim->setJointMotorControlArray(this->robot_id, _mode_params);
    }
    else if (this->active_control_mode == ControlModes::JointTrqCtrl)
    {
        b3RobotSimulatorJointMotorArrayArgs _mode_params_trq(CONTROL_MODE_TORQUE, this->num_joints);
        _mode_params_trq.m_jointIndices = this->joint_indices;
        ///////////////////////////////////////////////////////////////////////////////
        // TODO THINK ABOUT THE SEMANTIC REGARDING SENDING THE COMMANDS TO THE ROBOT //
        ///////////////////////////////////////////////////////////////////////////////
        if (this->in_JointTorqueCtrl_cmd_flow != RTT::NewData)
        {
            _mode_params_trq.m_forces = this->gc;
        }
        else
        {
            _mode_params_trq.m_forces = this->cmd_trq;
        }
        sim->setJointMotorControlArray(this->robot_id, _mode_params_trq);
    }
}

bool RobotManipulatorBullet::setActiveKinematicChain(const std::vector<std::string> &jointNames)
{
    if (jointNames.size() != this->vec_joint_indices.size())
    {
        return false;
    }
    // check for inconsistent names
    for (unsigned int i = 0; i < jointNames.size(); i++)
    {
        if (map_joint_names_2_indices.count(jointNames[i]))
        {
        }
        else
        {
            return false;
        }
    }

    for (unsigned int i = 0; i < jointNames.size(); i++)
    {
        this->vec_joint_indices[i] = this->map_joint_names_2_indices[jointNames[i]];
        this->joint_indices[i] = vec_joint_indices[i];
    }
    return true;
}

bool RobotManipulatorBullet::setControlMode(const std::string &controlMode)
{
    if (controlMode.compare("JointPositionCtrl") == 0)
    {
        this->requested_control_mode = ControlModes::JointPosCtrl;
    }
    else if (controlMode.compare("JointTorqueCtrl") == 0)
    {
        this->requested_control_mode = ControlModes::JointTrqCtrl;
    }
    else if (controlMode.compare("JointGravComp") == 0)
    {
        this->requested_control_mode = ControlModes::JointGravComp;
    }
}

bool RobotManipulatorBullet::configure()
{
    if (sim->isConnected())
    {
        // Check if a model is loaded, which needs to be the first step!
        if (this->robot_id < 0)
        {
            PRELOG(Error, this->tc) << "No robot associated, please spawn or connect a robot first!" << RTT::endlog();
            return false;
        }

        sim->syncBodies();

        // Get number of joints
        int _num_joints = sim->getNumJoints(this->robot_id);
        if (_num_joints <= 0)
        {
            PRELOG(Error, this->tc) << "The associated object is not a robot, since it has " << _num_joints << " joints!" << RTT::endlog();
            this->num_joints = -1;
            return false;
        }

        // Get motor indices (filter fixed joint types)
        map_joint_names_2_indices.clear();
        vec_joint_indices.clear();
        for (unsigned int i = 0; i < _num_joints; i++)
        {
            b3JointInfo jointInfo;
            sim->getJointInfo(this->robot_id, i, &jointInfo);
            int qIndex = jointInfo.m_jointIndex;
            if ((qIndex > -1) && (jointInfo.m_jointType != eFixedType))
            {
                PRELOG(Error, this->tc) << "Motorname " << jointInfo.m_jointName << ", index " << jointInfo.m_jointIndex << RTT::endlog();
                map_joint_names_2_indices[jointInfo.m_jointName] = qIndex;
                vec_joint_indices.push_back(qIndex);
            }
        }

        this->num_joints = vec_joint_indices.size();
        PRELOG(Error, this->tc) << "this->num_joints " << this->num_joints << RTT::endlog();

        // Here I should probably also check the order of the joints for the command order TODO

        // Initialize helper variables
        this->joint_indices = new int[this->num_joints];
        this->zero_forces = new double[this->num_joints];
        this->zero_accelerations = new double[this->num_joints];
        this->max_forces = new double[this->num_joints];
        this->target_positions = new double[this->num_joints];

        // Initialize sensing variables
        this->q = new double[this->num_joints];
        this->qd = new double[this->num_joints];
        this->gc = new double[this->num_joints];
        int byteSizeDofCountDouble = sizeof(double) * this->num_joints;
        this->M = (double *)malloc(this->num_joints * byteSizeDofCountDouble);

        // Initialize acting variables
        this->cmd_trq = new double[this->num_joints];
        this->cmd_pos = new double[this->num_joints];

        for (unsigned int i = 0; i < this->num_joints; i++)
        {
            this->joint_indices[i] = vec_joint_indices[i];
            this->zero_forces[i] = 0.0;
            this->max_forces[i] = 200.0;     // TODO magic number
            this->target_positions[i] = 0.0; // TODO magic number (initial config)
            this->zero_accelerations[i] = 0.0;

            this->q[i] = 0.0;
            this->qd[i] = 0.0;
            this->gc[i] = 0.0;
            for (unsigned int j = 0; j < this->num_joints; j++)
            {
                this->M[i * this->num_joints + j] = 0.0;
            }
            this->cmd_trq[i] = 0.0;
            this->cmd_pos[i] = 0.0;

            // sim->resetJointState(this->robot_id, this->joint_indices[i], 0.0);
            PRELOG(Error, this->tc) << "joint_indices[" << i << "] = " << joint_indices[i] << RTT::endlog();
        }

        return RobotManipulatorIF::configure();
    }
    else
    {
        PRELOG(Error, this->tc) << "Bullet Simulator Interface seems to be not connected!" << RTT::endlog();
        return false;
    }
    return true;
}
