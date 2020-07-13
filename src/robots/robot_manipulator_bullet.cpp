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

RobotManipulatorBullet::RobotManipulatorBullet(const std::string &name, const unsigned int &model_id, std::shared_ptr<b3CApiWrapperNoGui> sim, RTT::TaskContext *tc) : RobotManipulatorIF(name, tc),
                                                                                                                                                                       ctrl_mode_params_4_motor_joints(0,0),
                                                                                                                                                                       ctrl_mode_params_4_joints(0,0)
{
    this->sim = sim;
    this->robot_id = model_id;
}

InterfaceType RobotManipulatorBullet::getInterfaceType()
{
    return InterfaceType::Bullet;
}

void RobotManipulatorBullet::sense()
{
    ////////////////////////////////////////////////////
    ///////// Get Joint States from Simulation /////////
    ////////////////////////////////////////////////////

    // b3JointStates2 _joint_states;
    // sim->getJointStates(this->robot_id, _joint_states);
    for (unsigned int j = 0; j < this->num_motor_joints; j++)
    {
        b3JointSensorState state;
        bool ret_joint_state = sim->getJointState(this->robot_id, this->vec_motor_joint_indices[j], &state);
        if (!ret_joint_state)
        {
            PRELOG(Fatal, this->tc) << "Joint State computations failed" << RTT::endlog();
            return;
        }
        this->q[j] = state.m_jointPosition;
        this->qd[j] = state.m_jointVelocity;
        this->zero_accelerations[j] = 0.0;
    }

    //////////////////////////////////////////////
    ///////// Calculate Inverse Dynamics /////////
    //////////////////////////////////////////////
    bool ret_inv_dyn_calc = sim->calculateInverseDynamics(this->robot_id, this->q, this->qd, this->zero_accelerations, this->gc);
    if (!ret_inv_dyn_calc)
    {
        PRELOG(Fatal, this->tc) << "Inverse Dynamics computations failed" << RTT::endlog();
        return;
    }

    ///////////////////////////////////////////////
    ///////// Calculate JS Inertia Matrix /////////
    ///////////////////////////////////////////////

    // TODO flag? 0 or 1?
    int ret_mass_calc = sim->calculateMassMatrix(this->robot_id, this->q, this->num_motor_joints, this->M, 0);
    if (ret_mass_calc == 0)
    {
        PRELOG(Fatal, this->tc) << "Mass Matrix computations failed" << RTT::endlog();
        return;
    }

    ////////////////////////////////////////////////////////////////////////
    ///////// Convert To Eigen And Select Based On Kinematic Chain /////////
    ////////////////////////////////////////////////////////////////////////
    unsigned int _tmp_num_joints = index_of_controlled_joint_in_controllable_joints.size();
    for (unsigned int j = 0; j < _tmp_num_joints; j++)
    {
        // joint-space position and velocity feedback
        this->out_jointstate_fdb_var.position[j] = this->q[index_of_controlled_joint_in_controllable_joints[j]];
        this->out_jointstate_fdb_var.velocity[j] = this->qd[index_of_controlled_joint_in_controllable_joints[j]];

        // joint-space gravitational torques
        this->out_gc_fdb_var(j) = this->gc[index_of_controlled_joint_in_controllable_joints[j]];
    }

    // joint-space inertia matrix
    Eigen::MatrixXd _tmp_inertia_matrix = Eigen::MatrixXd::Zero(this->num_motor_joints, this->num_motor_joints);
    for (unsigned int u = 0; u < this->num_motor_joints; u++)
    {
        for (unsigned int v = 0; v < this->num_motor_joints; v++)
        {
            _tmp_inertia_matrix(u, v) = this->M[u * this->num_motor_joints + v];
        }
    }

    for (unsigned int j = 0; j < _tmp_num_joints; j++)
    {
        for (unsigned int v = 0; v < _tmp_num_joints; v++)
        {
            this->out_inertia_fdb_var(j, v) = _tmp_inertia_matrix(index_of_controlled_joint_in_controllable_joints[j], index_of_controlled_joint_in_controllable_joints[v]);
        }
    }

    // RTT::log(RTT::Fatal) << "_tmp_inertia_matrix =\n" << _tmp_inertia_matrix << RTT::endlog();
    // RTT::log(RTT::Fatal) << "out_inertia_fdb_var =\n" << out_inertia_fdb_var << RTT::endlog();
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
                // Unlocking the breaks for each controlled joint individually (e.g., ignoring gripper finger etc...)
                PRELOG(Error, this->tc) << "Releasing the breaks" << RTT::endlog();
                this->ctrl_mode_params_4_joints.m_controlMode = CONTROL_MODE_VELOCITY;
                // We have to initializ this, because this will not be done by the constructor of the mode itself
                for (unsigned int j_index_id = 0; j_index_id < this->num_joints; j_index_id++)
                {
                    this->ctrl_mode_params_4_joints.m_forces[j_index_id] = 0.0;
                }
                sim->setJointMotorControlArray(this->robot_id, this->ctrl_mode_params_4_joints);

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
            PRELOG(Error, this->tc) << "Switching to JointPosCtrl" << RTT::endlog();
            this->ctrl_mode_params_4_joints.m_controlMode = CONTROL_MODE_POSITION_VELOCITY_PD;
            for (unsigned int j_index_id = 0; j_index_id < this->num_joints; j_index_id++)
            {
                this->ctrl_mode_params_4_joints.m_forces[j_index_id] = this->max_forces[this->index_of_controlled_joint_in_controllable_joints[j_index_id]];
                this->ctrl_mode_params_4_joints.m_targetPositions[j_index_id] = this->out_jointstate_fdb_var.position[j_index_id];
            }
            sim->setJointMotorControlArray(this->robot_id, this->ctrl_mode_params_4_joints);
        }

        this->active_control_mode = this->requested_control_mode;
    }

    ///////////////////////////////////////////////////////////////
    ///////// Send Commands according to the Control Mode /////////
    ///////////////////////////////////////////////////////////////
    if (this->active_control_mode == ControlModes::JointGravComp)
    {
        this->ctrl_mode_params_4_joints.m_controlMode = CONTROL_MODE_TORQUE;
        // this->ctrl_mode_params_4_joints.m_jointIndices = this->joint_indices;
        // TODO perhaps this can be optimized
        for (unsigned int j_index_id = 0; j_index_id < this->num_joints; j_index_id++)
        {
            this->ctrl_mode_params_4_joints.m_forces[j_index_id] = this->out_gc_fdb_var(j_index_id);
        }
        sim->setJointMotorControlArray(this->robot_id, this->ctrl_mode_params_4_joints);
    }
    else if (this->active_control_mode == ControlModes::JointPosCtrl)
    {
        this->ctrl_mode_params_4_joints.m_controlMode = CONTROL_MODE_POSITION_VELOCITY_PD;
        // this->ctrl_mode_params_4_joints.m_jointIndices = this->joint_indices;
        ///////////////////////////////////////////////////////////////////////////////
        // TODO THINK ABOUT THE SEMANTIC REGARDING SENDING THE COMMANDS TO THE ROBOT //
        ///////////////////////////////////////////////////////////////////////////////
        if (this->in_JointPositionCtrl_cmd_flow != RTT::NewData)
        {
            for (unsigned int j_index_id = 0; j_index_id < this->num_joints; j_index_id++)
            {
                this->ctrl_mode_params_4_joints.m_targetPositions[j_index_id] = this->out_jointstate_fdb_var.position[j_index_id];
                this->ctrl_mode_params_4_joints.m_forces[j_index_id] = this->max_forces[this->index_of_controlled_joint_in_controllable_joints[j_index_id]];
            }
        }
        else
        {
            this->ctrl_mode_params_4_joints.m_targetPositions = this->cmd_pos;
            for (unsigned int j_index_id = 0; j_index_id < this->num_joints; j_index_id++)
            {
                this->ctrl_mode_params_4_joints.m_forces[j_index_id] = this->max_forces[this->index_of_controlled_joint_in_controllable_joints[j_index_id]];
            }
        }
        sim->setJointMotorControlArray(this->robot_id, this->ctrl_mode_params_4_joints);
    }
    else if (this->active_control_mode == ControlModes::JointTrqCtrl)
    {
        this->ctrl_mode_params_4_joints.m_controlMode = CONTROL_MODE_TORQUE;
        ///////////////////////////////////////////////////////////////////////////////
        // TODO THINK ABOUT THE SEMANTIC REGARDING SENDING THE COMMANDS TO THE ROBOT //
        ///////////////////////////////////////////////////////////////////////////////
        if (this->in_JointTorqueCtrl_cmd_flow != RTT::NewData)
        {
            this->ctrl_mode_params_4_joints.m_forces = this->_tmp_calc_on_me; // TODO I don't think this is necessary...
            for (unsigned int j_index_id = 0; j_index_id < this->num_joints; j_index_id++)
            {
                this->ctrl_mode_params_4_joints.m_forces[j_index_id] = this->out_gc_fdb_var(j_index_id);
            }
        }
        else
        {
            this->ctrl_mode_params_4_joints.m_forces = this->cmd_trq;
        }
        sim->setJointMotorControlArray(this->robot_id, this->ctrl_mode_params_4_joints);
    }
}

// bool RobotManipulatorBullet::setActiveKinematicChain(const std::vector<std::string> &jointNames)
// {
//     if (jointNames.size() != this->vec_joint_indices.size())
//     {
//         return false;
//     }
//     // check for inconsistent names
//     for (unsigned int i = 0; i < jointNames.size(); i++)
//     {
//         if (map_joint_names_2_indices.count(jointNames[i]))
//         {
//         }
//         else
//         {
//             return false;
//         }
//     }

//     for (unsigned int i = 0; i < jointNames.size(); i++)
//     {
//         this->vec_joint_indices[i] = this->map_joint_names_2_indices[jointNames[i]];
//         this->joint_indices[i] = vec_joint_indices[i];
//     }
//     return true;
// }

bool RobotManipulatorBullet::setControlMode(const std::string &controlMode)
{
    if (controlMode.compare("JointPositionCtrl") == 0)
    {
        this->requested_control_mode = ControlModes::JointPosCtrl;
        return true;
    }
    else if (controlMode.compare("JointTorqueCtrl") == 0)
    {
        this->requested_control_mode = ControlModes::JointTrqCtrl;
        return true;
    }
    else if (controlMode.compare("JointGravComp") == 0)
    {
        this->requested_control_mode = ControlModes::JointGravComp;
        return true;
    }
    return false;
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
        int _num_bullet_joints = sim->getNumJoints(this->robot_id);
        if (_num_bullet_joints <= 0)
        {
            PRELOG(Error, this->tc) << "The associated object is not a robot, since it has " << _num_bullet_joints << " joints!" << RTT::endlog();
            this->num_joints = -1;
            return false;
        }

        ////////////////////////////////////
        /////// Get All Motor Joints ///////
        ////////////////////////////////////
        this->num_motor_joints = 0;
        vec_motor_joint_indices.clear();

        // Calculate ALL (except fixed) bullet motor joints, which need to be used to receive the data and send the data.
        // This is different from num_joints, which represents the number of joints that we actually care about!
        for (unsigned int i = 0; i < _num_bullet_joints; i++)
        {
            b3JointInfo jointInfo;
            sim->getJointInfo(this->robot_id, i, &jointInfo);
            int qIndex = jointInfo.m_jointIndex;
            if ((qIndex > -1) && (jointInfo.m_jointType != eFixedType))
            {
                this->num_motor_joints++;
                vec_motor_joint_indices.push_back(qIndex);
            }
        }

        // Change bullet collision group
        for (unsigned int i = 0; i < _num_bullet_joints; i++)
        {
            // Configure Collision
            int collisionFilterGroup_kuka = 0x10;
            int collisionFilterMask_kuka = 0x1;
            PRELOG(Error, this->tc) << "Setting collision for bullet link " << i << RTT::endlog();
            sim->setCollisionFilterGroupMask(this->robot_id, i, collisionFilterGroup_kuka, collisionFilterMask_kuka);
        }

        ////////////////////////////////////////////////
        /////// Setup Controlled Kinematic Chain ///////
        ////////////////////////////////////////////////

        // Find all (non-fixed) bullet joints that make up the kinematic chain (of KDL joints)
        map_joint_names_2_indices.clear();
        vec_joint_indices.clear();

        // Find the right joint indices that match the ones defined by the chosen kinematic chain
        if ((this->kdl_chain.getNrOfJoints() > 0) && (this->kdl_chain.getNrOfSegments() > 0))
        {
            // Select joints accordingly!
            for (unsigned int i = 0; i < this->kdl_chain.segments.size(); i++)
            {
                PRELOG(Error, this->tc) << i << " KDL Segmentname " << this->kdl_chain.segments[i].getName() << ", Jointname " << this->kdl_chain.segments[i].getJoint().getName() << RTT::endlog();
                // Ignore fixed joints
                if (this->kdl_chain.segments[i].getJoint().getType() == KDL::Joint::None)
                {
                    continue;
                }

                // Get bullet joint id for this->kdl_chain.segments[i].getJoint().getName(). A lot of times it matches, but just to be sure.
                bool _found_bullet_joint_for_kdl_joint = false;
                for (unsigned int j = 0; j < _num_bullet_joints; j++)
                {
                    b3JointInfo jointInfo;
                    sim->getJointInfo(this->robot_id, j, &jointInfo);

                    // Candidate Joints with types
                    if (jointInfo.m_jointType == eFixedType)
                    {
                        PRELOG(Error, this->tc) << "[CANDIDATE] (eFixedType) joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    }
                    else if (jointInfo.m_jointType == eRevoluteType)
                    {
                        PRELOG(Error, this->tc) << "[CANDIDATE] (eRevoluteType) joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    }
                    else if (jointInfo.m_jointType == ePrismaticType)
                    {
                        PRELOG(Error, this->tc) << "[CANDIDATE] (ePrismaticType) joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    }
                    else if (jointInfo.m_jointType == eSphericalType)
                    {
                        PRELOG(Error, this->tc) << "[CANDIDATE] (eSphericalType) joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    }
                    else if (jointInfo.m_jointType == ePlanarType)
                    {
                        PRELOG(Error, this->tc) << "[CANDIDATE] (ePlanarType) joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    }
                    else if (jointInfo.m_jointType == ePoint2PointType)
                    {
                        PRELOG(Error, this->tc) << "[CANDIDATE] (ePoint2PointType) joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    }
                    else if (jointInfo.m_jointType == eGearType)
                    {
                        PRELOG(Error, this->tc) << "[CANDIDATE] (eGearType) joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    }
                    // Ignore fixed joints
                    if ((jointInfo.m_jointIndex > -1) && (jointInfo.m_jointType != eFixedType))
                    {
                        std::string _bullet_name = jointInfo.m_jointName;
                        if (this->kdl_chain.segments[i].getJoint().getName().compare(_bullet_name) == 0)
                        {
                            PRELOG(Error, this->tc) << "[KDL+BULLET] Found joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                            map_joint_names_2_indices[jointInfo.m_jointName] = jointInfo.m_jointIndex;
                            vec_joint_indices.push_back(jointInfo.m_jointIndex);
                            _found_bullet_joint_for_kdl_joint = true;
                            break;
                        }
                    }
                }

                if (!_found_bullet_joint_for_kdl_joint)
                {
                    PRELOG(Error, this->tc) << "KDL joint name " << this->kdl_chain.segments[i].getJoint().getName() << " not found in bullet joints." << RTT::endlog();
                    return false;
                }
            }
        }
        else
        {
            // Use all non-fixed bullet joints if no KDL chain is provided
            for (unsigned int i = 0; i < _num_bullet_joints; i++)
            {
                b3JointInfo jointInfo;
                sim->getJointInfo(this->robot_id, i, &jointInfo);
                int qIndex = jointInfo.m_jointIndex;
                if ((qIndex > -1) && (jointInfo.m_jointType != eFixedType))
                {
                    PRELOG(Error, this->tc) << "[BULLET ONLY] joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << RTT::endlog();
                    map_joint_names_2_indices[jointInfo.m_jointName] = qIndex;
                    vec_joint_indices.push_back(qIndex);
                }
            }
        }

        this->num_joints = vec_joint_indices.size();
        PRELOG(Error, this->tc) << "this->num_joints " << this->num_joints << RTT::endlog();

        // Now find the index of the controlled joints in the list of all controllable (i.e. non-fixed) joints
        index_of_controlled_joint_in_controllable_joints.clear();
        index_of_controlled_joint_in_controllable_joints.resize(this->num_joints);

        // Initialize acting variables
        this->cmd_trq = new double[this->num_joints];
        this->cmd_pos = new double[this->num_joints];

        this->_tmp_calc_on_me = new double[this->num_joints];

        for (unsigned int cj = 0; cj < this->num_joints; cj++)
        {
            for (unsigned int allj = 0; allj < this->num_motor_joints; allj++)
            {
                if (vec_motor_joint_indices[cj] == vec_joint_indices[allj])
                {
                    index_of_controlled_joint_in_controllable_joints[cj] = allj;
                    break;
                }
            }

            this->cmd_trq[cj] = 0.0;
            this->cmd_pos[cj] = 0.0;

            this->_tmp_calc_on_me[cj] = 0.0;
        }

        ////////////////////////////
        /////// Sensor Setup ///////
        ////////////////////////////

        // TODO !!!
        // Add force torque sensor at EEF (i 8, m_jointIndex 8, m_jointName iiwa7_joint_ee, m_parentIndex 7)
        // PRELOG(Error, this->tc) << "Attaching F/T sensor to link idx " << (this->num_joints - 1) << " at bullet idx " << this->vec_joint_indices[this->num_joints - 1] << RTT::endlog();
        // sim->enableJointForceTorqueSensor(this->robot_id, this->vec_joint_indices[this->num_joints - 1], true); // Attach to last link

        // // Show debug line to see where the F/T was attached!
        // b3RobotSimulatorAddUserDebugLineArgs _dLine_args;
        // _dLine_args.m_parentObjectUniqueId = this->robot_id;
        // _dLine_args.m_parentLinkIndex = this->vec_joint_indices[this->num_joints - 1];
        // _dLine_args.m_colorRGB[0] = 0.1;
        // _dLine_args.m_colorRGB[1] = 1.0;
        // _dLine_args.m_colorRGB[2] = 1.0;
        // _dLine_args.m_lineWidth = 2.0;

        // double* _from_xyz = new double[3];
        // _from_xyz[0] = 0.0;
        // _from_xyz[1] = 0.0;
        // _from_xyz[2] = 0.0;
        // double* _to_xyz = new double[3];
        // _to_xyz[0] = 0.0;
        // _to_xyz[1] = 1.0;
        // _to_xyz[2] = 0.0;
        // sim->addUserDebugLine(_from_xyz, _to_xyz, _dLine_args);

        ///////////////////////////////////////
        /////// Variable Initialization ///////
        ///////////////////////////////////////

        // Initialize helper variables (Here we need to initialize with the actual motor indices)
        this->joint_indices = new int[this->num_motor_joints];
        this->zero_forces = new double[this->num_motor_joints];
        this->zero_accelerations = new double[this->num_motor_joints];
        this->max_forces = new double[this->num_motor_joints];
        this->target_positions = new double[this->num_motor_joints];

        // Initialize sensing variables
        this->q = new double[this->num_motor_joints];
        this->qd = new double[this->num_motor_joints];
        this->gc = new double[this->num_motor_joints];
        int byteSizeDofCountDouble = sizeof(double) * this->num_motor_joints;
        this->M = (double *)malloc(this->num_motor_joints * byteSizeDofCountDouble);

        for (unsigned int i = 0; i < this->num_motor_joints; i++)
        {
            this->joint_indices[i] = vec_joint_indices[i];
            this->zero_forces[i] = 0.0;
            this->max_forces[i] = 200.0;     // TODO magic number
            this->target_positions[i] = 0.0; // TODO magic number (initial config)
            this->zero_accelerations[i] = 0.0;

            this->q[i] = 0.0;
            this->qd[i] = 0.0;
            this->gc[i] = 0.0;
            for (unsigned int j = 0; j < this->num_motor_joints; j++)
            {
                this->M[i * this->num_motor_joints + j] = 0.0;
            }
            // sim->resetJointState(this->robot_id, this->joint_indices[i], 0.0);
            PRELOG(Error, this->tc) << "Resetting joint [" << i << "] = " << this->joint_indices[i] << RTT::endlog();
        }

        // Initialize the global control parameters (This could perhaps evolve into a memory leak, if the arrays are not freed when stopped!)
        this->ctrl_mode_params_4_joints = b3RobotSimulatorJointMotorArrayArgs(CONTROL_MODE_VELOCITY, this->num_joints);
        this->ctrl_mode_params_4_joints.m_jointIndices = this->joint_indices;
        this->ctrl_mode_params_4_joints.m_forces = new double[this->ctrl_mode_params_4_joints.m_numControlledDofs];
        this->ctrl_mode_params_4_joints.m_targetPositions = new double[this->ctrl_mode_params_4_joints.m_numControlledDofs];

        this->ctrl_mode_params_4_motor_joints = b3RobotSimulatorJointMotorArrayArgs(CONTROL_MODE_POSITION_VELOCITY_PD, this->num_motor_joints);
        // Set control mode initially for all the controllable joints
        this->ctrl_mode_params_4_motor_joints.m_jointIndices = this->joint_indices;
        this->ctrl_mode_params_4_motor_joints.m_forces = this->max_forces;
        // Use current positions to avoid initial jumps
        this->ctrl_mode_params_4_motor_joints.m_targetPositions = this->q;
        PRELOG(Error, this->tc) << "Switching to JointPosCtrl" << RTT::endlog();
        sim->setJointMotorControlArray(this->robot_id, this->ctrl_mode_params_4_motor_joints);

        this->active_control_mode = ControlModes::JointPosCtrl;
        this->requested_control_mode = ControlModes::JointPosCtrl;

        return RobotManipulatorIF::configure();
    }
    else
    {
        PRELOG(Error, this->tc) << "Bullet Simulator Interface seems to be not connected!" << RTT::endlog();
        return false;
    }
    return true;
}

bool RobotManipulatorBullet::setBasePosition(const double &x, const double &y, const double &z)
{
    if (sim->isConnected())
    {
        btVector3 _basePosition;
        btQuaternion _baseOrientation;
        bool ret = sim->getBasePositionAndOrientation(this->robot_id, _basePosition, _baseOrientation);
        if (!ret)
        {
            return false;
        }
        _basePosition = btVector3(x, y, z);
        return sim->resetBasePositionAndOrientation(this->robot_id, _basePosition, _baseOrientation);
    }
    else
    {
        return false;
    }
}