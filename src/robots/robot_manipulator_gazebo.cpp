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
 
#ifndef DISABLE_GAZEBO

#include "../include/cosima-robot-sim/robots/robot_manipulator_gazebo.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#include <unistd.h>

#define PRELOG(X, T) (RTT::log(RTT::X) << "[" << (T->getName()) << ":" << this->robot_name << "] ")

using namespace cosima;
using namespace RTT;

RobotManipulatorGazebo::RobotManipulatorGazebo(const std::string &name, const unsigned int &model_id, RTT::TaskContext *tc) : RobotManipulatorIF(name, tc)
{
    //////////////////////////////////////////////////////////
    ///////// Retrieve Model Pointer from Simulation /////////
    //////////////////////////////////////////////////////////
    model = NULL;
}

// bool RobotManipulatorGazebo::loadModel(const std::string &model_urdf)
// {
//   if (model_urdf.length() <= 0)
//   {
//     PRELOG(Error, this->tc) << "Model name is empty. Returning." << RTT::endlog();
//     return false;
//   }

//   if (!kdl_parser::treeFromFile(model_urdf, this->kdl_tree))
//   {
//     PRELOG(Error, this->tc) << "Could not extract kdl tree. Returning." << RTT::endlog();
//     return false;
//   }

//   if (this->kdl_tree.getNrOfJoints() <= 0)
//   {
//     PRELOG(Error, this->tc) << "this->kdl_tree.getNrOfJoints() = " << this->kdl_tree.getNrOfJoints() << " need to be > 0. Returning." << RTT::endlog();
//     return false;
//   }

//   if (this->kdl_tree.getNrOfSegments() <= 0)
//   {
//     PRELOG(Error, this->tc) << "this->kdl_tree.getNrOfSegments() = " << this->kdl_tree.getNrOfSegments() << " need to be > 0. Returning." << RTT::endlog();
//     return false;
//   }

//   PRELOG(Debug, this->tc) << "kdl_tree NrOfJoints = " << this->kdl_tree.getNrOfJoints() << RTT::endlog();
//   PRELOG(Debug, this->tc) << "kdl_tree getNrOfSegments = " << this->kdl_tree.getNrOfSegments() << RTT::endlog();
//   return true;
// }

InterfaceType RobotManipulatorGazebo::getInterfaceType()
{
    return InterfaceType::Gazebo;
}

bool RobotManipulatorGazebo::setBasePosition(const double& x, const double& y, const double& z)
{
    // TODO
    if (!model)
    {
        return false;
    }
    return false;
}

void RobotManipulatorGazebo::WorldUpdateBegin()
{
    if (!model)
    {
        return;
    }
    //////////////////////////////////////////////////////////
    ///////// Read Data from Simulation and Store It /////////
    //////////////////////////////////////////////////////////
    std::lock_guard<std::mutex> lockGuard(this->sync_read_mtx);

    gazebo::physics::Joint_V joints = this->model->GetJoints();
    // unsigned int _number_of_dofs = this->model->GetJointCount();
    unsigned int _number_of_dofs = joints.size();

    for (unsigned int i = 0; i < _number_of_dofs; ++i)
    {
        q[i] = joints[i]->GetAngle(0).Radian();
        qd[i] = joints[i]->GetVelocity(0);
        //			gazebo::physics::JointWrench w =
        //					this->model->GetJoint(_joint_names[i])->GetForceTorque(0u);
        //			gazebo::math::Vector3 a =
        //					this->model->GetJoint(_joint_names[i])->GetLocalAxis(0u);
        //			full_feedback->joint_feedback.torques(i) = a.Dot(w.body1Torque);
        tau[i] = joints[i]->GetForce(0u);

        this->out_jointstate_fdb_var.position[i] = q[i];
        this->out_jointstate_fdb_var.velocity[i] = qd[i];
        this->out_jointstate_fdb_var.effort[i] = tau[i];
    }

    this->out_gc_fdb_var.setZero();      // TODO
    this->out_inertia_fdb_var.setZero(); // TODO

    // for (unsigned int i = 0; i < force_torque_sensors.size(); ++i)
    //     force_torque_sensors[i].sense();

    // for (unsigned int i = 0; i < imu_sensors.size(); ++i)
    //     imu_sensors[i].sense();
}

void RobotManipulatorGazebo::WorldUpdateEnd()
{
    if (!model)
    {
        return;
    }
    //////////////////////////////////////////////
    ///////// Handle Control Mode switch /////////
    //////////////////////////////////////////////
    if (this->requested_control_mode != this->active_control_mode)
    {
        if (this->requested_control_mode == ControlModes::JointTrqCtrl)
        {
            this->gazebo_position_joint_controller->Reset();
        }
        else if (this->requested_control_mode == ControlModes::JointGravComp)
        {
            this->gazebo_position_joint_controller->Reset();
            for (unsigned int i = 0; i < this->vec_active_joints.size(); i++)
            {
                this->gazebo_position_joint_controller->SetPositionPID(this->vec_active_joints[i]->GetScopedName(), gazebo::common::PID(0, 0, 0.01)); // TODO
            }
        }
        else if (this->requested_control_mode == ControlModes::JointPosCtrl)
        {
            this->gazebo_position_joint_controller->Reset();
            for (unsigned int i = 0; i < this->vec_active_joints.size(); i++)
            {
                this->gazebo_position_joint_controller->SetPositionPID(this->vec_active_joints[i]->GetScopedName(), gazebo::common::PID(100, 0.1, 5)); // TODO
            }
        }
        this->active_control_mode = this->requested_control_mode;
    }

    ///////////////////////////////////////////////////////////////
    ///////// Send Commands according to the Control Mode /////////
    ///////////////////////////////////////////////////////////////
    if (this->active_control_mode == ControlModes::JointGravComp)
    {
        for (unsigned int i = 0; i < this->vec_active_joints.size(); i++)
        {
            this->gazebo_position_joint_controller->SetPositionTarget(this->vec_active_joints[i]->GetScopedName(), this->vec_active_joints[i]->GetAngle(0).Radian());
        }
        this->gazebo_position_joint_controller->Update();
    }
    else if (this->active_control_mode == ControlModes::JointPosCtrl)
    {
        {
            std::lock_guard<std::mutex> lockGuard(this->sync_write_mtx);

            for (unsigned int i = 0; i < this->vec_active_joints.size(); i++)
            {
                this->gazebo_position_joint_controller->SetPositionTarget(this->vec_active_joints[i]->GetScopedName(), cmd_pos(i));
            }
        }
        this->gazebo_position_joint_controller->Update();
    }
    else if (this->active_control_mode == ControlModes::JointTrqCtrl)
    {
        std::lock_guard<std::mutex> lockGuard(this->sync_write_mtx);

        for (unsigned int i = 0; i < this->vec_active_joints.size(); i++)
        {
            this->vec_active_joints[i]->SetForce(0, cmd_trq(i));
        }
    }
}

void RobotManipulatorGazebo::sense()
{
}

void RobotManipulatorGazebo::writeToOrocos()
{
    if (!model)
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lockGuard(this->sync_read_mtx);
        this->out_jointstate_fdb.write(this->out_jointstate_fdb_var);
        this->out_gc_fdb.write(this->out_gc_fdb_var);
        this->out_inertia_fdb.write(this->out_inertia_fdb_var);
    }
    RTT::log(RTT::Error) << "Gazebo out_gc_fdb_var = " << out_gc_fdb_var << RTT::endlog();
}

void RobotManipulatorGazebo::readFromOrocos()
{
    if (!model)
    {
        return;
    }
    std::lock_guard<std::mutex> lockGuard(this->sync_write_mtx);

    this->in_JointPositionCtrl_cmd_flow = this->in_JointPositionCtrl_cmd.read(this->in_JointPositionCtrl_cmd_var);
    if (this->in_JointPositionCtrl_cmd_flow != RTT::NoData)
    {
        for (unsigned int i = 0; i < this->num_joints; i++)
        {
            this->cmd_pos(i) = this->in_JointPositionCtrl_cmd_var(i);
        }
    }
    this->in_JointTorqueCtrl_cmd_flow = this->in_JointTorqueCtrl_cmd.read(this->in_JointTorqueCtrl_cmd_var);
    if (this->in_JointTorqueCtrl_cmd_flow != RTT::NoData)
    {
        for (unsigned int i = 0; i < this->num_joints; i++)
        {
            this->cmd_trq(i) = this->in_JointTorqueCtrl_cmd_var(i);
        }
    }
}

void RobotManipulatorGazebo::act()
{
}

// bool RobotManipulatorGazebo::setActiveKinematicChain(const std::vector<std::string> &jointNames)
// {
//     // TODO

//     // if (jointNames.size() != this->vec_joint_indices.size())
//     // {
//     //     return false;
//     // }
//     // // check for inconsistent names
//     // for (unsigned int i = 0; i < jointNames.size(); i++)
//     // {
//     //     if (this->map_joint_names_2_indices.count(jointNames[i]))
//     //     {
//     //     }
//     //     else
//     //     {
//     //         return false;
//     //     }
//     // }

//     // for (unsigned int i = 0; i < jointNames.size(); i++)
//     // {
//     //     this->vec_joint_indices[i] = this->map_joint_names_2_indices[jointNames[i]];
//     //     this->joint_indices[i] = this->vec_joint_indices[i];
//     // }
//     return true;
// }

bool RobotManipulatorGazebo::setControlMode(const std::string &controlMode)
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

bool RobotManipulatorGazebo::initGazeboJointController()
{
    for (unsigned int i = 0; i < this->vec_active_joints.size(); i++)
    {
        this->gazebo_position_joint_controller->AddJoint(this->vec_active_joints[i]);
        this->gazebo_position_joint_controller->SetPositionPID(this->vec_active_joints[i]->GetScopedName(), gazebo::common::PID(100, 0.1, 5)); // TODO
    }
    return true;
}

bool RobotManipulatorGazebo::configure()
{
    if (!gazebo::physics::get_world())
    {
        PRELOG(Error, this->tc) << "Gazebo not connected, since the world pointer could not be retrived!" << RTT::endlog();
        return false;
    }

    this->model = gazebo::physics::get_world()->GetModel(this->robot_name);
    if (!this->model)
    {
        PRELOG(Error, this->tc) << "Model [" << this->robot_name << "] NOT found!" << RTT::endlog();
        return false;
    }

    gazebo::physics::Joint_V _joints = this->model->GetJoints();

    // Get number of joints
    int _num_joints = _joints.size();
    if (_num_joints <= 0)
    {
        PRELOG(Error, this->tc) << "The associated object is not a robot, since it has " << _num_joints << " joints!" << RTT::endlog();
        this->num_joints = -1;
        return false;
    }

    this->gazebo_position_joint_controller.reset(new gazebo::physics::JointController(this->model));
    this->initGazeboJointController();

    // Get motor indices (filter fixed joint types)
    this->map_joint_names_2_indices.clear();
    this->vec_joint_indices.clear();
    for (unsigned int i = 0; i < _num_joints; i++)
    {
        gazebo::physics::JointPtr joint = _joints[i];
        if (joint->GetType() != gazebo::physics::Joint::FIXED_JOINT)
        {
            PRELOG(Error, this->tc) << "Motorname " << joint->GetName() << ", index " << i << RTT::endlog();
            this->map_joint_names_2_indices[joint->GetName()] = i;
            this->vec_joint_indices.push_back(i);
            this->vec_active_joints.push_back(joint);
        }
    }

    this->num_joints = this->vec_joint_indices.size();
    PRELOG(Error, this->tc) << "this->num_joints " << this->num_joints << RTT::endlog();

    // Here I should probably also check the order of the joints for the command order TODO

    // Initialize sensing variables
    this->q = Eigen::VectorXd::Zero(this->num_joints);
    this->qd = Eigen::VectorXd::Zero(this->num_joints);
    this->tau = Eigen::VectorXd::Zero(this->num_joints);

    // Initialize acting variables
    this->cmd_trq = Eigen::VectorXd::Zero(this->num_joints);
    this->cmd_pos = Eigen::VectorXd::Zero(this->num_joints);

    // Initialize KDL solvers
    // TODO

    world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotManipulatorGazebo::WorldUpdateBegin, this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
        boost::bind(&RobotManipulatorGazebo::WorldUpdateEnd, this));

    
    this->gazebo_position_joint_controller->Reset();
    for (unsigned int i = 0; i < this->vec_active_joints.size(); i++)
    {
        this->gazebo_position_joint_controller->SetPositionPID(this->vec_active_joints[i]->GetScopedName(), gazebo::common::PID(100, 0.1, 5)); // TODO
    }

    this->active_control_mode = ControlModes::JointPosCtrl;
    this->requested_control_mode = ControlModes::JointPosCtrl;

    return RobotManipulatorIF::configure();
}

#endif
