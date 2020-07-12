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

// Robot Manipulator Interface includes
#include "../robots/robot_manipulator_if.hpp"

// Bullet-Specific includes
#include "../simulator_interface/bullet/b3_capi_wrapper_no_gui.hpp"

// ROS
#include <sensor_msgs/JointState.h>

namespace cosima
{
    class RobotManipulatorBullet : public RobotManipulatorIF
    {
    public:
        RobotManipulatorBullet(const std::string &name, const unsigned int &model_id, std::shared_ptr<b3CApiWrapperNoGui> sim, RTT::TaskContext *tc);

        bool configure();
        void sense();
        void act();

        bool setControlMode(const std::string &controlMode);
        // bool setActiveKinematicChain(const std::vector<std::string> &jointNames);
        void readFromOrocos();
        void writeToOrocos();

        bool setBasePosition(const double& x, const double& y, const double& z);

        InterfaceType getInterfaceType();

    private:
        std::shared_ptr<b3CApiWrapperNoGui> sim;
        int robot_id;

        int *joint_indices;

        // Helpers
        double *zero_forces;
        double *max_forces;
        double *zero_accelerations;
        double *target_positions;

        // Sense
        double *q;
        double *qd;
        double *gc;
        double *M;

        // Act
        double *cmd_trq;
        double *cmd_pos;

        unsigned int num_motor_joints;
        // std::map<std::string, int> map_motor_joint_names_2_indices;
        std::vector<unsigned int> vec_motor_joint_indices;

        std::vector<unsigned int> index_of_controlled_joint_in_controllable_joints;

        // Control Mode Parameters
        b3RobotSimulatorJointMotorArrayArgs ctrl_mode_params_4_motor_joints, ctrl_mode_params_4_joints;

        double *_tmp_calc_on_me;
    };

} // namespace cosima