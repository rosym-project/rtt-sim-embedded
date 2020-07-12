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

#include "../include/cosima-robot-sim/test/interface_test.hpp"

#include <unistd.h>

using namespace std::chrono;

TestInterface::TestInterface()
{
    // sim = std::shared_ptr<b3CApiWrapperNoGui>(new b3CApiWrapperNoGui());
    sim = std::shared_ptr<b3RobotSimulatorClientAPI>(new b3RobotSimulatorClientAPI());

    lastms = milliseconds();

    // bool isConnected = sim->connect(eCONNECT_SHARED_MEMORY);
    bool isConnected = sim->connect(eCONNECT_GUI_SERVER);

    if (isConnected)
    {
        std::cout << "Using shared memory" << std::endl;
    }
    else
    {
        std::cout << "NOT connected via shared memory" << std::endl;
        return;
    }

    /////////////////////////////////////////////////////
    //////////////////// spawn robot ////////////////////
    /////////////////////////////////////////////////////
    if (!sim->isConnected())
    {
        std::cout << "Not Connected!" << std::endl;
        return;
    }

    b3RobotSimulatorLoadUrdfFileArgs _args;
    // _args.m_flags = URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION;
    _args.m_flags = URDF_USE_INERTIA_FROM_FILE;

    model_id = sim->loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/catkin_py_ws/src/py-flex-assembly/gym_flexassembly/data/kuka-iiwa-7/model.urdf", _args);
    if (model_id >= 0)
    {
        std::cout << "Loaded Urdf. Received model_id = " << model_id << std::endl;

        btVector3 basePosition = btVector3(0.0, 0.0, 0.0);
        btQuaternion baseOrientation = btQuaternion(0.0, 0.0, 0.0, 1.0);

        bool ret = sim->resetBasePositionAndOrientation(model_id, basePosition, baseOrientation);
        if (!ret)
        {
            std::cout << "Could NOT reposition loaded URDF!" << std::endl;
            return;
        }
    }
    ///////////////////////////////////////////////////
    //////////////////// CONFIGURE ////////////////////
    ///////////////////////////////////////////////////
    sim->syncBodies();

    // Get number of joints
    int _num_bullet_joints = sim->getNumJoints(model_id);
    if (_num_bullet_joints <= 0)
    {
        std::cout << "The associated object is not a robot, since it has " << _num_bullet_joints << " joints!" << std::endl;
        return;
    }
    num_joints = 0;

    for (unsigned int i = 0; i < _num_bullet_joints; i++)
    {
        b3JointInfo jointInfo;
        sim->getJointInfo(model_id, i, &jointInfo);
        int qIndex = jointInfo.m_jointIndex;
        if ((qIndex > -1) && (jointInfo.m_jointType != eFixedType))
        {
            num_joints++;
            std::cout << "[BULLET ONLY] joint Motorname " << jointInfo.m_jointName << " at index " << jointInfo.m_jointIndex << std::endl;
            // map_joint_names_2_indices[jointInfo.m_jointName] = qIndex;
            vec_joint_indices.push_back(qIndex);
        }
    }
    std::cout << "num_joints = " << num_joints << std::endl;
    // // Change bullet collision group
    // for (unsigned int i = 0; i < _num_bullet_joints; i++)
    // {
    //     // Configure Collision
    //     int collisionFilterGroup_kuka = 0x10;
    //     int collisionFilterMask_kuka = 0x1;
    //     PRELOG(Error, tc) << "Setting collision for bullet link " << i << std::endl;
    //     sim->setCollisionFilterGroupMask(model_id, i, collisionFilterGroup_kuka, collisionFilterMask_kuka);
    // }

    // // Add force torque sensor at EEF (i 8, m_jointIndex 8, m_jointName iiwa7_joint_ee, m_parentIndex 7)
    // PRELOG(Error, tc) << "Attaching F/T sensor to link idx " << (num_joints - 1) << " at bullet idx " << vec_joint_indices[num_joints - 1] << std::endl;
    // sim->enableJointForceTorqueSensor(model_id, vec_joint_indices[num_joints - 1], true); // Attach to last link

    // Initialize helper variables
    joint_indices = new int[num_joints];
    zero_forces = new double[num_joints];
    zero_accelerations = new double[num_joints];
    max_forces = new double[num_joints];
    target_positions = new double[num_joints];

    _use_test_output = new double[num_joints];

    // Initialize sensing variables
    q = new double[num_joints];
    qd = new double[num_joints];
    gc = new double[num_joints];
    int byteSizeDofCountDouble = sizeof(double) * num_joints;
    M = (double *)malloc(num_joints * byteSizeDofCountDouble);

    // Initialize acting variables
    cmd_trq = new double[num_joints];
    cmd_pos = new double[num_joints];

    for (unsigned int i = 0; i < num_joints; i++)
    {
        joint_indices[i] = vec_joint_indices[i];
        zero_forces[i] = 0.0;
        max_forces[i] = 200.0;     // TODO magic number
        target_positions[i] = 0.0; // TODO magic number (initial config)
        zero_accelerations[i] = 0.0;

        q[i] = 0.0;
        qd[i] = 0.0;
        gc[i] = 0.0;
        for (unsigned int j = 0; j < num_joints; j++)
        {
            M[i * num_joints + j] = 0.0;
        }
        cmd_trq[i] = 0.0;
        cmd_pos[i] = 0.0;

        _use_test_output[i] = 0.0;

        // sim->resetJointState(model_id, joint_indices[i], 0.0);
        std::cout << "Resetting joint [" << i << "] = " << joint_indices[i] << std::endl;
    }

    // Switching to position control
    b3RobotSimulatorJointMotorArrayArgs _mode_params(CONTROL_MODE_POSITION_VELOCITY_PD, num_joints);
    _mode_params.m_jointIndices = joint_indices;
    _mode_params.m_forces = max_forces;
    // Use current positions to avoid initial jumps
    _mode_params.m_targetPositions = q;
    std::cout << "Switching to JointPosCtrl" << std::endl;
    sim->setJointMotorControlArray(model_id, _mode_params);

    // Change to torque control
    // Unlocking the breaks
    b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_VELOCITY, num_joints);
    mode_params.m_jointIndices = joint_indices;
    mode_params.m_forces = zero_forces;
    std::cout << "Releasing the breaks" << std::endl;
    sim->setJointMotorControlArray(model_id, mode_params);

    sim->setTimeStep(0.001);
}

void TestInterface::loop()
{
    milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    double timestamp = 0.001;//((double)(ms.count() - lastms.count())) * 1E-6;
    if ((ms.count() - lastms.count()) < 1)
    {
        return;
    }
    lastms = ms;
    ///////////////////////////////////////////////
    //////////////////// SENSE ////////////////////
    ///////////////////////////////////////////////

    Eigen::VectorXd fdb_position = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd fdb_velocity = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd fdb_gc = Eigen::VectorXd::Zero(num_joints);
    Eigen::MatrixXd fdb_inertia = Eigen::MatrixXd::Zero(num_joints, num_joints);

    ////////////////////////////////////////////////////
    ///////// Get Joint States from Simulation /////////
    ////////////////////////////////////////////////////

    // b3JointStates2 _joint_states;
    // sim->getJointStates(model_id, _joint_states);
    for (unsigned int j = 0; j < num_joints; j++)
    {
        b3JointSensorState state;
        bool ret_joint_state = sim->getJointState(model_id, joint_indices[j], &state);
        if (!ret_joint_state)
        {
            std::cout << "Joint State computations failed" << std::endl;
            return;
        }
        q[j] = state.m_jointPosition;
        qd[j] = state.m_jointVelocity;
        zero_accelerations[j] = 0.0;

        // Convert to eigen data types
        fdb_position(j) = q[j];
        fdb_velocity(j) = qd[j];
    }

    //////////////////////////////////////////////
    ///////// Calculate Inverse Dynamics /////////
    //////////////////////////////////////////////
    bool ret_inv_dyn_calc = sim->calculateInverseDynamics(model_id, q, qd, zero_accelerations, gc);
    if (!ret_inv_dyn_calc)
    {
        std::cout << "Inverse Dynamics computations failed" << std::endl;
        return;
    }
    for (unsigned int j = 0; j < num_joints; j++)
    {
        fdb_gc(j) = gc[j];
    }

    ///////////////////////////////////////////////
    ///////// Calculate JS Inertia Matrix /////////
    ///////////////////////////////////////////////

    // TODO flag? 0 or 1?
    int ret_mass_calc = sim->calculateMassMatrix(model_id, q, num_joints, M, 0);
    if (ret_mass_calc == 0)
    {
        std::cout << "Mass Matrix computations failed" << std::endl;
        return;
    }
    for (unsigned int u = 0; u < num_joints; u++)
    {
        for (unsigned int v = 0; v < num_joints; v++)
        {
            fdb_inertia(u, v) = M[u * num_joints + v];
        }
    }

    ////////////////////////////////////////////////////
    //////////////////// Controller ////////////////////
    ////////////////////////////////////////////////////

    /////////////////////////////////////////////
    //////////////////// ACT ////////////////////
    /////////////////////////////////////////////

    b3RobotSimulatorJointMotorArrayArgs _mode_params_trq(CONTROL_MODE_TORQUE, num_joints);
    _mode_params_trq.m_jointIndices = joint_indices;
    _mode_params_trq.m_forces = _use_test_output;

    Eigen::VectorXd qError = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd qdError = Eigen::VectorXd::Zero(num_joints);
    for (unsigned int i = 0; i < num_joints; i++)
    {
        qError(i) = 0 - fdb_position(i);
        qdError(i) = 0 - fdb_velocity(i);
    }

    // Eigen::VectorXd outtt = fdb_inertia.inverse() * fdb_gc;

    Eigen::MatrixXd Kd = (30 * Eigen::VectorXd::Ones(num_joints)).asDiagonal();

    Eigen::VectorXd pd = ((200 * Eigen::VectorXd::Ones(num_joints)).asDiagonal()) * qError.matrix() + Kd * qdError.matrix();

    Eigen::MatrixXd M = fdb_inertia + Kd * timestamp;

    Eigen::VectorXd qddot = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-fdb_gc + pd);


    Eigen::VectorXd out_torques_var = pd - (Kd * qddot) * timestamp + fdb_gc;

    // for (unsigned int i = 0; i < num_joints; i++)
    // {
    //     out_torques_var(i) = -10 * in_robotstatus_var.position[i] -1.5 * in_robotstatus_var.velocity[i] + in_coriolisAndGravity_var(i);
    // }
    




    for (unsigned int v = 0; v < num_joints; v++)
    {
        // _mode_params_trq.m_forces[v] = -10 * fdb_position(v) -1.5 * fdb_velocity(v);
        _mode_params_trq.m_forces[v] = out_torques_var(v);
    }
    sim->setJointMotorControlArray(model_id, _mode_params_trq);

    // Step Simulation
    sim->stepSimulation();
}

int main()
{
    std::cout << "Hallo, Welt!" << std::endl;
    TestInterface tinterface = TestInterface();
    while (true)
    {
        tinterface.loop();
    }

    return 0;
}