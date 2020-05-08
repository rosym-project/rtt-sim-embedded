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

// https://github.com/bulletphysics/bullet3/issues/1459

#include "rtt_bullet_embedded.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#include <unistd.h>

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace RTT;

RTTBulletEmbedded::RTTBulletEmbedded(std::string const &name) : RTT::TaskContext(name)
{
    addOperation("connect", &RTTBulletEmbedded::connect, this);
    addOperation("disconnect", &RTTBulletEmbedded::disconnect, this);

    addOperation("spawnModel", &RTTBulletEmbedded::spawnModel, this).doc("Returns the model_id to reference the loaded object.");
    addOperation("spawnModelAtPose", &RTTBulletEmbedded::spawnModelAtPose, this).doc("Returns the model_id to reference the loaded object.");

    addOperation("resetSimulation", &RTTBulletEmbedded::resetSimulation, this);

    this->addOperation("setGravityVector",
					   &RTTBulletEmbedded::setGravityVector, this, RTT::OwnThread)
		.doc("Set the gravity vector for the physics engine.")
		.arg("x", "World X axis")
		.arg("y", "World Y axis")
		.arg("z", "World Z axis");

    sim = std::shared_ptr<b3CApiWrapperNoGui>(new b3CApiWrapperNoGui());
}

void RTTBulletEmbedded::disconnect()
{
    sim->disconnect();
}

bool RTTBulletEmbedded::connect()
{
    if (!sim->isConnected())
    {
        bool isConnected = sim->connect(eCONNECT_SHARED_MEMORY);

        if (isConnected)
        {
            PRELOG(Error) << "Using shared memory" << RTT::endlog();
            return true;
        }
        else
        {
            PRELOG(Error) << "NOT connected via shared memory" << RTT::endlog();
            return false;
        }
    }
    else
    {
        PRELOG(Error) << "Seems to be already connected" << RTT::endlog();
    }
    
    return false;
}

bool RTTBulletEmbedded::configureHook()
{
    map_modelName_to_id.clear();
    vec_model_ids.clear();
    this->connect();
    if (sim->isConnected())
    {
        sim->setGravity(btVector3(0, 0, -9.81));
    }

	//remove all existing objects (if any)
	// sim->resetSimulation();
	// sim->setNumSolverIterations(100);
    return true;
}

void RTTBulletEmbedded::resetSimulation()
{
    if (sim->isConnected())
    {
        sim->resetSimulation();
    }
}

bool RTTBulletEmbedded::startHook()
{
    // TODO
    if (!sim->isConnected())
    {
        return false;
    }

    if (vec_model_ids.size() > 0) // TEST
    {
        int numJoints = sim->getNumJoints(vec_model_ids[0]);
        b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_PD, numJoints);
        int tmp_jointIndices[numJoints];
        double tmp_targetPositions[numJoints];
        double tmp_forces[numJoints];

        b3JointInfo jointInfo;

        for (unsigned int i = 0; i < numJoints; i++)
        {
            sim->getJointInfo(vec_model_ids[0], i, &jointInfo);
            int qIndex = jointInfo.m_jointIndex;
            if (qIndex > -1)
            {
                PRELOG(Error) << "Motorname " << jointInfo.m_jointName << ", index " << jointInfo.m_jointIndex << RTT::endlog();
                // self.motorNames.append(str(jointInfo[1]))
                // self.motorIndices.append(i)
                // self.zeroForces.append(0.0)
                tmp_jointIndices[i] = i;
                tmp_targetPositions[i] = 0.0;
                tmp_forces[i] = 200.0;
            }

            sim->resetJointState(vec_model_ids[0], i, 0.0);
        }
        
    }

    return true;
}

void RTTBulletEmbedded::updateHook()
{    
    // Load Feedback
    if (vec_model_ids.size() > 0) // TEST
    {
        
        // b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_VELOCITY, 7);
        // int tmp_jointIndices[7];
        // double tmp_forces[7];
        // for (unsigned int i = 0; i < 7; i++)
        // {
        //     tmp_forces[i] = 0.0;
        // }
        // mode_params.m_jointIndices = tmp_jointIndices;
        // mode_params.m_forces = tmp_forces;
        
        // sim->setJointMotorControlArray(vec_model_ids[0], mode_params);
    }

    // Step forward!
    sim->stepSimulation();
}

void RTTBulletEmbedded::stopHook()
{
}

void RTTBulletEmbedded::cleanupHook()
{
    sim->disconnect();
}

int RTTBulletEmbedded::getIdFromModelName(const std::string &modelName)
{
    if (map_modelName_to_id.count(modelName))
    {
        return map_modelName_to_id[modelName];
    }
    PRELOG(Error) << "Model name NOT found!" << RTT::endlog();
    return -1;
}

int RTTBulletEmbedded::spawnModelAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXf &t, const Eigen::VectorXf &r)
{
    if (sim->isConnected()) {
        int model_id = sim->loadURDF(modelURDF);
        if (model_id >= 0)
        {
            map_modelName_to_id[modelName] = model_id;
            vec_model_ids.push_back(model_id); // Not sure if we really need this!

            PRELOG(Error) << "Loaded Urdf. Received model_id = " << model_id << RTT::endlog();

            btVector3 basePosition = btVector3(t(0), t(1), t(2));
            btQuaternion baseOrientation = btQuaternion(r(1), r(2), r(3), r(0));

            sim->resetBasePositionAndOrientation(model_id, basePosition, baseOrientation);
            return model_id;
        }
    }
    PRELOG(Error) << "Could NOT load Urdf: " << modelURDF << RTT::endlog();
    return -1;
}

int RTTBulletEmbedded::spawnModel(const std::string &modelName, const std::string &modelURDF)
{
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    Eigen::Vector4f q = Eigen::Vector4f::Zero();
    q(0) = 1;
    return spawnModelAtPose(modelName, modelURDF, t, q);
}

bool RTTBulletEmbedded::setGravityVector(const double x, const double y, const double z)
{
	if (sim->isConnected())
	{
        sim->setGravity(btVector3(x, y, z));
        PRELOG(Info) << "Changed gravity vector to (" << x << ", " << y << ", " << z << ")" << RTT::endlog();
	}
	return false;
}

//this macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::RTTBulletEmbedded)
