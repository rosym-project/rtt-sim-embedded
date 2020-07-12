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

#include "../include/cosima-robot-sim/rtt_robot_manipulator_sim.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file
// #include <rtt/Activity.hpp>  // needed for using setActivity

#include <unistd.h>

#ifndef DISABLE_BULLET
#include "../include/cosima-robot-sim/robots/robot_manipulator_bullet.hpp"
#endif

#ifndef DISABLE_GAZEBO
#include "../include/cosima-robot-sim/robots/robot_manipulator_gazebo.hpp"
#endif

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace RTT;

RTTRobotManipulatorSim::RTTRobotManipulatorSim(std::string const &name) : RTT::TaskContext(name), new_state_sem_(0), new_cmd_sem_(0), new_cmd_(false), new_state_(false)
{
    bool _at_least_one_simulator = false;
#ifndef DISABLE_BULLET
    this->bullet_interface = std::shared_ptr<BulletInterface>(new BulletInterface());
    addOperation("connectBullet", &RTTRobotManipulatorSim::connectBullet, this);
    addOperation("disconnectBullet", &RTTRobotManipulatorSim::disconnectBullet, this);
    _at_least_one_simulator = true;
#endif

#ifndef DISABLE_GAZEBO
    this->gazebo_interface = std::shared_ptr<GazeboInterface>(new GazeboInterface());
    addOperation("connectGazebo", &RTTRobotManipulatorSim::connectGazebo, this);
    addOperation("disconnectGazebo", &RTTRobotManipulatorSim::disconnectGazebo, this);
    _at_least_one_simulator = true;
#endif

    if (_at_least_one_simulator)
    {
        addOperation("spawnRobot", &RTTRobotManipulatorSim::spawnRobot, this).doc("Returns the model_id to reference the loaded object.");
        addOperation("spawnRobotAtPose", &RTTRobotManipulatorSim::spawnRobotAtPose, this).doc("Returns the model_id to reference the loaded object.");

        addOperation("setControlMode", &RTTRobotManipulatorSim::setControlMode, this, RTT::OwnThread);

        // addOperation("setActiveKinematicChain", &RTTRobotManipulatorSim::setActiveKinematicChain, this, RTT::OwnThread); // TODO

        addOperation("connectToExternallySpawnedRobot", &RTTRobotManipulatorSim::connectToExternallySpawnedRobot, this, RTT::OwnThread);

        addOperation("spawnRobotAtPos", &RTTRobotManipulatorSim::spawnRobotAtPos, this, RTT::OwnThread);

        addOperation("defineKinematicChain", &RTTRobotManipulatorSim::defineKinematicChain, this, RTT::OwnThread);

        addOperation("setBasePosition", &RTTRobotManipulatorSim::setBasePosition, this, RTT::OwnThread);
    }

    this->req.tv_sec = 0;
    this->req.tv_nsec = 0;

    this->my_period = 0.0;
    addOperation("setUpdatePeriod", &RTTRobotManipulatorSim::setUpdatePeriod, this, RTT::OwnThread);

    this->last_time = 0.0;
}

bool RTTRobotManipulatorSim::setUpdatePeriod(double period)
{
    if (period >= 0)
    {
        this->my_period = period;
        return true;
    }
    PRELOG(Warning) << "Not changing the update period, because it was < 0." << RTT::endlog();
    return false;
}

bool RTTRobotManipulatorSim::setBasePosition(const std::string &modelName, const double &x, const double &y, const double &z)
{
    if (map_robot_manipulators.count(modelName))
    {
        return map_robot_manipulators[modelName]->setBasePosition(x, y, z);
    }
    else
    {
        PRELOG(Error) << "Robot " << modelName << " cannot be found!" << RTT::endlog();
        return false;
    }
}

bool RTTRobotManipulatorSim::defineKinematicChain(const std::string &modelName, const std::string &urdf, const std::string &chain_root_link_name, const std::string &chain_tip_link_name)
{
    if (map_robot_manipulators.count(modelName))
    {
        return map_robot_manipulators[modelName]->defineKinematicChain(urdf, chain_root_link_name, chain_tip_link_name);
    }
    else
    {
        PRELOG(Error) << "Robot " << modelName << " cannot be found!" << RTT::endlog();
        return false;
    }
}

bool RTTRobotManipulatorSim::setActiveKinematicChain(const std::vector<std::string> &jointNames)
{
    return true;
}

bool RTTRobotManipulatorSim::setControlMode(const std::string &modelName, const std::string &controlMode)
{
    if (map_robot_manipulators.count(modelName))
    {
        return map_robot_manipulators[modelName]->setControlMode(controlMode);
    }
    else
    {
        PRELOG(Error) << "Robot " << modelName << " cannot be found!" << RTT::endlog();
        return false;
    }
}

#ifndef DISABLE_BULLET
bool RTTRobotManipulatorSim::connectBullet()
{
    return this->bullet_interface->connect();
}

void RTTRobotManipulatorSim::disconnectBullet()
{
    this->bullet_interface->disconnect();
}
#endif

#ifndef DISABLE_GAZEBO
bool RTTRobotManipulatorSim::connectGazebo()
{
    return this->gazebo_interface->connect();
}

void RTTRobotManipulatorSim::disconnectGazebo()
{
    this->gazebo_interface->disconnect();
}
#endif

bool RTTRobotManipulatorSim::configureHook()
{
    // bool ret_act_update = this->setActivity(new RTT::Activity(RTT::os::HighestPriority, 0));
    // if (!ret_act_update)
    // {
    //     PRELOG(Error) << "Could not set a new activity!" << RTT::endlog();
    //     return false;
    // }
    // ret_act_update = this->getActivity()->setCpuAffinity(0);
    // if (!ret_act_update)
    // {
    //     PRELOG(Error) << "Could not update the CPU affinity of the activity!" << RTT::endlog();
    //     return false;
    // }

    for (auto const &e : map_robot_manipulators)
    {
        if (!e.second->configure())
        {
            return false;
        }
    }

    // Create the separate thread
    this->bullet_sim_thread_.reset(new BulletSimThread(this));
    return this->bullet_sim_thread_->start();
}

bool RTTRobotManipulatorSim::startHook()
{
    bool ret = false;
#ifndef DISABLE_BULLET
    bool ret_bullet = this->bullet_interface->isConnected();
    ret = ret || ret_bullet;
#endif

#ifndef DISABLE_GAZEBO
    bool ret_gazebo = this->gazebo_interface->isConnected();
    ret = ret || ret_gazebo;
#endif

    last_time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());

    return ret;
}

void RTTRobotManipulatorSim::updateHook()
{
    // This update hook is synchronized with the simulation thread, which uses
    // blocking RPC calls (over shared memory). Note that the simulation thread does not block and wait for
    // this thread to provide it with a new command. If one isn't ready, then
    // it does not send a command.
    {
        RTT::os::MutexLock lock(new_cmd_mutex_);
        for (auto const &e : map_robot_manipulators)
        {
            e.second->readFromOrocos();
            // if (e.second->getInterfaceType() == InterfaceType::Bullet)
            // {
            // }
        }

        // Signal that a new command is ready
        // RTT::os::MutexLock lock(new_cmd_mutex_);
        // new_cmd_ = true;
    }

    {
        // Wait for new state from the device thread
        // RTT::os::MutexLock lock(new_state_mutex_);
        // if (!new_state_)
        // {
            // new_state_cond_.wait(new_state_mutex_);
        // }
        // new_state_ = false;
        RTT::os::MutexLock lock(new_state_mutex_);
        for (auto const &e : map_robot_manipulators)
        {
            // if (e.second->getInterfaceType() == InterfaceType::Bullet)
            // {
            // }
            e.second->writeToOrocos();
        }
    }

    // // // Barrier with(semi) busy wait
    // // double this_time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
    // // if ((this_time - last_time) >= this->my_period)
    // // {

    // RTT::os::TimeService::ticks begin_ticks = RTT::os::TimeService::Instance()->getTicks();
    // for (auto const &e : map_robot_manipulators)
    // {
    //     e.second->sense();
    //     e.second->writeToOrocos();
    // }

    // for (auto const &e : map_robot_manipulators)
    // {
    //     e.second->readFromOrocos();
    //     e.second->act();
    // }

    // long double end_time_nsec = RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks() - begin_ticks);

    // long diff_time_nsec = ((long double)(this->my_period * 1E+9)) - end_time_nsec;

    // if (diff_time_nsec > 0)
    // {
    //     req.tv_nsec = diff_time_nsec;
    //     nanosleep(&req, (struct timespec *)NULL);
    // }

    // this->trigger();
    // // // this->getActivity()->execute(); // Does not seem to work
    // // last_time = this_time;
    // // }
    // // this->trigger();
}

void RTTRobotManipulatorSim::stopHook()
{
    this->bullet_sim_thread_->stop();
}

void RTTRobotManipulatorSim::cleanupHook()
{
#ifndef DISABLE_BULLET
    this->bullet_interface->disconnect();
#endif

#ifndef DISABLE_GAZEBO
    this->gazebo_interface->disconnect();
#endif
}

bool RTTRobotManipulatorSim::connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId, const std::string &simulator)
{
    if (map_robot_manipulators.count(modelName))
    {
        PRELOG(Warning) << "A robot with this name (" << modelName << ") is already registered!" << RTT::endlog();
        return false;
    }
#ifndef DISABLE_BULLET
    if (simulator.compare("bullet") == 0)
    {
        if (this->bullet_interface->connectToExternallySpawnedRobot(modelName, modelId))
        {
            // Create Robot
            std::shared_ptr<RobotManipulatorIF> robot = std::shared_ptr<RobotManipulatorIF>(new RobotManipulatorBullet(modelName, modelId, this->bullet_interface->sim, this));
            map_robot_manipulators[modelName] = robot;
            return true;
        }
    }
#endif

#ifndef DISABLE_GAZEBO
    if (simulator.compare("gazebo") == 0)
    {
        if (this->gazebo_interface->connectToExternallySpawnedRobot(modelName, modelId))
        {
            // Create Robot
            std::shared_ptr<RobotManipulatorIF> robot = std::shared_ptr<RobotManipulatorIF>(new RobotManipulatorGazebo(modelName, modelId, this));
            map_robot_manipulators[modelName] = robot;
            return true;
        }
    }
#endif
    return false;
}

int RTTRobotManipulatorSim::spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &t, const Eigen::VectorXd &r, const std::string &simulator)
{
    if (map_robot_manipulators.count(modelName))
    {
        PRELOG(Warning) << "A robot with this name (" << modelName << ") is already registered!" << RTT::endlog();
        return false;
    }
#ifndef DISABLE_BULLET
    if (simulator.compare("bullet") == 0)
    {
        int ret_id = this->bullet_interface->spawnRobotAtPose(modelName, modelURDF, t, r);
        if (ret_id >= 0)
        {
            // Create Robot
            map_robot_manipulators[modelName] = std::shared_ptr<RobotManipulatorIF>(new RobotManipulatorBullet(modelName, ret_id, this->bullet_interface->sim, this));
        }
        else
        {
            return false;
        }
    }
#endif

#ifndef DISABLE_GAZEBO
    if (simulator.compare("gazebo") == 0)
    {
        int ret_id = this->gazebo_interface->spawnRobotAtPose(modelName, modelURDF, t, r);
        if (ret_id >= 0)
        {
            // Create Robot
            map_robot_manipulators[modelName] = std::shared_ptr<RobotManipulatorIF>(new RobotManipulatorGazebo(modelName, ret_id, this));
        }
        else
        {
            return false;
        }
    }
#endif
    return false;
}

int RTTRobotManipulatorSim::spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &pos, const std::string &simulator)
{
    if (map_robot_manipulators.count(modelName))
    {
        PRELOG(Warning) << "A robot with this name (" << modelName << ") is already registered!" << RTT::endlog();
        return false;
    }
    Eigen::Vector4d baseOrientation = Eigen::Vector4d::Zero();
    baseOrientation(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, pos, baseOrientation, simulator);
}

int RTTRobotManipulatorSim::spawnRobot(const std::string &modelName, const std::string &modelURDF, const std::string &simulator)
{
    if (map_robot_manipulators.count(modelName))
    {
        PRELOG(Warning) << "A robot with this name (" << modelName << ") is already registered!" << RTT::endlog();
        return false;
    }
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Vector4d q = Eigen::Vector4d::Zero();
    q(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, t, q, simulator);
}

/////////////////////////////////////
///////////// THREADING /////////////
/////////////////////////////////////
RTTRobotManipulatorSim::BulletSimThread::BulletSimThread(RTTRobotManipulatorSim *owner) : RTT::os::Thread(
                                                                                              ORO_SCHED_RT,
                                                                                              RTT::os::HighestPriority,
                                                                                              owner->getPeriod(),
                                                                                              0,
                                                                                              owner->getName() + "-bullet-sim-thread"),
                                                                                          owner_(owner),
                                                                                          break_loop_sem_(0),
                                                                                          done_sem_(0)
{
    RTT::log(RTT::Error) << "Creating bullet sim thread on CPU 0 running at " << owner->getPeriod() << RTT::endlog();
}

bool RTTRobotManipulatorSim::BulletSimThread::initialize()
{
    // return owner_->simStartHook();
    return true;
}

void RTTRobotManipulatorSim::simUpdateHook()
{
    // ros::Time time = rtt_rosclock::rtt_now();
    // RTT::Seconds period = (time - last_update_time_).toSec();
    // period_ = period;

    // Sense
    {
        RTT::os::MutexLock lock(new_state_mutex_);

        // RTT::os::TimeService::ticks read_start = RTT::os::TimeService::Instance()->getTicks();

        for (auto const &e : map_robot_manipulators)
        {
            e.second->sense();
        }

        // read_duration_ = RTT::os::TimeService::Instance()->secondsSince(read_start);

        // Signal the new state
        // new_state_ = true;
        // new_state_cond_.broadcast();
    }

    // Act
    {
        RTT::os::MutexLock lock(new_cmd_mutex_);
        // if (new_cmd_)
        // {
            // RTT::os::TimeService::ticks write_start = RTT::os::TimeService::Instance()->getTicks();
            for (auto const &e : map_robot_manipulators)
            {
                e.second->act();
            }
            // write_duration_ = RTT::os::TimeService::Instance()->secondsSince(write_start);

            // new_cmd_ = false;
        // }

        // this->bullet_interface->stepSimulation();
    }

    this->bullet_interface->stepSimulation();

    // last_update_time_ = time;
}

void RTTRobotManipulatorSim::BulletSimThread::loop()
{
}

void RTTRobotManipulatorSim::BulletSimThread::step()
{
    if (owner_->isRunning())
    {
        owner_->simUpdateHook();
    }
}

bool RTTRobotManipulatorSim::BulletSimThread::breakLoop()
{
    return true;
}

void RTTRobotManipulatorSim::BulletSimThread::finalize()
{
    // owner_->simShutdownHook();
}

// This macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::RTTRobotManipulatorSim)
