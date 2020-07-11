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
#include <rtt/os/Timer.hpp>
#include <rtt/os/Thread.hpp>
#include <rtt/os/Semaphore.hpp>
#include <string>

#include <Eigen/Dense>
#include <iostream>

// Generic Robot Manipulator Interface includes
#include "robots/robot_manipulator_if.hpp"

// Generic Simulator Interface includes
#include "simulator_interface/sim_if.hpp"

#ifndef DISABLE_BULLET
// Bullet-Specific Simulator includes
#include "simulator_interface/bullet/bullet_if.hpp"
#endif

#ifndef DISABLE_GAZEBO
// Gazebo-Specific Simulator includes
#include "simulator_interface/gazebo/gazebo_if.hpp"
#endif

// For nanosleep
#include <time.h>

namespace cosima
{

  class RTTRobotManipulatorSim : public RTT::TaskContext
  {
  public:
    RTTRobotManipulatorSim(std::string const &name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    int spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &t, const Eigen::VectorXd &r, const std::string &simulator);
    int spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &pos, const std::string &simulator);
    int spawnRobot(const std::string &modelName, const std::string &modelURDF, const std::string &simulator);
    bool connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId, const std::string &simulator);

    bool setControlMode(const std::string &modelName, const std::string &controlMode);
    bool setActiveKinematicChain(const std::vector<std::string> &jointNames);

    bool defineKinematicChain(const std::string &modelName, const std::string &urdf, const std::string &chain_root_link_name, const std::string &chain_tip_link_name);

    bool setBasePosition(const std::string &modelName, const double& x, const double& y, const double& z);

    bool setUpdatePeriod(double period);

    // Update hook for thread
    void simUpdateHook();

#ifndef DISABLE_BULLET
    bool connectBullet();
    void disconnectBullet();
#endif

#ifndef DISABLE_GAZEBO
    bool connectGazebo();
    void disconnectGazebo();
#endif

  private:
#ifndef DISABLE_BULLET
    std::shared_ptr<BulletInterface> bullet_interface;
#endif

#ifndef DISABLE_GAZEBO
    std::shared_ptr<GazeboInterface> gazebo_interface;
#endif

    // std::map<unsigned int, std::shared_ptr<RobotManipulatorIF>> map_robot_manipulators;
    std::map<std::string, std::shared_ptr<RobotManipulatorIF>> map_robot_manipulators;

    struct timespec req;
    double my_period;

    double last_time;

    //! An RTT thread class for low level IO
    class BulletSimThread : public RTT::os::Thread
    {
    public:
      BulletSimThread(RTTRobotManipulatorSim* owner);
      RTTRobotManipulatorSim* owner_;
    protected:
      virtual bool initialize();
      virtual void loop();
      virtual void step();
      virtual bool breakLoop();
      virtual void finalize();

      RTT::os::Semaphore break_loop_sem_;
      RTT::os::Semaphore done_sem_;
    };

    boost::shared_ptr<BulletSimThread> bullet_sim_thread_;
    // Threading synchronization
    RTT::os::Semaphore new_state_sem_;
    RTT::os::Semaphore new_cmd_sem_;
    
    bool new_state_;
    RTT::os::Mutex new_state_mutex_;
    RTT::os::Condition new_state_cond_;

    bool new_cmd_;
    RTT::os::Mutex new_cmd_mutex_;
  };

} // namespace cosima