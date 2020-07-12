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

#include "../include/cosima-robot-sim/test/rtt_interface_test.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file
// #include <rtt/Activity.hpp>  // needed for using setActivity

#include <unistd.h>

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace RTT;

RTTInterfaceTest::RTTInterfaceTest(std::string const &name) : RTT::TaskContext(name)
{
}

bool RTTInterfaceTest::configureHook()
{
    return true;
}

bool RTTInterfaceTest::startHook()
{
    return true;
}

void RTTInterfaceTest::updateHook()
{
    it.loop();
}

void RTTInterfaceTest::stopHook()
{
}

void RTTInterfaceTest::cleanupHook()
{
}

// This macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::RTTInterfaceTest)
