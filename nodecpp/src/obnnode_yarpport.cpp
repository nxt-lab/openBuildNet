/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief YARP port classes for the C++ node interface.
 *
 * Implement the communication interface with YARP port for a C++ node.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnnode_yarpnode.h>

using namespace OBNnode;

std::pair<int, std::string> YarpPortBase::connect_from_port(const std::string& source) {
    assert(!source.empty());
    
    std::string srcport = '/' + source, tgtport = getYarpPort().getName();
    
    // Check that the source port exists
    if (!yarp::os::Network::exists(srcport)) {
        return std::make_pair(-2, "");
    }
    
    // Check if the connection already exists
    if (yarp::os::Network::isConnected(srcport, tgtport)) {
        return std::make_pair(1, "");
    }
    
    // Try to establish the connection
    return std::make_pair(yarp::os::Network::connect(srcport, tgtport)?0:-2, "");
}