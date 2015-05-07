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


YarpPortBase::~YarpPortBase() {
    //std::cout << "~YarpPortBase" << std::endl;
    if (isValid()) {
        // Notify the node to remove me
        _theNode->removePort(this);
        _theNode = nullptr;
        //std::cout << "Calling removePort." << std::endl;
    }
}

YarpOutputPortBase::~YarpOutputPortBase() {
    //std::cout << "~YarpOutputPortBase" << std::endl;
    if (isValid()) {
        // Notify the node to remove me
        _theNode->removePort(this);
        _theNode = nullptr;
        //std::cout << "Calling removePort for output." << std::endl;
    }
}