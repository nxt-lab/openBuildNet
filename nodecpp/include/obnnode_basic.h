/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basic definitions for node.cpp framework.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_BASIC_H
#define OBNNODE_BASIC_H

#include <cstdint>

namespace OBNnode {
    typedef int64_t simtime_t;  ///< Simulation time type, as number of nano-seconds from beginning.
    typedef int64_t outputmask_t;  ///< Output group mask type: each bit corresponds to one group, so the width of the type is the maximum number of groups.
}


#endif // OBNNODE_BASIC_H

