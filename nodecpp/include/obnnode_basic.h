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

#include <obnsim_basic.h>

namespace OBNnode {
    typedef OBNsim::simtime_t simtime_t;  ///< Simulation time type, as number of nano-seconds from beginning.
    typedef OBNsim::updatemask_t updatemask_t;  ///< Output group mask type: each bit corresponds to one group, so the width of the type is the maximum number of groups.
    const int MAX_UPDATE_INDEX = 63;    ///< The maximum index of update type allowed = number of bits in updatemast_t - 1
}


#endif // OBNNODE_BASIC_H

