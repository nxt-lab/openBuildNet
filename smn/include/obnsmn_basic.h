/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basic definitions.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSIM_BASIC_H
#define OBNSIM_BASIC_H

#include <cstdint>

namespace OBNsmn {
    typedef int64_t simtime_t;  ///< Simulation time type, as number of nano-seconds from beginning.
    typedef int64_t updatemask_t;  ///< Update mask type: each bit corresponds to one update, so the width of the type is the maximum number of updates.
    
    /* Contains configuration information about the SMN program.
    struct SMN_CONFIGURATION_INFO {
        bool support_yarp;
        bool support_mqtt;
    };
    extern SMN_CONFIGURATION_INFO smn_configuration_info;
     */
}


#endif // OBNSIM_BASIC_H

