/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_ext.cpp
 * \brief Generic external interface implementation, regardless of the communication network, of the openBuildNet simulation framework.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnsim_basic.h>
#include <obnnode_ext.h>

// Returns the maximum ID allowed for an update type.
EXPORT
int maxUpdateID() {
    return OBNsim::MAX_UPDATE_INDEX;
}