/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_ext_stdmemory.h
 * \brief Standard memory management used by the external interface library.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_EXT_STDMEMORY_H
#define OBNNODE_EXT_STDMEMORY_H

#include <obnnode_ext.h>

// These standard functions are simply empty.
// This is true for regular dynamic libraries.
// For special cases, e.g., Matlab's MEX, these functions must be redefined to call appropriate functions of the interface.

inline void OBNNodeExtInt::lockPointer(void*) { }
inline void OBNNodeExtInt::unlockPointer(void*) { }

#endif // OBNNODE_EXT_STDMEMORY_H