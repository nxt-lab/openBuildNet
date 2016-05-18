/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file extmqtt.cpp
 * \brief Dynamic library version of Generic external interface for MQTTNode of the openBuildNet simulation framework.
 *
 * This file is typically used by external interfaces in other languages (e.g., Matlab, Python).
 * Requires MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnnode_extmqtt.h>

// Implements the functions to lock and unlock a pointer, in this case are empty
void OBNNodeExtInt::lockPointer(void*) { }
void OBNNodeExtInt::unlockPointer(void*) { }