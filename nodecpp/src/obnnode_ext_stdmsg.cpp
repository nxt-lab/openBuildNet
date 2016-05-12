/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnsim_ext_stdmsg.cpp
 * \brief Definitions of standard messages (errors, warnings,...) used in the external interface.
 *
 * This file is typically used by external interfaces in other languages (e.g., Matlab, Python).
 * Requires MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnnode_ext_stdmsg.h>

char OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST[] = "Node does not exist.";
char OBNNodeExtInt::StdMsgs::INVALID_PORT_NAME[] = "Invalid port name.";
char OBNNodeExtInt::StdMsgs::INVALID_PORT_ID[] = "Invalid port ID.";
char OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE[] = "Internal error: port type does not match its declared type.";
char OBNNodeExtInt::StdMsgs::PORT_NOT_INPUT[] = "Port is not an input.";
char OBNNodeExtInt::StdMsgs::INTERNAL_INVALID_VALUE_FROM_PORT[] = "Internal error: invalid value returned from port.";