/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnsim_ext_stdmsg.h
 * \brief Declarations of standard messages (errors, warnings,...) used in the external interface.
 *
 * This file is typically used by external interfaces in other languages (e.g., Matlab, Python).
 * Requires MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */


#ifndef __obnnode_ext_stdmsg__
#define __obnnode_ext_stdmsg__

namespace OBNNodeExtInt {
    namespace StdMsgs {
        extern char NODE_NOT_EXIST[];
        extern char INVALID_PORT_NAME[];
        extern char INVALID_PORT_ID[];
        extern char INTERNAL_PORT_NOT_MATCH_DECL_TYPE[];
        extern char PORT_NOT_INPUT[];
        extern char PORT_NOT_OUTPUT[];
        extern char INTERNAL_INVALID_VALUE_FROM_PORT[];
    }
}

#endif /* defined(__obnnode_ext_stdmsg__) */
