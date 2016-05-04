/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_ext.h
 * \brief Generic external interface, regardless of the communication network, of the openBuildNet simulation framework.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_EXT_H_
#define OBNNODE_EXT_H_

/** Types and functions the external interface can use. */
#ifdef __cplusplus
extern "C" {
#endif
    
    /** The update mask type, see obnsim_basic.h for the definition. That should match the definition here. */
    typedef uint64_t OBNUpdateMask;
    
    /** The event type */
    enum OBNEI_EventType {
        OBNEI_INIT = 0,         // Init of simulation
        OBNEI_Y = 1,            // Update Y
        OBNEI_X = 2,            // Update X
        OBNEI_TERM = 3,         // Termination of simulation
        OBNEI_RCV = 4           // A port has received a message
    };

    /** Type to pass arguments of an event */
    union OBNEI_EventArg {
        OBNUpdateMask mask;     // Update mask (see above)
        size_t index;     // Index, equivalent to uint32_t
    };
    
#ifdef __cplusplus
}
#endif


#endif /* OBNNODE_EXT_H_ */
