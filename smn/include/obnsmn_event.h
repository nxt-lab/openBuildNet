/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Definition of event classes.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * Header file for the OBN-Sim SMN events, that are sent to the main queue of the global clock in the SMN.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSIM_EVENT_H_
#define OBNSIM_EVENT_H_

#include <iostream>
#include <obnsmn_basic.h>
#include <obnsim_msg.pb.h>

namespace OBNsmn {
    /*
    enum EVENTTYPE {
        SYSTEM_EVT,   // a system event, e.g. start/stop simulation, system error, etc.
        NODE_EVT      // a node event, i.e. a message from a node
    };

    
    // SMNEvent is the base class for all events in the SMN (system, communication, etc.)
    // An event type must subclass this abstract class to implement the actual event type.
    struct SMNEvent
    {
        SMNEvent() { std::cout << "Construct" << std::endl; }
        virtual ~SMNEvent() { std::cout << "Delete" << std::endl; }
        
        // This method returns the class/category of the event
        virtual EVENTTYPE event_type() = 0;
    };
    
    
    // The system event class
    struct SMNSysEvent: public SMNEvent {
        virtual EVENTTYPE event_type() { return SYSTEM_EVT; }
    };
    */
    
    /** \brief The node event class
     
     This class implements the events that are sent from other nodes to the SMN (e.g. for synchronous simulation).
     */
    struct SMNNodeEvent //: public SMNEvent
    {
        /** ID of the node that sent event. */
        int nodeID;
        
        /** The type of the event (same as the N2SMN message type. */
        OBNSimMsg::N2SMN_MSGTYPE type;
        
        /** Category of a node (event/message), to speed up the event processing in the GC. */
        enum EventCategory {
            EVT_ACK,  ///< Simple acknowledgement message from a node, with no data
            EVT_SIM   ///< Other co-simulation related message, usually contain extra data
        };
        EventCategory category;
        
        ///@{
        /** These bits indicate if the event / message has certain optional data (in the data field). They are initialized to 0.
         */
        unsigned char has_t : 1;
        unsigned char has_i : 1;
        unsigned char has_ix: 1;
        unsigned char has_b : 1;
        ///@}
        
        // Currently we store the fields in this object, but later on, we may optimize for space by providing subclasses which have only t, only i, only b, t & i, t & b, etc.
        simtime_t t;
        int64_t i, ix;
        std::string b;
        
        SMNNodeEvent(int _id, OBNSimMsg::N2SMN_MSGTYPE _type, EventCategory _cat):
        nodeID(_id), type(_type), category(_cat), has_t(0), has_i(0), has_ix(0), has_b(0)
        { }
    };
}


#endif /* OBNSIM_EVENT_H_ */
