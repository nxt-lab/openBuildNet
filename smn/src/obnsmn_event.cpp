/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implement the event objects in the Global Clock.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnsmn_gc.h>
#include <obnsmn_report.h>


bool OBNsmn::GCThread::gc_waitfor_process_ACK(const OBNSimMsg::N2SMN& msg, int ID) {
    OBNSimMsg::N2SMN::MSGTYPE type = msg.msgtype();
    
    std::lock_guard<std::mutex> lock(gc_waitfor_mutex);
    if (gc_waitfor_status == GC_WAITFOR_RESULT_ERROR) { return true; }
    if (gc_waitfor_status != GC_WAITFOR_RESULT_ACTIVE || gc_waitfor_type != type) {
        report_warning(0, "Unexpected ACK received from node " + std::to_string(ID) + " with type " + std::to_string(type));
        return true;
    }
    if (gc_waitfor_predicate && !gc_waitfor_predicate(msg)) {
        // Invalid ACK messasge
        gc_waitfor_status = GC_WAITFOR_RESULT_ERROR;
        mWakeupCondition.notify_all();
        return true;
    }
    // This ACK message has been checked -> check it in the bit array
    if (!gc_waitfor_bits[ID]) {
        gc_waitfor_bits[ID] = true;
        gc_waitfor_num--;
    }
    if (gc_waitfor_num == 0) {
        gc_waitfor_status = GC_WAITFOR_RESULT_DONE;
        mWakeupCondition.notify_all();
    }
    return true;
}


/** Except for ACK messages, this method dynamically create a new node event object from an N2SMN message and push it to the queue.
 This method should be used to convert an N2SMN message to a node event because it also checks for the validity of the message and assigns a suitable category.
 
 \param msg The N2SMN message object.
 \param defaultID The default ID of the node that sent the message, to be used only if it's not included in the message itself or if the ID is overridden.
 \param overrideID true if the message's ID is always overridden by defaultID; false if defaultID is only used if the message does not contain an ID.
 \return true if successful.
 */
bool OBNsmn::GCThread::pushNodeEvent(const OBNSimMsg::N2SMN& msg, int defaultID, bool overrideID) {
    int ID = overrideID?defaultID:(msg.has_id()?msg.id():defaultID);
    bool hasID = msg.has_id() || overrideID;
    OBNSimMsg::N2SMN::MSGTYPE type = msg.msgtype();
    
    bool isNotSysMsg = static_cast<uint16_t>(type) >= 0x0100;   // Not a system management message
    
    // If this is not a system management message (i.e. if type >= 0x0100),
    // check that the ID is valid, otherwise it may cause security problem
    if (isNotSysMsg && (!hasID || ID > maxID || ID < 0)) {
        report_warning(0, "Received message (type: " + std::to_string(type) + ") with an invalid ID (" + std::to_string(ID) + ").");
        return false;
    }
    
    OBNsmn::SMNNodeEvent::EventCategory cat = OBNsmn::SMNNodeEvent::EVT_SYS;

    if (isNotSysMsg) {
        // In the switch-case, put the more frequent/typical cases first, to improve performance
        switch (type) {
            case OBNSimMsg::N2SMN::SIM_Y_ACK:
            case OBNSimMsg::N2SMN::SIM_X_ACK:
            case OBNSimMsg::N2SMN::SIM_INIT_ACK:
                // pushEvent(new SMNNodeEvent(type, OBNsmn::SMNNodeEvent::EVT_ACK, ID));
                // return true;
                // Process wait-for here
                return gc_waitfor_process_ACK(msg, ID);
                
//            case OBNSimMsg::N2SMN::SIM_INIT_ACK:
//                cat = OBNsmn::SMNNodeEvent::EVT_ACK;
//                break;
                
            default:
                cat = OBNsmn::SMNNodeEvent::EVT_SIM;
        }
    }
    
    // For complex events with attached data
    OBNsmn::SMNNodeEvent* pe = new OBNsmn::SMNNodeEvent(type, cat, ID, hasID);
    if (msg.has_data()) {
        OBNSimMsg::MSGDATA data = msg.data();
        if (data.has_b()) {
            pe->has_b = 1;
            pe->b = data.b();
        }
        if (data.has_t()) {
            pe->has_t = 1;
            pe->t = data.t();
        }
        if (data.has_i()) {
            pe->has_i = 1;
            pe->i = data.i();
        }
    }
    
    pushEvent(pe);
    return true;
}
