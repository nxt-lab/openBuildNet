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


/** Dynamically create a new node event object from an N2SMN message and push it to the queue.
 This method should be used to convert an N2SMN message to a node event because it also checks for the validity of the message and assigns a suitable category.
 
 \param msg The N2SMN message object.
 \param defaultID The default ID of the node that sent the message, to be used only if it's not included in the message itself or if the ID is overridden.
 \param overrideID true if the message's ID is always overridden by defaultID; false if defaultID is only used if the message does not contain an ID.
 \return true if successful (a valid event is pushed to the queue).
 */
bool OBNsmn::GCThread::pushNodeEvent(const OBNSimMsg::N2SMN& msg, int defaultID, bool overrideID) {
    int ID = overrideID?defaultID:(msg.has_id()?msg.id():defaultID);
    OBNSimMsg::N2SMN::MSGTYPE type = msg.msgtype();
    
    // Check that the ID is valid, otherwise it may cause security problem
    if (ID > maxID || ID < 0) {
        report_warning(0, "Received message (type: " + std::to_string(type) + ") with an invalid ID (" + std::to_string(ID) + ").");
        return false;
    }
    
    OBNsmn::SMNNodeEvent::EventCategory cat;

    // In the switch-case, put the more frequent/typical cases first, to improve performance
    switch (type) {
        case OBNSimMsg::N2SMN::SIM_Y_ACK:
        case OBNSimMsg::N2SMN::SIM_X_ACK:
        case OBNSimMsg::N2SMN::SIM_YI_ACK:
            pushEvent(new SMNNodeEvent(ID, type, OBNsmn::SMNNodeEvent::EVT_ACK));
            return true;
            
        case OBNSimMsg::N2SMN::SIM_INIT_ACK:
            cat = OBNsmn::SMNNodeEvent::EVT_ACK;
            break;
            
        default:
            cat = OBNsmn::SMNNodeEvent::EVT_SIM;
    }
    
    OBNsmn::SMNNodeEvent* pe = new OBNsmn::SMNNodeEvent(ID, type, cat);
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
