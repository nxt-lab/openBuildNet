/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief For inline functions only used in the GC/SMN algorithm.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef smnchai_obnsmn_gc_inline_h
#define smnchai_obnsmn_gc_inline_h

#include <obnsmn_gc.h>
#include <obnsmn_report.h>

inline void OBNsmn::GCThread::gc_process_msg_sim_event(OBNsmn::SMNNodeEvent* pEv) {
    OBNSimMsg::SMN2N msg;
    msg.set_msgtype(OBNSimMsg::SMN2N_MSGTYPE_SIM_EVENT_ACK);
    msg.set_time(current_sim_time);
    
    auto *data = new OBNSimMsg::MSGDATA;
    
    // Request for an irregular update
    // It must contain the requested time
    if (pEv->has_t) {
        data->set_t(pEv->t);
        
        if (pEv->t > current_sim_time) {
            // Requested time is in the future: it's accepted
            data->set_i(0);  // OK
            _nodes[pEv->nodeID]->insertIrregularUpdate(pEv->t, (pEv->has_i)?pEv->i:0);
            //report_info(0, "Accept event for node " + _nodes[pEv->nodeID]->name + " for mask " + std::to_string((pEv->has_i)?pEv->i:0) + " at time " + std::to_string(pEv->t));
        }
        else {
            // Requested time is invalid: denied
            report_warning(0, "An invalid irregular update request from node " + std::to_string(pEv->nodeID) +
                           " (" + _nodes[pEv->nodeID]->name + ") with past time.");
            
            data->set_i(1);  // Error code 1 = invalid requested time (past time)
        }
    }
    else {
        report_warning(0, "An irregular update request from node " + std::to_string(pEv->nodeID) +
                       " (" + _nodes[pEv->nodeID]->name + ") doesn't specify time; it is ignored.");
        data->set_t(0);
        data->set_i(2);  // Error code 2 = no time requested
    }
    
    msg.set_allocated_data(data);
    _nodes[pEv->nodeID]->sendMessage(pEv->nodeID, msg);
}


inline bool OBNsmn::GCThread::gc_process_msg_sys_request_stop(OBNsmn::SMNNodeEvent* pEv) {
    // This method processes the SYS_REQUEST_STOP
    // The message can come from an existing node or not
    // Ideally, we should have switches to: accept this type of requests or not (default: true), accept requests from non-existing node (true -> less secured), accept/reject based on the request reason code.
    // For now, we always accept the request.
    int success = 0;
    std::string reason; // Reason of rejection
    
    if (pEv->has_id) {
        // It comes from an existing node
        if (pEv->nodeID > maxID || pEv->nodeID < 0) {
            // Invalid node ID -> report and return, no ACK sent back
            report_warning(0, "Request to stop simulation from an invalid node.");
            return true;
        }
        report_info(0, "Request to stop simulation from node " + std::to_string(pEv->nodeID) + " (" + _nodes[pEv->nodeID]->name + ").");
    } else {
        report_info(0, "Request to stop simulation from an anonymous node.");
    }
    
    if (gc_exec_state != GCSTATE_RUNNING && gc_exec_state != GCSTATE_PAUSED) {
        // Not running -> can't stop
        success = 1;
    }
    
    // Prepare the ACK message
    OBNSimMsg::SMN2N msg;
    msg.set_msgtype(OBNSimMsg::SMN2N_MSGTYPE_SYS_REQUEST_STOP_ACK);
    msg.set_time(current_sim_time);
    if (pEv->has_id) {
        msg.set_id(pEv->nodeID);
    } else {
        msg.clear_id();
    }
    msg.set_i(success);
    
    if (!reason.empty()) {
        auto *data = new OBNSimMsg::MSGDATA;
        data->set_b(reason);
        msg.set_allocated_data(data);
    }
    
    // Send the ACK message either to the node or to the main GC port
    if (pEv->has_id) {
        _nodes[pEv->nodeID]->sendMessage(pEv->nodeID, msg);
    } else if (m_send_msg_to_sys_port) {
        m_send_msg_to_sys_port(msg);
    }

    if (success == 0) {
        // Actually stop the simulation
        gc_exec_state = GCSTATE_TERMINATING;
        return false;
    }
    
    return true;
}

#endif
