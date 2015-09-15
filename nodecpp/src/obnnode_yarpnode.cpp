/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief YARP node class for the C++ node interface.
 *
 * Implement the main node class for a C++ node.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <chrono>
#include <thread>
#include <obnnode_yarpnode.h>
#include <obnnode_exceptions.h>

using namespace std;
using namespace OBNnode;
using namespace OBNSimMsg;


// The main network object
yarp::os::Network OBNnode::YarpNodeBase::yarp_network;


/* =============================================================================
        Implementation of the SMNPort class for communication with the SMN
   ============================================================================= */

/** Callback of the GC port on the node, which posts SMN events to the node's queue. */
void YarpNodeBase::SMNPort::onRead(SMNMsg& b) {
    // printf("Callback[%s]\n", getName().c_str());
    
    // Parse the ProtoBuf message
    if (!b.getMessage(_smn_message)) {
        // Error while parsing the raw message
        _the_node->onOBNError("Error while parsing a system message from the SMN.");
        return;
    }
    
    // Post the corresponding event to the queue
    _the_node->postEvent(_smn_message);
}

bool YarpNodeBase::openSMNPort() {
    if (_smn_port.isClosed()) {
        bool success = _smn_port.open('/' + _workspace + _nodeName + "/_gc_");
        if (success) {
            _smn_port.useCallback();        // Always use callback for the SMN port
            _smn_port.setStrict();          // Make sure that no messages from the SMN are dropped
        }
        return success;
    }
    return true;
}

void YarpNodeBase::sendN2SMNMsg() {
    YarpNodeBase::SMNMsg& msg = _smn_port.prepare();
    msg.setMessage(_n2smn_message);
    _smn_port.writeStrict();
}


/** This method requests a future update from the Global Clock by sending a request to the SMN.
 The request still needs to be approved by the SMN, by an acknowledgement (ACK) message from the SMN.
 The ACK will tell if the request is accepted or rejected (and the reason for the rejection).
 If waiting = true, the function will wait (block) until it receives the ACK; otherwise it will return immediately.
 If the method returns a valid pointer to an WaitForCondition object, the request has been sent; if it returns nullptr, the request is invalid (e.g. a past or present update is requested).
 In case waiting = false, the ACK can be waited for later on by calling wait() on the returned object.
 Once the ACK has been received (the request has been answered), the result of the request can be checked by the I field of the returned data record (accessed by calling getData() on the condition object, see the OBN document for details).
 Make sure that the condition has been cleared (either after waiting for it or by calling YarpNodeBase::isCleared()) before accessing its data, otherwise the content of the data record is undefined or it may cause a data race issue.
 \param t The time of the requested update; must be in the future t > current simulation time
 \param m The update mask requested for the update.
 \param waiting Whether the method should wait (blocking/synchronously) for the ACK to receive [default: true]; if a timeout is desired, call this function with waiting=false and explicitly call wait() on the returned condition object.
 \return A pointer to the wait-for condition, which is used to wait for the ACK; or nullptr if the request is invalid.
 */
YarpNodeBase::WaitForCondition* YarpNodeBase::requestFutureUpdate(simtime_t t, updatemask_t m, bool waiting) {
    if (t <= _current_sim_time) {
        // Cannot request a present or past update time
        return nullptr;
    }
    
    // Send request to the SMN
    YarpNodeBase::SMNMsg& msg = _smn_port.prepare();
    _n2smn_message.set_msgtype(OBNSimMsg::N2SMN_MSGTYPE_SIM_EVENT);
    _n2smn_message.set_id(_node_id);
    
    auto *data = new OBNSimMsg::MSGDATA;
    data->set_t(t);
    data->set_i(m);
    _n2smn_message.set_allocated_data(data);
        
    msg.setMessage(_n2smn_message);
    _smn_port.writeStrict();
    
    
    // Register an wait-for condition (lock mutex at beginning and unlock it after we've done)
    YarpNodeBase::WaitForCondition *pCond = nullptr;
    YarpNodeBase::WaitForCondition::the_checker f = [t](const OBNSimMsg::SMN2N& msg) {
        return msg.msgtype() == OBNSimMsg::SMN2N_MSGTYPE_SIM_EVENT_ACK && (msg.has_data() && (msg.data().has_t() && msg.data().t() == t));
    };
    
    _waitfor_conditions_mutex.lock();
    
    // Look through the list of conditions to find an inactive one
    for (auto c = _waitfor_conditions.begin(); c != _waitfor_conditions.end(); ++c) {
        if (c->status == WaitForCondition::INACTIVE) {
            // Found one => reuse it
            c->_check_func = f;
            c->status = WaitForCondition::ACTIVE;
            pCond = &(*c);
            break;
        }
    }
    
    if (!pCond) {
        // No inactive condition can be reused => create new one
        _waitfor_conditions.emplace_front(f);
        pCond = &(_waitfor_conditions.front());
    }
    
    _waitfor_conditions_mutex.unlock();     // We've done changing the list

    // If waiting = true, we will wait (blocking) until the wait-for condition is cleared; otherwise, just return
    if (waiting) {
        pCond->wait(-1.0);
    }
    
    return pCond;
}

/** This method returns the result of a pending request for a future update. If the request hasn't been acknowledged by the SMN, this method will wait (block) until it receives the ACK for this request. It returns the value of the I field of the ACK message's data (see the OBN design document for details). Basically if it returns 0, the request was successful; otherwise there was an error and the request failed.
  This method will reset the condition after it's cleared.
 \param pCond Pointer to the condition object, as returned by requestFutureUpdate().
 \param timeout An optional timeout value; if a timeout occurs and the waiting failed then the returned value will be -1.
 \return The result of the request: 0 if successful; -1 if the waiting failed (due to timeout).
 \sa requestFutureUpdate()
 */
int64_t YarpNodeBase::resultFutureUpdate(YarpNodeBase::WaitForCondition* pCond, double timeout) {
    return resultWaitForCondition(pCond);
}

/** This method waits until a wait-for condition is cleared and returns the value of the integer field I of the message data. This method does not check if the return message actually had the message data and the I field; if it did not, the default value (0) is returned.
 This method will reset the condition after it's cleared.
 \param pCond Pointer to the condition object.
 \param timeout An optional timeout value; if a timeout occurs and the waiting failed then the returned value will be -1.
 \return The integer field I of the message data if the waiting is successful (default to 0 if I does not exist); or -1 if the waiting failed (due to timeout).
 */
int64_t YarpNodeBase::resultWaitForCondition(YarpNodeBase::WaitForCondition* pCond, double timeout) {
    assert(pCond);
    
    _waitfor_conditions_mutex.lock();
    auto s = pCond->status;
    _waitfor_conditions_mutex.unlock();
    assert(s != WaitForCondition::INACTIVE);
    if ((s == YarpNodeBase::WaitForCondition::ACTIVE)?pCond->wait(timeout):true) {
        // At this point, the status of the condition must be CLEARED => get the data and the I field
        auto i = pCond->getData().i();
        resetWaitFor(pCond);
        return i;
    } else {
        return -1;
    }
}

/** This method iterates the list of wait-for conditions and check if any of them can be cleared according to the given message. If there is one, its status will be changed to CLEARED, the MSGDATA will be saved. At most one condition can be cleared. */
void YarpNodeBase::checkWaitForCondition(const OBNSimMsg::SMN2N& msg) {
    // Lock access to the list, because the SMN port thread may access it
    yarp::os::LockGuard lock(_waitfor_conditions_mutex);
    
    // Look through the list of conditions
    for (auto c = _waitfor_conditions.begin(); c != _waitfor_conditions.end(); ++c) {
        if (c->status == WaitForCondition::ACTIVE && c->_check_func(msg)) {
            // This condition is cleared
            c->status = WaitForCondition::CLEARED;
            if (msg.has_data()) {
                c->_data.CopyFrom(msg.data());
            }
            c->_event.post();
            return;
        }
    }
}


bool YarpNodeBase::connectWithSMN(const char *carrier) {
    // Note that Yarp requires / at the beginning of any port name
    if (!openSMNPort()) { return false; }
    if (carrier) {
        return yarp::os::Network::connect('/' + fullPortName("_gc_"), '/' + _workspace + "_smn_/_gc_", carrier, false) &&   // Connect from node to SMN
        yarp::os::Network::connect('/' + _workspace + "_smn_/" + _nodeName, '/' + fullPortName("_gc_"), carrier, false);    // Connect from SMN to node
    } else {
        return yarp::os::Network::connect('/' + fullPortName("_gc_"), '/' + _workspace + "_smn_/_gc_", "", false) &&   // Connect from node to SMN
        yarp::os::Network::connect('/' + _workspace + "_smn_/" + _nodeName, '/' + fullPortName("_gc_"), "", false);    // Connect from SMN to node
    }
}


/**
 \param t_idx The desired index of the new update.
 \param t_T The sampling time in microseconds, <= 0 if non-periodic.
 \param t_ycallback Output callback function for UPDATE_Y message of this update.
 \param t_xcallback State callback function for UPDATE_X message of this update.
 \param t_name Optional name of the update.
 \return The index of this update, or an error code: -1 if the index is out of range, -2 if an update already exists at that index, -3 if an input port does not exist, -4 if an output port does not exist.
 */
int YarpNode::addUpdate(int t_idx, UpdateType::UPDATE_CALLBACK t_ycallback, UpdateType::UPDATE_CALLBACK t_xcallback, double t_T,  const UpdateType::INPUT_LIST& t_inputs, const UpdateType::OUTPUT_LIST& t_outputs, const std::string& t_name) {
    // Check index
    if (t_idx < 0 || t_idx > OBNsim::MAX_UPDATE_INDEX) {
        return -1;  // Index out of range
    }
    
    // Check that we can insert a new update to the given index
    if (m_updates.size() <= t_idx) {
        // Create the new update at the position
        m_updates.resize(t_idx+1);
    } else if (m_updates[t_idx].enabled) {
        return -2;  // Update already existed
    }
    
    // Check that specified inputs and outputs actually exist
    for (const auto &p: t_inputs) {
        if (!does_port_exists(_input_ports, p.first)) {
            return -3;
        }
    }
    
    for (const auto &p: t_outputs) {
        if (!does_port_exists(_output_ports, p)) {
            return -4;
        }
    }
    
    // Add the new update
    m_updates[t_idx].enabled = true;
    m_updates[t_idx].T_in_us = t_T;
    m_updates[t_idx].y_callback = t_ycallback;
    m_updates[t_idx].x_callback = t_xcallback;
    m_updates[t_idx].name = t_name;
    m_updates[t_idx].inputs = t_inputs;
    m_updates[t_idx].outputs = t_outputs;
    
    return t_idx;
}


/**
 \param t_T The sampling time in microseconds, <= 0 if non-periodic.
 \param t_ycallback Output callback function for UPDATE_Y message of this update.
 \param t_xcallback State callback function for UPDATE_X message of this update.
 \param t_name Optional name of the update.
 \return The index of this update, or an error code: -1 if no more updates could be added (the node runs out of allowed number of updates), -3 if an input port does not exist, -4 if an output port does not exist.
 */
int YarpNode::addUpdate(UpdateType::UPDATE_CALLBACK t_ycallback, UpdateType::UPDATE_CALLBACK t_xcallback, double t_T, const UpdateType::INPUT_LIST& t_inputs, const UpdateType::OUTPUT_LIST& t_outputs, const std::string& t_name) {
    
    // Find a free spot
    int idx = 0;
    for (; idx < m_updates.size(); ++idx) {
        if (!m_updates[idx].enabled) {
            break;
        }
    }
    
    if (idx == m_updates.size()) {
        // Could not find a free spot, check if we can add a new one
        if (m_updates.size() > OBNsim::MAX_UPDATE_INDEX) {
            return -1;
        }
        // Add one at the end
        m_updates.push_back(UpdateType());
    }
    
    // Check that specified inputs and outputs actually exist
    for (const auto &p: t_inputs) {
        if (!does_port_exists(_input_ports, p.first)) {
            return -3;
        }
    }
    
    for (const auto &p: t_outputs) {
        if (!does_port_exists(_output_ports, p)) {
            return -4;
        }
    }
    
    // Assign the new update
    m_updates[idx].enabled = true;
    m_updates[idx].T_in_us = t_T;
    m_updates[idx].y_callback = t_ycallback;
    m_updates[idx].x_callback = t_xcallback;
    m_updates[idx].name = t_name;
    m_updates[idx].inputs = t_inputs;
    m_updates[idx].outputs = t_outputs;

    return idx;
}

/**
 \param t_update The definition of the new update.
 \return See the other addUpdate() for details.
 */
int YarpNode::addUpdate(const UpdateType& t_update) {
    return addUpdate(t_update.y_callback, t_update.x_callback,
                     t_update.T_in_us, t_update.inputs, t_update.outputs, t_update.name);
}

/**
 \param t_idx The desired index of the new update.
 \param t_update The definition of the new update.
 \return See the other addUpdate() for details.
 */
int YarpNode::addUpdate(int t_idx, const UpdateType& t_update) {
    return addUpdate(t_idx, t_update.y_callback, t_update.x_callback,
                     t_update.T_in_us, t_update.inputs, t_update.outputs, t_update.name);
}

/**
 \param t_idx The index of the update to be removed.
 \return true if successful.
 */
bool YarpNode::removeUpdate(int t_idx) {
    if (0 <= t_idx && t_idx < m_updates.size()) {
        if (m_updates[t_idx].enabled) {
            m_updates[t_idx].enabled = false;
            return true;
        }
    }
    return false;
}

/** Callback for UPDATE_Y: call the appropriate callbacks in the registered updates. */
void YarpNode::onUpdateY(updatemask_t m) {
    int idx = 0;
    int n_updates = m_updates.size();
    while (m && idx < n_updates) {
        if ((m & (1 << idx)) && m_updates[idx].enabled) {
            // Call the y_callback
            if (m_updates[idx].y_callback) {
                m_updates[idx].y_callback();
            }
            // Update flag m
            m ^= (1 << idx);
        }
        idx++;
    }
}

/** Callback for UPDATE_X: call the appropriate callbacks in the registered updates. */
void YarpNode::onUpdateX(updatemask_t m) {
    int idx = 0;
    int n_updates = m_updates.size();
    while (m && idx < n_updates) {
        if ((m & (1 << idx)) && m_updates[idx].enabled) {
            // Call the x_callback
            if (m_updates[idx].x_callback) {
                m_updates[idx].x_callback();
            }
            // Update flag m
            m ^= (1 << idx);
        }
        idx++;
    }
}