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


#include <obnnode_yarpnode.h>

using namespace std;
using namespace OBNnode;
using namespace OBNSimMsg;


// The main network object
yarp::os::Network YarpNodeBase::yarp_network;

// Trim a string from spaces at both ends
string trim(const string& s0) {
    string s(s0);
    size_t found = s.find_last_not_of(" \t\f\v\n\r");
    if (found != string::npos) {
        s.erase(found+1);
        found = s.find_first_not_of(" \t\f\v\n\r");
        s.erase(0, found);
    } else {
        s.clear();
    }
    
    return s;
}


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
        bool success = _smn_port.open(_workspace + _nodeName + "/_gc_");
        if (success) {
            _smn_port.useCallback();        // Always use callback for the SMN port
            _smn_port.setStrict();          // Make sure that no messages from the SMN are dropped
        }
        return success;
    }
    return true;
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


bool YarpNodeBase::attachAndOpenPort(YarpPortBase * port) {
    assert(port != nullptr);
    if (port->isValid()) {
        // Port must have not been attached
        return false;
    }
    if (!port->attach(this)) {
        return false;
    }
    if (!port->configure()) {
        return false;
    }
    
    // Open the port
    return port->getYarpPort().open(fullPortName(port->getPortName()));
}

/** This method adds an existing (physical) input port to the node.
 The port will be automatically managed by the node (reading, acknowledging, etc.).
 The port object must not be already attached to any node (i.e. calling isValid() on it must return false).
 The port object will be attached to this node and initialized and opened (connected to the Yarp network).
 The port object's life must be managed outside the node object, for example if it was dynamically allocated, it must be deleted by user's code.  The node object will only be responsible for managing the communication activity of the port, and closing the port's activity when the node is destroyed.
 \param port Pointer to the port object.
 \return true if successful; false if error.
 */
bool YarpNodeBase::addInput(YarpPortBase* port) {
    bool result = attachAndOpenPort(port);
    if (result) {
        // Add this port to the list of inputs
        _input_ports.push_front(port);
    }
    else {
        // Detach it, making invalid again
        port->detach();
    }
    return result;
}


/** This method adds an existing (physical) output port to the node.
 The port will be automatically managed by the node (sending, acknowledging, etc.).
 The port object must not be already attached to any node (i.e. calling isValid() on it must return false).
 The port object will be attached to this node and initialized and opened (connected to the Yarp network).
 The port object's life must be managed outside the node object, for example if it was dynamically allocated, it must be deleted by user's code.  The node object will only be responsible for managing the communication activity of the port, and closing the port's activity when the node is destroyed.
 \param port Pointer to the port object.
 \return true if successful; false if error.
 */
bool YarpNodeBase::addOutput(YarpOutputPortBase* port) {
    bool result = attachAndOpenPort(port);
    if (result) {
        // Add this port to the list of inputs
        _output_ports.push_front(port);
    }
    else {
        // Detach it, making invalid again
        port->detach();
    }
    return result;
}

/** The constructor of a node object. It initializes the node object with name, optional workspace name, and initial state.
 \param name The required name of the node; must be a valid node name.
 \param ws The optional workspace; must be a valid identifier. The workspace name is prepended to all names used by this node, including the SMN. Default workspace is "" (i.e. no workspace).
 */
YarpNodeBase::YarpNodeBase(const string& _name, const string& ws): _smn_port(this) {
    _nodeName = trim(_name);
    assert(OBNsim::Utils::isValidNodeName(_nodeName));
    
    _workspace = trim(ws);
    if (_workspace.empty()) {
        _workspace = "/";
    } else {
        _workspace.append("/").insert(_workspace.begin(), '/');     // Prepend and append it with '/'
    }
    
    // Initialize the node (setting its internal parameters)
    _node_id = 0;
    _node_state = NODE_STOPPED;
}

YarpNodeBase::~YarpNodeBase() {
    // Tell all input ports to detach from this node
    for (auto port : _input_ports) {
        port->detach();
    }
    
    // Tell all output ports to detach from this node
    for (auto port : _output_ports) {
        port->detach();
    }
}

/** This method tries to remove a port from this node.
 The port object can be an input or a generic data port.
 The detach() method of the port won't be called.
 \param port Pointer to the port object (must be valid).
 */
void YarpNodeBase::removePort(YarpPortBase* port) {
    assert(port);

    _input_ports.remove(port);
    //_data_ports.remove(port);
}

/** This method tries to remove a port from this node.
 The port object is an output port.
 The detach() method of the port won't be called.
 \param port Pointer to the port object (must be valid).
 */
void YarpNodeBase::removePort(YarpOutputPortBase* port) {
    assert(port);

    _output_ports.remove(port);
}

//unsigned int YarpNodeBase::createPort(const string& name, bool bCallback, bool strict) {
//    auto sName = trim(name);
//    if (sName.empty()) {
//        reportError("YarpNodeBase:invalid_port_name", "Invalid port's name.");
//    }
//    
//    // Check if the name already existed
//    if (portsByName.find(sName) != portsByName.end()) {
//        reportError("YarpNodeBase:invalid_port_name", "Port's name already existed.");
//    }
//    
//    unsigned int id = nextPortID++;
//    portsByName.emplace(sName, ports.emplace(id, new YPort(this, sName, id, bCallback, strict)).first->second);
//    
//    return id;
//}
//


bool YarpNodeBase::connectWithSMN(const char *carrier) {
    if (!openSMNPort()) { return false; }
    if (carrier) {
        return yarp_network.connect(fullPortName("_gc_"), _workspace + "_smn_/_gc_", carrier) &&   // Connect from node to SMN
        yarp_network.connect(_workspace + "_smn_/" + _nodeName, fullPortName("_gc_"), carrier);    // Connect from SMN to node
    } else {
        return yarp_network.connect(fullPortName("_gc_"), _workspace + "_smn_/_gc_") &&   // Connect from node to SMN
        yarp_network.connect(_workspace + "_smn_/" + _nodeName, fullPortName("_gc_"));    // Connect from SMN to node
    }
}


/** This method initializes the node before a simulation starts. This is different from the callback for the INIT message. This method is called before the node even starts waiting for the INIT message. */
void YarpNodeBase::initializeForSimulation() {
    _current_sim_time = 0;
    _current_updates = 0;
}


/** This method sends a simple ACK message to the SMN.
 \param type The type of the ACK message.
 */
void YarpNodeBase::sendACK(OBNSimMsg::N2SMN::MSGTYPE type) {
    YarpNodeBase::SMNMsg& msg = _smn_port.prepare();
    
    _n2smn_message.set_msgtype(type);
    _n2smn_message.set_id(_node_id);
    
    msg.setMessage(_n2smn_message);
    _smn_port.writeStrict();
}

/** This method sends a simple ACK message to the SMN.
 \param type The type of the ACK message.
 \param I Integer value for MSGDATA.I
 */
void YarpNodeBase::sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I) {
    YarpNodeBase::SMNMsg& msg = _smn_port.prepare();
    
    _n2smn_message.set_msgtype(type);
    _n2smn_message.set_id(_node_id);
    
    auto *data = new OBNSimMsg::MSGDATA;
    data->set_i(I);
    _n2smn_message.set_allocated_data(data);
    
    msg.setMessage(_n2smn_message);
    _smn_port.writeStrict();
}


/** This method sends a simple ACK message to the SMN.
 \param type The type of the ACK message.
 \param I Integer value for MSGDATA.I
 \param IX Integer value for MSGDATA.IX
 */
void YarpNodeBase::sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I, int64_t IX) {
    YarpNodeBase::SMNMsg& msg = _smn_port.prepare();
    
    _n2smn_message.set_msgtype(type);
    _n2smn_message.set_id(_node_id);
    
    auto *data = new OBNSimMsg::MSGDATA;
    data->set_i(I);
    data->set_ix(IX);
    _n2smn_message.set_allocated_data(data);
    
    msg.setMessage(_n2smn_message);
    _smn_port.writeStrict();
}

/** This method runs the node in the openBuildNet simulation network.
 The node must start from state NODE_STOPPED, otherwise it will return immediately.
 A timeout in seconds can be given (default: -1). If the timeout is positive, the node will wait for new events (from the network) only up to that timeout. If a timeout occurs, the callback for timeout will be called but the simulation will not stop automatically (not even error); it's up to the callback to decide what to do with this situation, e.g. it can change the node's state to ERROR, or simply terminate the simulation.
 Be careful using the timeout as it may terminate the node unexpectedly, e.g. when the simulation goes to Debugging mode, or some other node just needs long computation time.
 \param timeout The timeout value; non-positive if there is no timeout.
 */
void YarpNodeBase::run(double timeout) {
    // Make sure that the SMN port is opened (but won't connect it)
    if (!openSMNPort()) {
        // Error
        _node_state = NODE_ERROR;
        onReportInfo("[NODE] Could not open the SMN port. Check the network and the name server.");
        return;
    }
    
    // Must start from the STOPPED state
    if (_node_state != NODE_STOPPED) {
        onReportInfo("[NODE] Node can't be started from a non-STOPPED state. Make sure the node is stopped before starting it.");
        return;
    }

    initializeForSimulation();
    _node_state = NODE_STARTED;     // Node has started, but not yet initialized
    
    // Looping to process events until the simulation stops or a timeout occurs
    std::shared_ptr<NodeEvent> pEvent;
    if (timeout <= 0.0) {
        // Without timeout
        while (_node_state == NODE_RUNNING || _node_state == NODE_STARTED) {
            pEvent = _event_queue.wait_and_pop();
            assert(pEvent);
            pEvent->executeMain(this);  // Execute the event
            pEvent->executePost(this);  // Post-Execute the event
        }
    } else {
        // With timeout
        while (_node_state == NODE_RUNNING || _node_state == NODE_STARTED) {
            pEvent = _event_queue.wait_and_pop_timeout(timeout);
            if (pEvent) {
                // Execute the event if not timeout
                pEvent->executeMain(this);
                pEvent->executePost(this);
            } else {
                // If timeout then we stop
                onRunTimeout();
                onReportInfo("[NODE] Node's execution has a timeout.");
                return;
            }
        }
    }
    
    // This is the end of the simulation
    onReportInfo("[NODE] Node's execution has stopped.");
}


/** This method will request/notify the SMN to stop, then terminate the current node's simulation regardless of whether the request was accepted or not.
 This is like an emergency shutdown. To simply request to stop the simulation, but may not actually stop if the request is not accepted by the SMN, use the function requestStopSimulation().
 This method also clear any error in the node.
 It's guaranteed that after this method, the node's state is STOPPED.
 \sa  requestStopSimulation().
 */
void YarpNodeBase::stopSimulation() {
    if (_node_state == NODE_RUNNING) {
        // Notify/request the SMN to stop ...
        requestStopSimulation();
        // ... but shutdown anyway
        onTermination();
    }

    // Always switch to STOPPED, even if it's not running (e.g. already STOPPED, or just STARTED, or has ERROR)
    _node_state = NODE_STOPPED;
}



/** This method requests the SMN/GC to stop the simulation (by sending a request message to the SMN).
 However, the SMN/GC may deny the request and continue the simulation.
 This method will not stop the current node, so if the request is not accepted, the node will continue running.
 To stop the simulation definitely, use the function stopSimulation().
 If the node is currently in ERROR state, this method will not clear the error, use the function stopSimulation() instead.
 */
void YarpNodeBase::requestStopSimulation() {
    // Currently doing nothing because the request message has not been designed yet.
    if (_node_state == NODE_RUNNING || _node_state == NODE_STARTED) {
        // Send the request
    }
}


/** This is a very important method. It takes an SMN2N message (system message) and generates an appropriate node event.
In the case the system message is urgent and important (e.g. an urgent shutdown, a system-wide error), it will immediately act on the event.
 \param msg Reference to the SMN2N message.
*/
void YarpNodeBase::postEvent(const OBNSimMsg::SMN2N& msg) {
    // The cases should be ordered in the frequency of the message types
    switch (msg.msgtype()) {
        case SMN2N_MSGTYPE_SIM_Y:
            _event_queue.push(new NodeEvent_UPDATEY(msg));
            break;
            
        case SMN2N_MSGTYPE_SIM_X:
            _event_queue.push(new NodeEvent_UPDATEX(msg));
            break;
            
        case SMN2N_MSGTYPE_SIM_EVENT_ACK:
            checkWaitForCondition(msg);
            break;
            
        case SMN2N_MSGTYPE_SIM_TERM:
            // Stop the simulation (often at the end of the simulation time, or requested by the user, but not because of a system error
            _event_queue.push(new NodeEvent_TERMINATE(msg));
            break;
            
        case SMN2N_MSGTYPE_SIM_INIT:
            _event_queue.push(new NodeEvent_INITIALIZE(msg));
            break;
            
        default:
            onOBNWarning("Unrecognized system message from SMN with type " + std::to_string(msg.msgtype()));
            break;
    }
}


/** Handle UPDATE_X events: Main. */
void YarpNodeBase::NodeEvent_UPDATEX::executeMain(YarpNodeBase* pnode) {
    // Skip if the node is not RUNNING
    if (pnode->_node_state != YarpNodeBase::NODE_RUNNING) {
        return;
    }

    basic_processing(pnode);

    // Call the callback to perform UPDATE_X
    pnode->onUpdateX();
}


/** Handle UPDATE_X events: Post. */
void YarpNodeBase::NodeEvent_UPDATEX::executePost(YarpNodeBase* pnode) {
    // Send ACK to the SMN
    pnode->sendACK(OBNSimMsg::N2SMN_MSGTYPE_SIM_X_ACK);
}

/** Handle UPDATE_Y events: Main. */
void YarpNodeBase::NodeEvent_UPDATEY::executeMain(YarpNodeBase* pnode) {
    // Skip if the node is not RUNNING; this should never happen if the node is used properly
    if (pnode->_node_state != YarpNodeBase::NODE_RUNNING) {
        return;
    }
    
    basic_processing(pnode);
    
    // Save the current update type mask, just in case UPDATE_X needs it
    pnode->_current_updates = _updates;
    
    // Call the callback to perform UPDATE_Y
    pnode->onUpdateY(_updates);
}

/** Handle UPDATE_Y events: Post. */
void YarpNodeBase::NodeEvent_UPDATEY::executePost(YarpNodeBase* pnode) {
    // Send out values from output ports which have been updated
    for (auto port: pnode->_output_ports) {
        if (port->isChanged()) {
            //TODO: Should change this to asynchronous send.
            port->sendSync();
            if (pnode->hasError()) {
                break;
            }
        }
    }
    
    // Send ACK to the SMN, regardless of whether it had an error or not
    // If an error happened and the node should stop, it should also send an error message to the SMN to notify it
    pnode->sendACK(OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK);
}

/** Handle Initialization before simulation: Main. */
void YarpNodeBase::NodeEvent_INITIALIZE::executeMain(YarpNodeBase* pnode) {
    // Skip if the node is not STARTED.
    // This actually should never happen if the node is used properly, because the run() method will not let the node run unless it's in a proper state.
    if (pnode->_node_state != YarpNodeBase::NODE_STARTED) {
        return;
    }
    
    basic_processing(pnode);
    
    // Save the initial settings sent from the SMN
    if (_has_wallclock) {
        pnode->_initial_wallclock = _wallclock;
    }
    if (_has_timeunit) {
        pnode->_timeunit = _timeunit;
    }
    
    pnode->onInitialization();
}

/** Handle Initialization before simulation: Post. */
void YarpNodeBase::NodeEvent_INITIALIZE::executePost(YarpNodeBase* pnode) {
    // Send out values from output ports, whether updated or not
    for (auto port: pnode->_output_ports) {
        //TODO: Should change this to asynchronous send.
        port->sendSync();
        if (pnode->hasError()) {
            break;
        }
    }
    
    // Set the node's state to RUNNING if it's still STARTED (if it's ERROR, we won't change it)
    if (pnode->_node_state == YarpNodeBase::NODE_STARTED) {
        pnode->_node_state = YarpNodeBase::NODE_RUNNING;
    }
    
    // Send an ACK message to the SMN
    if (pnode->_node_state == YarpNodeBase::NODE_RUNNING) {
        // Successful
        pnode->sendACK(OBNSimMsg::N2SMN::MSGTYPE::N2SMN_MSGTYPE_SIM_INIT_ACK);
    } else {
        // Unsuccessful => ACK with data.I > 0
        pnode->sendACK(OBNSimMsg::N2SMN::MSGTYPE::N2SMN_MSGTYPE_SIM_INIT_ACK, 1);
    }
}

/** Handle Termination: Main. */
void YarpNodeBase::NodeEvent_TERMINATE::executeMain(YarpNodeBase* pnode) {
    // Skip if the node is not RUNNING
    if (pnode->_node_state != YarpNodeBase::NODE_RUNNING) {
        return;
    }

    basic_processing(pnode);

    // Set the node's state to STOPPED
    // We change the node's state before calling the callback onTermination() because this system message means that simulation is definitely going to terminate immediately.
    if (pnode->_node_state == YarpNodeBase::NODE_RUNNING) {
        pnode->_node_state = YarpNodeBase::NODE_STOPPED;
    }
    
    pnode->onTermination();
}

/** Handle Termination: Post. */
void YarpNodeBase::NodeEvent_TERMINATE::executePost(YarpNodeBase* pnode) {
    // No ACK is needed
}

/**
 \param t_idx The desired index of the new update.
 \param t_T The sampling time in microseconds, <= 0 if non-periodic.
 \param t_ycallback Output callback function for UPDATE_Y message of this update.
 \param t_xcallback State callback function for UPDATE_X message of this update.
 \param t_name Optional name of the update.
 \return The index of this update, or -1 if the index is out of range, or -2 if an update already exists at that index.
 */
int YarpNode::addUpdate(int t_idx, double t_T, UpdateType::UPDATE_CALLBACK t_ycallback, UpdateType::UPDATE_CALLBACK t_xcallback, const UpdateType::INPUT_LIST& t_inputs, const UpdateType::OUTPUT_LIST& t_outputs, const std::string& t_name) {
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
 \return The index of this update, or -1 if no more updates could be added (the node runs out of allowed number of updates).
 */
int YarpNode::addUpdate(double t_T, UpdateType::UPDATE_CALLBACK t_ycallback, UpdateType::UPDATE_CALLBACK t_xcallback, const UpdateType::INPUT_LIST& t_inputs, const UpdateType::OUTPUT_LIST& t_outputs, const std::string& t_name) {
    
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
    return addUpdate(t_update.T_in_us, t_update.y_callback, t_update.x_callback,
                     t_update.inputs, t_update.outputs, t_update.name);
}

/**
 \param t_idx The desired index of the new update.
 \param t_update The definition of the new update.
 \return See the other addUpdate() for details.
 */
int YarpNode::addUpdate(int t_idx, const UpdateType& t_update) {
    return addUpdate(t_idx, t_update.T_in_us, t_update.y_callback, t_update.x_callback,
                     t_update.inputs, t_update.outputs, t_update.name);
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
void YarpNode::onUpdateX() {
    auto m = _current_updates;
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