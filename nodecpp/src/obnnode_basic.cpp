/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basic functions and classes for OBN node in C++.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <chrono>
#include <thread>

#include <obnsim_basic.h>
#include <obnnode_basic.h>
#include <obnnode_exceptions.h>

using namespace std;
using namespace OBNnode;
using namespace OBNSimMsg;


bool PortBase::open() {
    if (m_node != nullptr) {
        return open(m_node->fullPortName(m_name));
    } else {
        return false;
    }
}

PortBase::~PortBase() {
    //std::cout << "~PortBase" << std::endl;
    if (isValid()) {
        // Notify the node to remove me
        m_node->removePort(this);
        m_node = nullptr;
    }
}

// DO NOT REMOVE THIS DESTRUCTOR: it's important because it helps using the right NodeBase::removePort(OutputPortBase*) for output ports.
OutputPortBase::~OutputPortBase() {
    //std::cout << "~OutputPortBase" << std::endl;
    if (isValid()) {
        // Notify the node to remove me
        m_node->removePort(this);
        m_node = nullptr;
    }
}

bool NodeBase::attachAndOpenPort(PortBase * port) {
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
    return port->open();
}

/** This method adds an existing (physical) input port to the node.
 The port will be automatically managed by the node (reading, acknowledging, etc.).
 The port object must not be already attached to any node (i.e. calling isValid() on it must return false).
 The port object will be attached to this node and initialized and opened (connected to the Yarp network).
 The port object's life must be managed outside the node object, for example if it was dynamically allocated, it must be deleted by user's code.  The node object will only be responsible for managing the communication activity of the port, and closing the port's activity when the node is destroyed.
 \param port Pointer to the port object.
 \param owned True if this node owns the port object and will delete it; false by default.
 \return true if successful; false if error.
 */
bool NodeBase::addInput(PortBase* port, bool owned) {
    bool result = attachAndOpenPort(port);
    if (result) {
        // Add this port to the list of inputs
        _input_ports.emplace_front(port, owned);
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
 \param owned True if this node owns the port object and will delete it; false by default.
 \return true if successful; false if error.
 */
bool NodeBase::addOutput(OutputPortBase* port, bool owned) {
    bool result = attachAndOpenPort(port);
    if (result) {
        // Add this port to the list of inputs
        _output_ports.emplace_front(port, owned);
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
NodeBase::NodeBase(const string& _name, const string& ws) {
    _nodeName = OBNsim::Utils::trim(_name);
    assert(OBNsim::Utils::isValidNodeName(_nodeName));
    
    _workspace = OBNsim::Utils::trim(ws);
    if (!_workspace.empty()) {
        _workspace.append("/");     // append it with '/'
    }
    
    // Initialize the node (setting its internal parameters)
    _node_id = 0;
    _node_state = NODE_STOPPED;
}

NodeBase::~NodeBase() {
    // Tell all input ports to detach from this node
    for (auto port : _input_ports) {
        port.first->detach();
        
        // Delete the port object if this node owns it
        if (port.second) {
            delete port.first;
        }
    }
    
    // Tell all output ports to detach from this node
    for (auto port : _output_ports) {
        port.first->detach();
        
        // Delete the port object if this node owns it
        if (port.second) {
            delete port.first;
        }
    }
}

/** This method tries to remove a port from this node.
 The port object can be an input or a generic data port.
 The detach() method of the port won't be called. The port object won't be deleted even if this node owns the port object.
 \param port Pointer to the port object (must be valid).
 */
void NodeBase::removePort(PortBase* port) {
    assert(port);
    
    _input_ports.remove_if([port](decltype(_input_ports)::const_reference pair){ return pair.first == port; });
    
    //_data_ports.remove(port);
}

/** This method tries to remove a port from this node.
 The port object is an output port.
 The detach() method of the port won't be called. The port object won't be deleted even if this node owns the port object.
 \param port Pointer to the port object (must be valid).
 */
void NodeBase::removePort(OutputPortBase* port) {
    assert(port);
    
    _output_ports.remove_if([port](decltype(_output_ports)::const_reference pair){ return pair.first == port; });;
}


void NodeBase::delayBeforeShutdown() const {
    // Delay by an amount proportional to the node's ID
    // In ms, with a max value (we don't want to wait too long)
    auto mydelay = (std::max(0, _node_id) % 400) * 10;
    std::this_thread::sleep_for(std::chrono::milliseconds(mydelay));
}


/** This method initializes the node before a simulation starts. This is different from the callback for the INIT message. This method is called before the node even starts waiting for the INIT message. */
void NodeBase::initializeForSimulation() {
    _current_sim_time = 0;
}


/** This method sends a simple ACK message to the SMN.
 \param type The type of the ACK message.
 */
void NodeBase::sendACK(OBNSimMsg::N2SMN::MSGTYPE type) {
    _n2smn_message.Clear();
    _n2smn_message.set_msgtype(type);
    _n2smn_message.set_id(_node_id);

    sendN2SMNMsg();
}

/** This method sends a simple ACK message to the SMN.
 \param type The type of the ACK message.
 \param I Integer value for MSGDATA.I
 */
void NodeBase::sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I) {
    _n2smn_message.Clear();
    _n2smn_message.set_msgtype(type);
    _n2smn_message.set_id(_node_id);
    
    auto *data = new OBNSimMsg::MSGDATA;
    data->set_i(I);
    _n2smn_message.set_allocated_data(data);

    sendN2SMNMsg();
}



/** This method runs the node in the openBuildNet simulation network.
 The node must start from state NODE_STOPPED, otherwise it will return immediately.
 A timeout in seconds can be given (default: -1). If the timeout is positive, the node will wait for new events (from the network) only up to that timeout. If a timeout occurs, the callback for timeout will be called but the simulation will not stop automatically (not even error); it's up to the callback to decide what to do with this situation, e.g. it can change the node's state to ERROR, or simply terminate the simulation.
 Be careful using the timeout as it may terminate the node unexpectedly, e.g. when the simulation goes to Debugging mode, or some other node just needs long computation time.
 \param timeout The timeout value; non-positive if there is no timeout.
 */
void NodeBase::run(double timeout) {
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
            pEvent = eventqueue_wait_and_pop();
            assert(pEvent);
            pEvent->executeMain(this);  // Execute the event
            pEvent->executePost(this);  // Post-Execute the event
        }
    } else {
        // With timeout
        while (_node_state == NODE_RUNNING || _node_state == NODE_STARTED) {
            pEvent = eventqueue_wait_and_pop(timeout);
            if (pEvent) {
                // Execute the event if not timeout
                pEvent->executeMain(this);
                pEvent->executePost(this);
            } else {
                // If timeout then we stop
                onReportInfo("[NODE] Node's execution has a timeout.");
                onRunTimeout();
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
void NodeBase::stopSimulation() {
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
void NodeBase::requestStopSimulation() {
    if (_node_state == NODE_RUNNING || _node_state == NODE_STARTED) {
        // Send the request to stop
        _n2smn_message.Clear();
        _n2smn_message.set_msgtype(OBNSimMsg::N2SMN_MSGTYPE_SYS_REQUEST_STOP);
        _n2smn_message.set_id(_node_id);

        sendN2SMNMsg();
    }
}


/** This is a very important method. It takes an SMN2N message (system message) and generates an appropriate node event.
 In the case the system message is urgent and important (e.g. an urgent shutdown, a system-wide error), it will immediately act on the event.
 \param msg Reference to the SMN2N message.
 */
void NodeBase::postEvent(const OBNSimMsg::SMN2N& msg) {
    // The cases should be ordered in the frequency of the message types
    switch (msg.msgtype()) {
        case SMN2N_MSGTYPE_SIM_Y:
            eventqueue_push(new NodeEvent_UPDATEY(msg));
            break;
            
        case SMN2N_MSGTYPE_SIM_X:
            eventqueue_push(new NodeEvent_UPDATEX(msg));
            break;
            
        case SMN2N_MSGTYPE_SIM_EVENT_ACK:
            checkWaitForCondition(msg);
            break;
            
        case SMN2N_MSGTYPE_SIM_TERM:
            // Stop the simulation (often at the end of the simulation time, or requested by the user, but not because of a system error
            eventqueue_push(new NodeEvent_TERMINATE(msg));
            break;
            
        case SMN2N_MSGTYPE_SIM_INIT:
            eventqueue_push(new NodeEvent_INITIALIZE(msg));
            break;
            
        case SMN2N_MSGTYPE_SYS_PORT_CONNECT:
            // Request from the SMN to connect ports
            eventqueue_push(new NodeEvent_PORT_CONNECT(msg));
            break;
            
        case SMN2N_MSGTYPE_SYS_REQUEST_STOP_ACK:
            // We catch this but don't do anything about it for now
            // Later we should have a waitfor condition for this
            break;
            
        default:
            onOBNWarning("Unrecognized system message from SMN with type " + std::to_string(msg.msgtype()));
            break;
    }
}


/** Handle UPDATE_X events: Main. */
void NodeBase::NodeEvent_UPDATEX::executeMain(NodeBase* pnode) {
    // Skip if the node is not RUNNING
    if (pnode->_node_state != NodeBase::NODE_RUNNING) {
        return;
    }
    
    basic_processing(pnode);
    
    // Call the callback to perform UPDATE_X
    pnode->onUpdateX(_updates);
}


/** Handle UPDATE_X events: Post. */
void NodeBase::NodeEvent_UPDATEX::executePost(NodeBase* pnode) {
    // Send ACK to the SMN
    pnode->sendACK(OBNSimMsg::N2SMN_MSGTYPE_SIM_X_ACK);
}

/** Handle UPDATE_Y events: Main. */
void NodeBase::NodeEvent_UPDATEY::executeMain(NodeBase* pnode) {
    // Skip if the node is not RUNNING; this should never happen if the node is used properly
    if (pnode->_node_state != NodeBase::NODE_RUNNING) {
        return;
    }
    
    basic_processing(pnode);
    
    // Save the current update type mask, just in case UPDATE_X needs it
    // pnode->_current_updates = _updates;
    
    // Call the callback to perform UPDATE_Y
    pnode->onUpdateY(_updates);
}

/** Handle UPDATE_Y events: Post. */
void NodeBase::NodeEvent_UPDATEY::executePost(NodeBase* pnode) {
    // Send out values from output ports which have been updated
    for (auto port: pnode->_output_ports) {
        if (port.first->isChanged()) {
            //TODO: Should change this to asynchronous send.
            port.first->sendSync();
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
void NodeBase::NodeEvent_INITIALIZE::executeMain(NodeBase* pnode) {
    // Skip if the node is not STARTED.
    // This actually should never happen if the node is used properly, because the run() method will not let the node run unless it's in a proper state.
    if (pnode->_node_state != NodeBase::NODE_STARTED) {
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
void NodeBase::NodeEvent_INITIALIZE::executePost(NodeBase* pnode) {
    // Send out values from output ports if they have been set / updated
    for (auto port: pnode->_output_ports) {
        //TODO: Should change this to asynchronous send.
        if (port.first->isChanged()) {
            port.first->sendSync();
            if (pnode->hasError()) {
                break;
            }
        }
    }
    
    // Set the node's state to RUNNING if it's still STARTED (if it's ERROR, we won't change it)
    if (pnode->_node_state == NodeBase::NODE_STARTED) {
        pnode->_node_state = NodeBase::NODE_RUNNING;
    }
    
    // Send an ACK message to the SMN
    if (pnode->_node_state == NodeBase::NODE_RUNNING) {
        // Successful
        pnode->sendACK(OBNSimMsg::N2SMN::MSGTYPE::N2SMN_MSGTYPE_SIM_INIT_ACK);
    } else {
        // Unsuccessful => ACK with data.I > 0
        pnode->sendACK(OBNSimMsg::N2SMN::MSGTYPE::N2SMN_MSGTYPE_SIM_INIT_ACK, 1);
    }
}

/** Handle Termination: Main. */
void NodeBase::NodeEvent_TERMINATE::executeMain(NodeBase* pnode) {
    // Skip if the node is not RUNNING
    if (pnode->_node_state != NodeBase::NODE_RUNNING) {
        return;
    }
    
    basic_processing(pnode);
    
    // Set the node's state to STOPPED
    // We change the node's state before calling the callback onTermination() because this system message means that simulation is definitely going to terminate immediately.
    if (pnode->_node_state == NodeBase::NODE_RUNNING) {
        pnode->_node_state = NodeBase::NODE_STOPPED;
    }
    
    pnode->onTermination();
}

/** Handle Termination: Post. */
void NodeBase::NodeEvent_TERMINATE::executePost(NodeBase* pnode) {
    // No ACK is needed
}

/** Handle port error */
void NodeBase::NodeEventException::executeMain(NodeBase* pnode) {
    // rethrow the exception and catch it to call the appropriate error callback function
    try {
        std::rethrow_exception(m_exception);
    } catch (OBNnode::inputport_error& e) {
        switch (e.m_errortype) {
            case OBNnode::inputport_error::ERR_RAWMSG:
                pnode->onRawMessageError(e.m_port, e.what());
                break;
                
            case OBNnode::inputport_error::ERR_READVALUE:
                pnode->onReadValueError(e.m_port, e.what());
                break;
                
            default:
                break;
        }
    } catch (OBNnode::outputport_error& e) {
        switch (e.m_errortype) {
            case OBNnode::outputport_error::ERR_SENDMSG:
                pnode->onSendMessageError(e.m_port, e.what());
                break;
                
            default:
                break;
        }
    }
    
    // Any unhandled exception will go through and trigger the default handler
}

/** Handle port connection request. */
void NodeBase::NodeEvent_PORT_CONNECT::executeMain(NodeBase* pnode) {
    std::pair<int, std::string> result{_valid_msg?-1:-3, ""};
    
    if (_valid_msg) {
        // Find the port on this node
        PortBase* myport = nullptr;
        for (const auto& p: pnode->_input_ports) {
            if (p.first->getPortName() == _myport) {
                myport = p.first;
                break;
            }
        }
        
        // Do not find the port in the list of physical output ports
        
        if (myport) {
            // Found --> ask the port to connect
            result = myport->connect_from_port(_otherport);
        }
    }
    
    // Prepare the ACK message
    pnode->_n2smn_message.Clear();
    pnode->_n2smn_message.set_msgtype(OBNSimMsg::N2SMN_MSGTYPE_SYS_PORT_CONNECT_ACK);
    if (_hasID) {
        pnode->_n2smn_message.set_id(_id);
    }

    // Only need Data field if not successful
    if (result.first != 0) {
        OBNSimMsg::MSGDATA* pData = new OBNSimMsg::MSGDATA();
        pData->set_i(result.first);
        if (!result.second.empty()) {
            pData->set_b(result.second);
        }
        pnode->_n2smn_message.set_allocated_data(pData);
    }
    
    pnode->sendN2SMNMsg();
}