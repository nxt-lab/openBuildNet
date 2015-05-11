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
yarp::os::Network YarpNode::yarp_network;

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
void YarpNode::SMNPort::onRead(SMNMsg& b) {
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

bool YarpNode::openSMNPort() {
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

bool YarpNode::attachAndOpenPort(YarpPortBase * port) {
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
bool YarpNode::addInput(YarpPortBase* port) {
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
bool YarpNode::addOutput(YarpOutputPortBase* port) {
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
 \param name The required name of the node; must be a valid identifier.
 \param ws The optional workspace; must be a valid identifier. The workspace name is prepended to all names used by this node, including the SMN. Default workspace is "" (i.e. no workspace).
 */
YarpNode::YarpNode(const string& _name, const string& ws): _smn_port(this) {
    _nodeName = trim(_name);
    assert(!_nodeName.empty());
    
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

YarpNode::~YarpNode() {
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
void YarpNode::removePort(YarpPortBase* port) {
    assert(port);

    _input_ports.remove(port);
    //_data_ports.remove(port);
}

/** This method tries to remove a port from this node.
 The port object is an output port.
 The detach() method of the port won't be called.
 \param port Pointer to the port object (must be valid).
 */
void YarpNode::removePort(YarpOutputPortBase* port) {
    assert(port);

    _output_ports.remove(port);
}

//unsigned int YarpNode::createPort(const string& name, bool bCallback, bool strict) {
//    auto sName = trim(name);
//    if (sName.empty()) {
//        reportError("YarpNode:invalid_port_name", "Invalid port's name.");
//    }
//    
//    // Check if the name already existed
//    if (portsByName.find(sName) != portsByName.end()) {
//        reportError("YarpNode:invalid_port_name", "Port's name already existed.");
//    }
//    
//    unsigned int id = nextPortID++;
//    portsByName.emplace(sName, ports.emplace(id, new YPort(this, sName, id, bCallback, strict)).first->second);
//    
//    return id;
//}
//


bool YarpNode::connectWithSMN(const char *carrier) {
    if (!openSMNPort()) { return false; }
    if (carrier) {
        return yarp_network.connect(fullPortName("_gc_"), _workspace + "_smn_/_gc_", carrier) &&   // Connect from node to SMN
        yarp_network.connect(_workspace + "_smn_/" + _nodeName, fullPortName("_gc_"), carrier);    // Connect from SMN to node
    } else {
        return yarp_network.connect(fullPortName("_gc_"), _workspace + "_smn_/_gc_") &&   // Connect from node to SMN
        yarp_network.connect(_workspace + "_smn_/" + _nodeName, fullPortName("_gc_"));    // Connect from SMN to node
    }
}


/** This method sends a simple ACK message to the SMN.
 \param type The type of the ACK message.
 */
void YarpNode::sendACK(OBNSimMsg::N2SMN::MSGTYPE type) {
    YarpNode::SMNMsg& msg = _smn_port.prepare();
    
    _n2smn_message.set_msgtype(type);
    _n2smn_message.set_id(_node_id);
    
    msg.setMessage(_n2smn_message);
    _smn_port.writeStrict();
}

/** This method sends a simple ACK message to the SMN.
 \param type The type of the ACK message.
 \param I Integer value for MSGDATA.I
 */
void YarpNode::sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I) {
    YarpNode::SMNMsg& msg = _smn_port.prepare();
    
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
void YarpNode::sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I, int64_t IX) {
    YarpNode::SMNMsg& msg = _smn_port.prepare();
    
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
 A timeout in seconds can be given (default: -1). If the timeout is positive, the node will wait for new events (from the network) only up to that timeout. If a timeout occurs, the callback for timeout will be called and the simulation will stopped.
 Be careful using the timeout as it may terminate the node unexpectedly, e.g. when the simulation goes to Debugging mode, or some other node just needs long computation time.
 \param timeout The timeout value; non-positive if there is no timeout.
 */
void YarpNode::run(double timeout) {
    // Make sure that the SMN port is opened (but won't connect it)
    if (!openSMNPort()) {
        // Error
        onReportInfo("[NODE] Could not open the SMN port. Check the network and the name server.");
        _node_state = NODE_ERROR;
        return;
    }
    
    // Must start from the STOPPED state
    if (_node_state != NODE_STOPPED) {
        onReportInfo("[NODE] Node can't be started from a non-STOPPED state. Make sure the node is stopped before starting it.");
        return;
    }

    _current_sim_time = 0;
    _node_state = NODE_STARTED;     // Node has started, but not yet initialized
    
    // Looping to process events until the simulation stops or a timeout occurs
    std::shared_ptr<NodeEvent> pEvent;
    if (timeout <= 0.0) {
        // Without timeout
        while (_node_state == NODE_RUNNING || _node_state == NODE_STARTED) {
            pEvent = _event_queue.wait_and_pop();
            assert(pEvent);
            pEvent->execute(this);  // Execute the event
        }
    } else {
        // With timeout
        while (_node_state == NODE_RUNNING || _node_state == NODE_STARTED) {
            pEvent = _event_queue.wait_and_pop_timeout(timeout);
            assert(pEvent);
            pEvent->execute(this);  // Execute the event
        }
    }
    
    // This is the end of the simulation
    onReportInfo("[NODE] Node's execution has stopped.");
}


/** This is a very important method. It takes an SMN2N message (system message) and generates an appropriate node event.
In the case the system message is urgent and important (e.g. an urgent shutdown, a system-wide error), it will immediately act on the event.
 \param msg Reference to the SMN2N message.
*/
void YarpNode::postEvent(const OBNSimMsg::SMN2N& msg) {
    // The cases should be ordered in the frequency of the message types
    switch (msg.msgtype()) {
        case SMN2N_MSGTYPE_SIM_Y:
            _event_queue.push(new NodeEvent_UPDATEY(msg));
            break;
            
        case SMN2N_MSGTYPE_SIM_X:
            _event_queue.push(new NodeEvent_UPDATEX(msg));
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


/** Handle UPDATE_X events. */
void YarpNode::NodeEvent_UPDATEX::execute(YarpNode* pnode) {
    // Skip if the node is not RUNNING
    if (pnode->_node_state != YarpNode::NODE_RUNNING) {
        return;
    }

    basic_processing(pnode);

    // Call the callback to perform UPDATE_X
    pnode->onUpdateX();
    
    // Send ACK to the SMN
    pnode->sendACK(OBNSimMsg::N2SMN_MSGTYPE_SIM_X_ACK);
}

/** Handle UPDATE_Y events. */
void YarpNode::NodeEvent_UPDATEY::execute(YarpNode* pnode) {
    // Skip if the node is not RUNNING; this should never happen if the node is used properly
    if (pnode->_node_state != YarpNode::NODE_RUNNING) {
        return;
    }
    
    basic_processing(pnode);
    
    // Save the current update type mask, just in case UPDATE_X needs it
    pnode->_current_updates = _updates;
    
    // Call the callback to perform UPDATE_Y
    pnode->onUpdateY(_updates);
    
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

/** Handle Initialization before simulation. */
void YarpNode::NodeEvent_INITIALIZE::execute(YarpNode* pnode) {
    // Skip if the node is not STARTED.
    // This actually should never happen if the node is used properly, because the run() method will not let the node run unless it's in a proper state.
    if (pnode->_node_state != YarpNode::NODE_STARTED) {
        return;
    }
    
    basic_processing(pnode);
    
    pnode->onInitialization();
    
    // Send out values from output ports, whether updated or not
    for (auto port: pnode->_output_ports) {
        //TODO: Should change this to asynchronous send.
        port->sendSync();
        if (pnode->hasError()) {
            break;
        }
    }
    
    // Set the node's state to RUNNING if it's still STARTED (if it's ERROR, we won't change it)
    if (pnode->_node_state == YarpNode::NODE_STARTED) {
        pnode->_node_state = YarpNode::NODE_RUNNING;
    }
    
    // Send an ACK message to the SMN
    if (pnode->_node_state == YarpNode::NODE_RUNNING) {
        // Successful
        pnode->sendACK(OBNSimMsg::N2SMN::MSGTYPE::N2SMN_MSGTYPE_SIM_INIT_ACK);
    } else {
        // Unsuccessful => ACK with data.I > 0
        pnode->sendACK(OBNSimMsg::N2SMN::MSGTYPE::N2SMN_MSGTYPE_SIM_INIT_ACK, 1);
    }
}

void YarpNode::NodeEvent_TERMINATE::execute(YarpNode* pnode) {
    // Skip if the node is not RUNNING
    if (pnode->_node_state != YarpNode::NODE_RUNNING) {
        return;
    }

    basic_processing(pnode);
    
    pnode->onTermination();
    
    // Set the node's state to STOPPED
    if (pnode->_node_state == YarpNode::NODE_RUNNING) {
        pnode->_node_state = YarpNode::NODE_STOPPED;
    }
    
    // No ACK is needed
}
