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
using namespace yarp::os;
using namespace OBNnode;
using namespace OBNSimMsg;


// The main network object
Network _the_yarp_network;

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


YarpNode::YarpNode(const string _name) {
    _nodeName = trim(_name);
    assert(!_nodeName.empty());
    
    // Initialize the node (setting its internal parameters)
    _node_id = 0;
    _node_state = NODE_RUNNING;
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

/** This method runs the node in the openBuildNet simulation network.
 The node must start from state NODE_STOPPED, otherwise it will return immediately.
 A timeout in seconds can be given (default: -1). If the timeout is positive, the node will wait for new events (from the network) only up to that timeout. If a timeout occurs, the callback for timeout will be called and the simulation will stopped.
 Be careful using the timeout as it may terminate the node unexpectedly, e.g. when the simulation goes to Debugging mode, or some other node just needs long computation time.
 \param timeout The timeout value; non-positive if there is no timeout.
 */
void YarpNode::run(double timeout) {
    if (_node_state != NODE_STOPPED) {
        return;
    }
    
    // Looping to process events until the simulation stops or a timeout occurs
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