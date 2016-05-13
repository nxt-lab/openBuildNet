/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnsim_extmqtt.cpp
 * \brief Generic external interface for MQTTNode of the openBuildNet simulation framework.
 *
 * This file is typically used by external interfaces in other languages (e.g., Matlab, Python).
 * Requires MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>       // cout
#include <vector>
#include <algorithm>    // std::copy
#include <functional>
#include <utility>      // std::pair


#include <obnnode.h>    // node.C++ framework
#include <obnnode_extmqtt.h>
#include <obnnode_ext_stdmsg.h>

using namespace OBNnode;
using std::string;

/* This is required so we can manage the node instances.
 All implementations must have this line.
 */
template class OBNNodeExtInt::Session<MQTTNodeExt>;


/******* Implementation of the MQTT Node for External Interface *******/

#define YNM_PORT_CLASS_BY_NAME(BASE,PCLS,CTNR,TYPE,...) \
  TYPE==OBNEI_Element_double?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<double> >(__VA_ARGS__)):(\
    TYPE==OBNEI_Element_logical?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<bool> >(__VA_ARGS__)):(\
      TYPE==OBNEI_Element_int32?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int32_t> >(__VA_ARGS__)):(\
        TYPE==OBNEI_Element_int64?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int64_t> >(__VA_ARGS__)):(\
          TYPE==OBNEI_Element_uint32?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint32_t> >(__VA_ARGS__)):(\
            TYPE==OBNEI_Element_uint64?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint64_t> >(__VA_ARGS__)):nullptr)))));

#define YNM_PORT_CLASS_BY_NAME_STRICT(BASE,PCLS,CTNR,TYPE,STRICT,...) \
  TYPE==OBNEI_Element_double?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<double>,STRICT >(__VA_ARGS__)):(\
    TYPE==OBNEI_Element_logical?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<bool>,STRICT >(__VA_ARGS__)):(\
      TYPE==OBNEI_Element_int32?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int32_t>,STRICT >(__VA_ARGS__)):(\
        TYPE==OBNEI_Element_int64?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int64_t>,STRICT >(__VA_ARGS__)):(\
          TYPE==OBNEI_Element_uint32?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint32_t>,STRICT >(__VA_ARGS__)):(\
            TYPE==OBNEI_Element_uint64?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint64_t>,STRICT >(__VA_ARGS__)):nullptr)))));



/** This is a meta-function for creating all kinds of input ports supported by this class.
 It creates an input port with the specified type, name and configuration, adds it to the node object, then open it.
 \param name A valid name of the port in this node.
 \param format specifies the format type, e.g., ProtoBuf or JSON.
 \param container specifies the container type.
 \param element specifies the data type of the elements of the container type (except for binary type).
 \param strict Whether the input port uses strict reading.
 \return The unique ID of the port in this node (which must be non-negative), or a negative integer if there was an error.
 */
int MQTTNodeExt::createInputPort(const std::string &name,
                                 OBNEI_FormatType format,
                                 OBNEI_ContainerType container,
                                 OBNEI_ElementType element,
                                 bool strict)
{
    // Currently we only support ProtoBuf
    if (format != OBNEI_Format_ProtoBuf) {
        return -1000;       // Unsupported format
    }
    
    MQTTNodeExt::PortInfo portinfo;
    OBNnode::InputPortBase *port;
    portinfo.type = OBNEI_Port_Input;
    switch (container) {
        case OBNEI_Container_Scalar:
            if (strict) {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_scalar,element,true,name);
            } else {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_scalar,element,false,name);
            }
            break;

        case OBNEI_Container_Vector:
            if (strict) {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_vector_raw,element,true,name);
            } else {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_vector_raw,element,false,name);
            }
            break;
            
        case OBNEI_Container_Matrix:
            if (strict) {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_matrix_raw,element,true,name);
            } else {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_matrix_raw,element,false,name);
            }
            break;
            
        case OBNEI_Container_Binary:
            if (strict) {
                port = new MQTTInput<OBN_BIN,bool,true>(name);
            } else {
                port = new MQTTInput<OBN_BIN,bool,false>(name);
            }
            break;
    }
    
    if (!port) {
        // port is nullptr => error
        return -1;  // Error while creating port
    }
    
    // Add the port to the node, which will OWN the port and will delete it
    bool result = addInput(port, true);
    if (!result) {
        // failed to add to node
        delete port;
        return -2;  // Could not add new port to node.
    }
    
    int id = _all_ports.size();
    portinfo.port = port;
    portinfo.container = container;
    portinfo.elementType = element;
    portinfo.strict = strict;
    _all_ports.push_back(portinfo);
    return id;
}

/** \brief Meta-function for creating all kinds of output ports supported by this class. */
int MQTTNodeExt::createOutputPort(const std::string &name,
                                  OBNEI_FormatType format,
                                  OBNEI_ContainerType container,
                                  OBNEI_ElementType element)
{
    // Currently we only support ProtoBuf
    if (format != OBNEI_Format_ProtoBuf) {
        return -1000;       // Unsupported format
    }

    MQTTOutputPortBase *port;
    MQTTNodeExt::PortInfo portinfo;
    portinfo.type = OBNEI_Port_Output;
    switch (container) {
        case OBNEI_Container_Scalar:
            port = YNM_PORT_CLASS_BY_NAME(MQTTOutputPortBase,MQTTOutput,obn_scalar,element,name);
            break;
            
        case OBNEI_Container_Vector:
            port = YNM_PORT_CLASS_BY_NAME(MQTTOutputPortBase,MQTTOutput,obn_vector_raw,element,name);
            break;
            
        case OBNEI_Container_Matrix:
            port = YNM_PORT_CLASS_BY_NAME(MQTTOutputPortBase,MQTTOutput,obn_matrix_raw,element,name);
            break;
            
        case OBNEI_Container_Binary:
            port = new MQTTOutput<OBN_BIN,bool>(name);
            break;
    }
    
    if (!port) {
        // port is nullptr => error
        return -1;      // Error while creating port
    }
    
    // Add the port to the node, which will OWN the port and will delete it
    bool result = addOutput(port, true);
    if (!result) {
        // failed to add to node
        delete port;
        return -2;      // Could not add new port to node.
    }
    
    auto id = _all_ports.size();
    portinfo.port = port;
    portinfo.container = container;
    portinfo.elementType = element;
    _all_ports.push_back(portinfo);
    return id;
}


MQTTNodeExt::~MQTTNodeExt() {
    // If the node is still running, we need to stop it first
    stopSimulation();
    
    // We need to delete all port objects belonging to this node (in _all_ports vector) because if we don't, they will be deleted in ~NodeBase() when the MQTTClient object (which belongs to the child class MQTTNodeBase) is already deleted --> access to the MQTT client will cause an error.
    for (auto p:_all_ports) {
        delete p.port;
    }
}


/** This method runs the simulation until the next event that requires a callback (e.g. an UPDATE_Y message from the GC), or until the node stops (because the simulation terminates, because of timeout or errors...).
 \param timeout Timeout value in seconds; if there is no new message from the SMN after this timeout, the function will return immediately; if timeout <= 0.0, there is no timeout and the method can run indefinitely (maybe undesirable).
 \return 0 if everything is going well and there is an event pending, 1 if timeout (but the simulation won't stop automatically, it's still running), 2 if the simulation has stopped (properly, not because of an error), 3 if the simulation has stopped due to an error (the node's state becomes NODE_ERROR)
 */
int MQTTNodeExt::runStep(double timeout) {
    if (_node_state == NODE_ERROR) {
        // We can't continue in error state
        return 3;
    }

    // If there is a pending event, which means the current event has just been processed in Matlab, we must resume the execution of the event object to finish it, then we can continue
    if (_ml_pending_event) {
        if (_current_node_event) {
            // If there is a valid event object, call the post execution method
            _current_node_event->executePost(this);
        }
        _ml_pending_event = false;      // Clear the current event
    }

    // Run until there is a pending ML event
    while (!_ml_pending_event) {
        NODE_STATE state = _node_state;
        switch (state) {
            case NODE_RUNNING:  // Node is running normally, so keep running it until the next event
            case NODE_STARTED:
                // Check if there is any port event -> process them immediately -> they have higher priority
                if (!m_port_events.empty()) {
                    // Pop it out
                    auto evt = m_port_events.wait_and_pop();
                    assert(evt);    // The event must be valid
                    
                    // Create an external-interface event for this port event
                    _ml_current_event.type = OBNEI_Event_RCV;   // The default event is Message Received; check evt->event_type for other types
                    _ml_current_event.arg.index = evt->port_index;
                    _ml_pending_event = true;
                    _current_node_event.reset();    // The node event pointer is set to null
                    
                    break;
                }
                
                // Else (if there is no pending port event, get and process node events
                // Wait for the next event and process it
                if (timeout <= 0.0) {
                    // Without timeout
                    _current_node_event = _event_queue.wait_and_pop();
                    assert(_current_node_event);
                    _current_node_event->executeMain(this);  // Execute the event
                    
                    // if there is no pending event, we must finish the event's execution now
                    // if there is, we don't need to because it will be finished later when this function is resumed
                    if (!_ml_pending_event) {
                        _current_node_event->executePost(this);
                    }
                } else {
                    // With timeout
                    _current_node_event = _event_queue.wait_and_pop_timeout(timeout);
                    if (_current_node_event) {
                        _current_node_event->executeMain(this);  // Execute the event if not timeout
                        
                        // if there is no pending event, we must finish the event's execution now
                        // if there is, we don't need to because it will be finished later when this function is resumed
                        if (!_ml_pending_event) {
                            _current_node_event->executePost(this);
                        }
                    } else {
                        // If timeout then we return but won't stop
                        onRunTimeout();
                        return 1;
                    }
                }
                
                break;
                
            case NODE_STOPPED:
                // if _node_is_stopping = true then the node is stopping (we've just pushed the TERM event to external interface, now we need to actually stop it)
                if (_node_is_stopping) {
                    _node_is_stopping = false;
                    return 2;
                }
                // Otherwise, start the simulation from beginning
                // Make sure that the SMN port is opened (but won't connect it)
                if (!openSMNPort()) {
                    // Error
                    _node_state = NODE_ERROR;
                    reportError("Could not open the SMN port; check the network and the name server.");
                    break;
                }
                
                // Initialize the node's state
                initializeForSimulation();
                
                // Switch to STARTED to wait for INIT message from the SMN
                _node_state = NODE_STARTED;
                break;
                
            case NODE_ERROR:
                // Error state
                reportError("Internal error: node's state is ERROR.");
                break;
        }
        
        // At this point, if node state is not RUNNING or STARTED, we must stop it
        if (_node_state != NODE_RUNNING && _node_state != NODE_STARTED) {
            break;
        }
    }
    
    // Return appropriate value depending on the current state
    if (_node_state == NODE_STOPPED && !_node_is_stopping) {
        // Stopped (properly) but not when the node is stopping (we still need to push a TERM event to external interface)
        return 2;
    } else if (_node_state == NODE_ERROR) {
        // error
        return 3;
    }
    
    // this can only be reached if there is a pending event
    return 0;
}

/** This method waits and gets the next port event.
 \param timeout Timeout value in seconds to wait for the next event. If timeout <= 0.0, the function will return immediately.
 \return A unique_ptr to the event object (of type PortEvent); the pointer is null if there was no pending event when the timeout was up.
 */
std::unique_ptr<MQTTNodeExt::PortEvent> MQTTNodeExt::getNextPortEvent(double timeout) {
    if (timeout <= 0.0) {
        return m_port_events.try_pop();
    }
    return m_port_events.wait_and_pop_timeout(timeout);
}

/* ============ The external interface ===============*/

// Create a new node
EXPORT
int createOBNNode(const char* name, const char* workspace, const char* addr, size_t* id) {
    if (!name) {
        return -1;      // Empty name
    }
    
    // Check if the node has already existed
    {
        std::string newFullName( (workspace)?(std::string(workspace) + '/' + name):(name) );
        if (OBNNodeExtInt::Session<MQTTNodeExt>::exist([&newFullName](const MQTTNodeExt& node){
            return node.full_name() == newFullName;
        })) {
            reportError("Node already exists.");
            return 1;       // Node exists
        }
    }
    
    // Create a node
    MQTTNodeExt *y;
    try {
        if (workspace) {
            // Workspace is given
            y = new MQTTNodeExt(std::string(name), std::string(workspace));
        } else {
            // No workspace is given
            y = new MQTTNodeExt(std::string(name));
        }
    } catch (...) {
        reportError("Error while creating the node.");
        return -2;      // Error while creating the node object
    }
    
    // Set the server
    if (addr) {
        std::string s(addr);
        if (!s.empty()) {
            y->setServerAddress(addr);
        }
    }
    
    // For MQTT, we start the client immediately
    if (y->startMQTT()) {
        *id = OBNNodeExtInt::Session<MQTTNodeExt>::create(y);
        return 0;
    } else {
        // Delete the node object and report error
        delete y;
        reportError("Error while starting the node.");
        return -3;      // Error while starting the node object
    }
}


// Delete a node object
EXPORT
int deleteOBNNode(size_t id) {
    return (OBNNodeExtInt::Session<MQTTNodeExt>::destroy(id))?0:-1;
}


// Request/notify the SMN to stop, then terminate the node's simulation
EXPORT
int nodeStopSimulation(size_t nodeid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    pnode->stopSimulation();
    return 0;
}


// Requests the SMN/GC to stop the simulation (by sending a request message to the SMN) but does not terminate the node.
EXPORT
int nodeRequestStopSimulation(size_t nodeid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    pnode->requestStopSimulation();
    return 0;
}


// Check if node has stopped
EXPORT
int nodeIsStopped(size_t nodeid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    return (pnode->nodeState() == OBNnode::MQTTNode::NODE_STOPPED)?1:0;
}

// Check if node is in error state
EXPORT
int nodeIsError(size_t nodeid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    return pnode->hasError()?1:0;
}


// Check if the current state of the node is RUNNING
EXPORT
int nodeIsRunning(size_t nodeid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    return (pnode->nodeState() == OBNnode::MQTTNode::NODE_RUNNING)?1:0;
}


// Get the next port event (e.g. message received) with a possible timeout.
EXPORT
int simGetPortEvent(size_t nodeid, double timeout, unsigned int* event_type, size_t* portid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // get next PortEvent from node
    auto evt = pnode->getNextPortEvent(timeout);
    
    if (!evt) {
        // No event
        return 1;
    }
    
    switch (evt->event_type) {
        case MQTTNodeExt::PortEvent::RCV:
            *event_type = OBNEI_Event_RCV;
            break;
    }
    
    *portid = evt->port_index;
    return 0;
}


// Runs the node's simulation until the next event, or until the node stops or has errors.
EXPORT
int simRunStep(size_t nodeid, double timeout, unsigned int* event_type, OBNEI_EventArg* event_args) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Call node's runStep
    int result = pnode->runStep(timeout);
    
    // Handle the special cases
    if (result == 3) {
        reportError("Node is in error state; can't continue simulation; please stop the node to clear the error state before continuing.");
        return result;
    }
    
    if (pnode->_ml_pending_event) {
        *event_type = pnode->_ml_current_event.type;
        *event_args = pnode->_ml_current_event.arg;
    }
    
    return result;
}


// Returns the current simulation time of the node with a desired time unit.
// 0 = second, -1 = millisecond, -2 = microsecond, 1 = minute, 2 = hour
EXPORT
int nodeSimulationTime(size_t nodeid, int timeunit, double* T) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    switch (timeunit) {
        case 0:
            *T = pnode->currentSimulationTime<std::chrono::seconds>();
            break;
            
        case 1:
            *T = pnode->currentSimulationTime<std::chrono::minutes>();
            break;
            
        case 2:
            *T = pnode->currentSimulationTime<std::chrono::hours>();
            break;
            
        case -1:
            *T = pnode->currentSimulationTime<std::chrono::milliseconds>();
            break;
            
        case -2:
            *T = pnode->currentSimulationTime<std::chrono::microseconds>();
            break;
            
        default:
            reportError("Invalid time unit.");
            return -2;  // Invalid time unit
    }
    
    return 0;
}


EXPORT
int nodeWallClockTime(size_t nodeid, long* T) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    *T = static_cast<long>(pnode->currentWallClockTime());
    
    return 0;
}


EXPORT
int simRequestFutureUpdate(size_t nodeid, OBNSimTimeType t, OBNUpdateMask mask, double timeout) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1001;
    }
    
    auto *pCond = pnode->requestFutureUpdate(t, mask, false);
    if (pCond) {
        return pnode->resultFutureUpdate(pCond, timeout);
    } else {
        return -2;
    }
}



// Create a new input port on a node
// Arguments: node ID, port's name, format type, container type, element type, strict or not
// Returns port's id; or negative number if error.
// id is an integer starting from 0.
EXPORT
int createInputPort(size_t id,
                    const char* name,
                    OBNEI_FormatType format,
                    OBNEI_ContainerType container,
                    OBNEI_ElementType element,
                    bool strict)
{
    if (!name) {
        return -1;      // Port name not provided.
    }
    std::string portname(name);
    if (portname.empty()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_NAME);
        return -2;      // Invalid port name.
    }
    
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(id);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -3;      // Node doesn't exist
    }
    
    // Try to add the port
    return pnode->createInputPort(portname, format, container, element, strict);
}

// Create a new output port on a node
// Arguments: node ID, port's name, format type, container type, element type
// Returns port's id; or negative number if error.
// id is an integer starting from 0.
EXPORT
int createOutputPort(size_t id,
                     const char* name,
                     OBNEI_FormatType format,
                     OBNEI_ContainerType container,
                     OBNEI_ElementType element)
{
    if (!name) {
        return -1;      // Port name not provided.
    }
    std::string portname(name);
    if (portname.empty()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_NAME);
        return -2;      // Invalid port name.
    }
    
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(id);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -3;      // Node doesn't exist
    }
    
    // Try to add the port
    return pnode->createOutputPort(portname, format, container, element);
}

// Synchronous sending: request an output port to send its current value/message immediately and wait until it can be sent.
// Note that this function does not accept a value to be sent; instead the value/message of the port is set by another function.
// Args: node ID, port's ID
// Returns: zero if successful
// This function will return an error if the given port is not a physical output port.
EXPORT
int outputSendSync(size_t nodeid, size_t portid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    if (portinfo.type != OBNEI_Port_Output) {
        reportError("Given port is not an output.");
        return -3;
    }
    
    // Cast the pointer to an output port object and send
    MQTTOutputPortBase *p = dynamic_cast<MQTTOutputPortBase*>(portinfo.port);
    if (p) {
        p->sendSync();
    } else {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    return 0;
}

// Is there a value pending at an input port?
// Args: node ID, port's ID
// Returns: true if >0, false if =0, error if <0.
EXPORT
int inputPending(size_t nodeid, size_t portid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    if (portinfo.type != OBNEI_Port_Input) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_INPUT);
        return -3;
    }
    
    // Cast and query the port
    OBNnode::InputPortBase *port = dynamic_cast<OBNnode::InputPortBase*>(portinfo.port);
    if (port) {
        return port->isValuePending()?1:0;
    } else {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
}

// Returns information about a port.
// Arguments: node ID, port's ID, pointer to a valid OBNEI_PortInfo structure to receive info
// Returns: 0 if successful.
EXPORT
int portInfo(size_t nodeid, size_t portid, OBNEI_PortInfo* pInfo) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    // Return the information
    pInfo->type = portinfo.type;
    pInfo->container = portinfo.container;
    pInfo->element_type = portinfo.elementType;
    pInfo->strict = portinfo.strict;
    
    return 0;
}


// Generic (template) function to read from scalar input port
template <typename T>
int READ_INPUT_SCALAR_HELPER(size_t nodeid, size_t portid, T* pval) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];

    if (portinfo.type != OBNEI_Port_Input) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_INPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Scalar) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }

    // Query its value based on its type
    if (portinfo.strict) {
        MQTTInput<OBN_PB,obn_scalar<T>,true> *p = dynamic_cast<MQTTInput<OBN_PB,obn_scalar<T>,true>*>(portinfo.port);
        
        if (p) {
            if (p->isValuePending()) {
                *pval = p->pop();
                return 0;
            } else {
                // No value
                return 1;
            }
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
    } else {
        MQTTInput<OBN_PB,obn_scalar<T>,false> *p = dynamic_cast<MQTTInput<OBN_PB,obn_scalar<T>,false>*>(portinfo.port);
        if (p) {
            *pval = p->get();
            return 0;
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
    }
}

// Float64
EXPORT
int inputScalarDoubleGet(size_t nodeid, size_t portid, double* pval) {
    return READ_INPUT_SCALAR_HELPER(nodeid, portid, pval);
}

// C++ bool (1 byte)
EXPORT
int inputScalarBoolGet(size_t nodeid, size_t portid, bool* pval) {
    return READ_INPUT_SCALAR_HELPER(nodeid, portid, pval);
}

// Int32
EXPORT
int inputScalarInt32Get(size_t nodeid, size_t portid, int32_t* pval) {
    return READ_INPUT_SCALAR_HELPER(nodeid, portid, pval);
}

// Int64
EXPORT
int inputScalarInt64Get(size_t nodeid, size_t portid, int64_t* pval) {
    return READ_INPUT_SCALAR_HELPER(nodeid, portid, pval);
}

// UInt32
EXPORT
int inputScalarUInt32Get(size_t nodeid, size_t portid, uint32_t* pval) {
    return READ_INPUT_SCALAR_HELPER(nodeid, portid, pval);
}

// UInt64
EXPORT
int inputScalarUInt64Get(size_t nodeid, size_t portid, uint64_t* pval) {
    return READ_INPUT_SCALAR_HELPER(nodeid, portid, pval);
}


/** The wrapper for the access management object for vector/matrix input port,
 which is a pair of bool (strict or non-strict port) and a void* pointer of the actual access object. */
typedef std::pair<bool, void*> AccessManagementWrapper;


// Generic (template) function to read from vector input port - the *GET function
template <typename ETYPE>
int read_input_vector_get(size_t nodeid, size_t portid, void** pMan, const ETYPE** pVals, size_t* nelems)
{
    // Sanity check
    if (pMan == nullptr || nelems == nullptr) {
        return -1000;
    }
    
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    if (portinfo.type != OBNEI_Port_Input) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_INPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Vector) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    // Query its value based on its type
    if (portinfo.strict) {
        MQTTInput<OBN_PB,obn_vector_raw<ETYPE>,true> *p = dynamic_cast<MQTTInput<OBN_PB,obn_vector_raw<ETYPE>,true>*>(portinfo.port);
        
        if (p) {
            if (p->isValuePending()) {
                // Get unique_ptr to an array container.
                auto pv = p->pop();
                
                // pMan contains this array container object, taken from the unique_ptr
                raw_array_container<ETYPE>* pContainer = pv.release();
                
                if (!pContainer) {
                    reportError(OBNNodeExtInt::StdMsgs::INTERNAL_INVALID_VALUE_FROM_PORT);
                    return -5;
                }

                // Returns the pointer pMan which wraps pContainer
                void* pContainerVoid = static_cast<void*>(pContainer);
                *pMan = static_cast<void*>(new AccessManagementWrapper(portinfo.strict, pContainerVoid));
                
                // Lock the pointers so that it will remain in memory
                OBNNodeExtInt::lockPointer(pContainerVoid);
                OBNNodeExtInt::lockPointer(*pMan);

                // Number of elements
                *nelems = pContainer->size();
                
                // Returns the array if requested
                if (pVals) {
                    *pVals = pContainer->data();
                }
                
                return 0;
            } else {
                // No value
                return 1;
            }
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
        
    } else {
        using ThisPortType = MQTTInput<OBN_PB,obn_vector_raw<ETYPE>,false>;
        ThisPortType* p = dynamic_cast<ThisPortType*>(portinfo.port);
        
        if (p) {
            // Get direct access to values via a LockedAccess object
            // Create a new dynamic LockedAccess object, which will be wrapped in pMan
            typename ThisPortType::LockedAccess* locked_access = new typename ThisPortType::LockedAccess(p->lock_and_get());
            
            // Returns the pointer pMan which wraps locked_access
            void* locked_access_void = static_cast<void*>(locked_access);
            *pMan = static_cast<void*>(new AccessManagementWrapper(portinfo.strict, locked_access_void));
            
            // Lock the pointer so that it will remain in memory
            OBNNodeExtInt::lockPointer(locked_access_void);
            OBNNodeExtInt::lockPointer(*pMan);

            // Number of elements
            *nelems = (*locked_access)->size();
            
            // Returns the array if requested
            if (pVals) {
                *pVals = (*locked_access)->data();
            }
            
            return 0;
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
    }
}

// Generic (template) function to copy/release the management object for reading from a vector input port - the *RELEASE function
template <typename ETYPE>
void read_input_vector_copy_release(void* pMan, ETYPE* pBuf) {
    if (!pMan) {
        return;
    }
    
    // Cast it back to the wrapper type
    AccessManagementWrapper* wrapper = static_cast<AccessManagementWrapper*>(pMan);
    
    if (wrapper->first) {
        // Strict port
        using AccessObjectType = raw_array_container<ETYPE>;
        AccessObjectType* access_obj = static_cast<AccessObjectType*>(wrapper->second);
        
        // Copy the values
        if (pBuf) {
            std::copy_n(access_obj->data(), access_obj->size(), pBuf);
        }
        
        // Unlock and delete the access object
        OBNNodeExtInt::unlockPointer(wrapper->second);
        delete access_obj;
    } else {
        // Non-strict port
        using AccessObjectType = typename MQTTInput<OBN_PB,obn_vector_raw<ETYPE>,false>::LockedAccess;
        AccessObjectType* access_obj = static_cast<AccessObjectType*>(wrapper->second);
        
        // Copy the values
        if (pBuf) {
            std::copy_n((*access_obj)->data(), (*access_obj)->size(), pBuf);
        }
        
        // Unlock and delete the access object
        OBNNodeExtInt::unlockPointer(wrapper->second);
        delete access_obj;
    }
    
    // Unlock and delete the managegement objects (wrapper)
    OBNNodeExtInt::unlockPointer(pMan);
    delete wrapper;
}

// Float64
EXPORT
int inputVectorDoubleGet(size_t nodeid, size_t portid, void** pMan, const double** pVals, size_t* nelems) {
    return read_input_vector_get(nodeid, portid, pMan, pVals, nelems);
}

EXPORT
void inputVectorDoubleRelease(void* pMan, double* pBuf) {
    read_input_vector_copy_release(pMan, pBuf);
}

// C++ bool (1 byte)
EXPORT
int inputVectorBoolGet(size_t nodeid, size_t portid, void** pMan, const bool** pVals, size_t* nelems) {
    return read_input_vector_get(nodeid, portid, pMan, pVals, nelems);
}

EXPORT
void inputVectorBoolRelease(void* pMan, bool* pBuf) {
    read_input_vector_copy_release(pMan, pBuf);
}

// Int32
EXPORT
int inputVectorInt32Get(size_t nodeid, size_t portid, void** pMan, const int32_t** pVals, size_t* nelems) {
    return read_input_vector_get(nodeid, portid, pMan, pVals, nelems);
}

EXPORT
void inputVectorInt32Release(void* pMan, int32_t* pBuf) {
    read_input_vector_copy_release(pMan, pBuf);
}

// Int64
EXPORT
int inputVectorInt64Get(size_t nodeid, size_t portid, void** pMan, const int64_t** pVals, size_t* nelems) {
    return read_input_vector_get(nodeid, portid, pMan, pVals, nelems);
}

EXPORT
void inputVectorInt64Release(void* pMan, int64_t* pBuf) {
    read_input_vector_copy_release(pMan, pBuf);
}

// UInt32
EXPORT
int inputVectorUInt32Get(size_t nodeid, size_t portid, void** pMan, const uint32_t** pVals, size_t* nelems) {
    return read_input_vector_get(nodeid, portid, pMan, pVals, nelems);
}

EXPORT
void inputVectorUInt32Release(void* pMan, uint32_t* pBuf) {
    read_input_vector_copy_release(pMan, pBuf);
}

// UInt64
EXPORT
int inputVectorUInt64Get(size_t nodeid, size_t portid, void** pMan, const uint64_t** pVals, size_t* nelems) {
    return read_input_vector_get(nodeid, portid, pMan, pVals, nelems);
}

EXPORT
void inputVectorUInt64Release(void* pMan, uint64_t* pBuf) {
    read_input_vector_copy_release(pMan, pBuf);
}


// Generic (template) function to read from matrix input port - the *GET function
template <typename ETYPE>
int read_input_matrix_get(size_t nodeid, size_t portid, void** pMan, const ETYPE** pVals, size_t* nrows, size_t* ncols)
{
    // Sanity check
    if (pMan == nullptr || nrows == nullptr || ncols == nullptr) {
        return -1000;
    }
    
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    if (portinfo.type != OBNEI_Port_Input) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_INPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Matrix) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    // Query its value based on its type
    if (portinfo.strict) {
        MQTTInput<OBN_PB,obn_matrix_raw<ETYPE>,true> *p = dynamic_cast<MQTTInput<OBN_PB,obn_matrix_raw<ETYPE>,true>*>(portinfo.port);
        
        if (p) {
            if (p->isValuePending()) {
                // Get unique_ptr to an array container.
                auto pv = p->pop();
                
                // pMan contains this array container object, taken from the unique_ptr
                typename obn_matrix_raw<ETYPE>::raw_matrix_data_type* pContainer = pv.release();

                // Returns the pointer pMan which wraps pContainer
                void* pContainerVoid = static_cast<void*>(pContainer);
                *pMan = static_cast<void*>(new AccessManagementWrapper(portinfo.strict, pContainerVoid));
                
                // Lock the pointers so that it will remain in memory
                OBNNodeExtInt::lockPointer(pContainerVoid);
                OBNNodeExtInt::lockPointer(*pMan);
                
                // Number of elements
                *nrows = pContainer->nrows;
                *ncols = pContainer->ncols;
                
                // Returns the array if requested
                if (pVals) {
                    *pVals = pContainer->data.data();
                }
                
                return 0;
            } else {
                // No value
                return 1;
            }
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
        
    } else {
        using ThisPortType = MQTTInput<OBN_PB,obn_matrix_raw<ETYPE>,false>;
        ThisPortType* p = dynamic_cast<ThisPortType*>(portinfo.port);
        
        if (p) {
            // Get direct access to values via a LockedAccess object
            // Create a new dynamic LockedAccess object, which will be wrapped in pMan
            typename ThisPortType::LockedAccess* locked_access = new typename ThisPortType::LockedAccess(p->lock_and_get());
            
            // Returns the pointer pMan which wraps locked_access
            void* locked_access_void = static_cast<void*>(locked_access);
            *pMan = static_cast<void*>(new AccessManagementWrapper(portinfo.strict, locked_access_void));
            
            // Lock the pointer so that it will remain in memory
            OBNNodeExtInt::lockPointer(locked_access_void);
            OBNNodeExtInt::lockPointer(*pMan);
            
            // Number of elements
            *nrows = (*locked_access)->nrows;
            *ncols = (*locked_access)->ncols;
            
            // Returns the array if requested
            if (pVals) {
                *pVals = (*locked_access)->data.data();
            }
            
            return 0;
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
    }
}

// Generic (template) function to copy/release the management object for reading from a matrix input port - the *RELEASE function
template <typename ETYPE>
void read_input_matrix_copy_release(void* pMan, ETYPE* pBuf = nullptr) {
    if (!pMan) {
        return;
    }
    
    // Cast it back to the wrapper type
    AccessManagementWrapper* wrapper = static_cast<AccessManagementWrapper*>(pMan);
    
    if (wrapper->first) {
        // Strict port
        using AccessObjectType = typename obn_matrix_raw<ETYPE>::raw_matrix_data_type;
        AccessObjectType* access_obj = static_cast<AccessObjectType*>(wrapper->second);
        
        // Copy the values
        if (pBuf) {
            std::copy_n(access_obj->data.data(), access_obj->data.size(), pBuf);
        }
        
        // Unlock and delete the access object
        OBNNodeExtInt::unlockPointer(wrapper->second);
        delete access_obj;
    } else {
        // Non-strict port
        using AccessObjectType = typename MQTTInput<OBN_PB,obn_matrix_raw<ETYPE>,false>::LockedAccess;
        AccessObjectType* access_obj = static_cast<AccessObjectType*>(wrapper->second);
        
        // Copy the values
        if (pBuf) {
            std::copy_n((*access_obj)->data.data(), (*access_obj)->data.size(), pBuf);
        }
        
        // Unlock and delete the access object
        OBNNodeExtInt::unlockPointer(wrapper->second);
        delete access_obj;
    }
    
    // Unlock and delete the managegement objects (wrapper)
    OBNNodeExtInt::unlockPointer(pMan);
    delete wrapper;
}


// Float64
EXPORT
int inputMatrixDoubleGet(size_t nodeid, size_t portid, void** pMan, const double** pVals, size_t* nrows, size_t* ncols) {
    return read_input_matrix_get(nodeid, portid, pMan, pVals, nrows, ncols);
}

EXPORT
void inputMatrixDoubleRelease(void* pMan, double* pBuf) {
    read_input_matrix_copy_release(pMan, pBuf);
}

// C++ bool (1 byte)
EXPORT
int inputMatrixBoolGet(size_t nodeid, size_t portid, void** pMan, const bool** pVals, size_t* nrows, size_t* ncols) {
    return read_input_matrix_get(nodeid, portid, pMan, pVals, nrows, ncols);
}

EXPORT
void inputMatrixBoolRelease(void* pMan, bool* pBuf) {
    read_input_matrix_copy_release(pMan, pBuf);
}

// Int32
EXPORT
int inputMatrixInt32Get(size_t nodeid, size_t portid, void** pMan, const int32_t** pVals, size_t* nrows, size_t* ncols) {
    return read_input_matrix_get(nodeid, portid, pMan, pVals, nrows, ncols);
}

EXPORT
void inputMatrixInt32Release(void* pMan, int32_t* pBuf) {
    read_input_matrix_copy_release(pMan, pBuf);
}

// Int64
EXPORT
int inputMatrixInt64Get(size_t nodeid, size_t portid, void** pMan, const int64_t** pVals, size_t* nrows, size_t* ncols) {
    return read_input_matrix_get(nodeid, portid, pMan, pVals, nrows, ncols);
}

EXPORT
void inputMatrixInt64Release(void* pMan, int64_t* pBuf) {
    read_input_matrix_copy_release(pMan, pBuf);
}

// UInt32
EXPORT
int inputMatrixUInt32Get(size_t nodeid, size_t portid, void** pMan, const uint32_t** pVals, size_t* nrows, size_t* ncols) {
    return read_input_matrix_get(nodeid, portid, pMan, pVals, nrows, ncols);
}

EXPORT
void inputMatrixUInt32Release(void* pMan, uint32_t* pBuf) {
    read_input_matrix_copy_release(pMan, pBuf);
}

// UInt64
EXPORT
int inputMatrixUInt64Get(size_t nodeid, size_t portid, void** pMan, const uint64_t** pVals, size_t* nrows, size_t* ncols) {
    return read_input_matrix_get(nodeid, portid, pMan, pVals, nrows, ncols);
}

EXPORT
void inputMatrixUInt64Release(void* pMan, uint64_t* pBuf) {
    read_input_matrix_copy_release(pMan, pBuf);
}


EXPORT
int inputBinaryGet(size_t nodeid, size_t portid, void** pMan, const char** pVals, size_t* nbytes)
{
    // Sanity check
    if (pMan == nullptr || nbytes == nullptr) {
        return -1000;
    }
    
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    if (portinfo.type != OBNEI_Port_Input) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_INPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Binary) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    // Query its value based on its type
    if (portinfo.strict) {
        MQTTInput<OBN_BIN,bool,true> *p = dynamic_cast<MQTTInput<OBN_BIN,bool,true>*>(portinfo.port);
        
        if (p) {
            if (p->isValuePending()) {
                // Get a std::string containing the data
                std::string* pContainer = new std::string(p->pop());
                
                // Returns the pointer pMan which wraps pContainer
                void* pContainerVoid = static_cast<void*>(pContainer);
                *pMan = static_cast<void*>(new AccessManagementWrapper(portinfo.strict, pContainerVoid));
                
                // Lock the pointers so that it will remain in memory
                OBNNodeExtInt::lockPointer(pContainerVoid);
                OBNNodeExtInt::lockPointer(*pMan);
                
                // Number of elements
                *nbytes = pContainer->size();
                
                // Returns the array if requested
                if (pVals) {
                    *pVals = pContainer->data();
                }
                
                return 0;
            } else {
                // No value
                return 1;
            }
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
        
    } else {
        using ThisPortType = MQTTInput<OBN_BIN,bool,false>;
        ThisPortType* p = dynamic_cast<ThisPortType*>(portinfo.port);
        
        if (p) {
            // Get direct access to values via a LockedAccess object
            // Create a new dynamic LockedAccess object, which will be wrapped in pMan
            typename ThisPortType::LockedAccess* locked_access = new typename ThisPortType::LockedAccess(p->lock_and_get());
            
            // Returns the pointer pMan which wraps locked_access
            void* locked_access_void = static_cast<void*>(locked_access);
            *pMan = static_cast<void*>(new AccessManagementWrapper(portinfo.strict, locked_access_void));
            
            // Lock the pointer so that it will remain in memory
            OBNNodeExtInt::lockPointer(locked_access_void);
            OBNNodeExtInt::lockPointer(*pMan);
            
            // Number of bytes
            *nbytes = (*locked_access)->size();
            
            // Returns the array if requested
            if (pVals) {
                *pVals = (*locked_access)->data();
            }
            
            return 0;
        } else {
            reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
            return -4;
        }
    }
}

EXPORT
void inputBinaryRelease(void* pMan, char* pBuf) {
    if (!pMan) {
        return;
    }
    
    // Cast it back to the wrapper type
    AccessManagementWrapper* wrapper = static_cast<AccessManagementWrapper*>(pMan);
    
    if (wrapper->first) {
        // Strict port
        using AccessObjectType = std::string;
        AccessObjectType* access_obj = static_cast<AccessObjectType*>(wrapper->second);
        
        // Copy the values
        if (pBuf) {
            access_obj->copy(pBuf, std::string::npos);
        }
        
        // Unlock and delete the access object
        OBNNodeExtInt::unlockPointer(wrapper->second);
        delete access_obj;
    } else {
        // Non-strict port
        using AccessObjectType = typename MQTTInput<OBN_BIN,bool,false>::LockedAccess;
        AccessObjectType* access_obj = static_cast<AccessObjectType*>(wrapper->second);
        
        // Copy the values
        if (pBuf) {
            (*access_obj)->copy(pBuf, std::string::npos);
        }
        
        // Unlock and delete the access object
        OBNNodeExtInt::unlockPointer(access_obj);
        delete access_obj;
    }
    
    // Unlock and delete the managegement objects (wrapper)
    OBNNodeExtInt::unlockPointer(pMan);
    delete wrapper;
}


// Generic (template) function to write scalar value to a scalar output port
template <typename T>
int WRITE_OUTPUT_SCALAR_HELPER(size_t nodeid, size_t portid, T val) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    if (portinfo.type != OBNEI_Port_Output) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_OUTPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Scalar) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    // Cast the port to the actual object
    MQTTOutput< OBN_PB,obn_scalar<T> > *p = dynamic_cast<MQTTOutput< OBN_PB,obn_scalar<T> >*>(portinfo.port);
    
    if (p) {
        *p = val;
        return 0;
    } else {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
}


// Float64
EXPORT
int outputScalarDoubleSet(size_t nodeid, size_t portid, double val) {
    return WRITE_OUTPUT_SCALAR_HELPER(nodeid, portid, val);
}

// C++ bool (1 byte)
EXPORT
int outputScalarBoolSet(size_t nodeid, size_t portid, bool val) {
    return WRITE_OUTPUT_SCALAR_HELPER(nodeid, portid, val);
}

// Int32
EXPORT
int outputScalarInt32Set(size_t nodeid, size_t portid, int32_t val) {
    return WRITE_OUTPUT_SCALAR_HELPER(nodeid, portid, val);
}

// Int64
EXPORT
int outputScalarInt64Set(size_t nodeid, size_t portid, int64_t val) {
    return WRITE_OUTPUT_SCALAR_HELPER(nodeid, portid, val);
}

// UInt32
EXPORT
int outputScalarUInt32Set(size_t nodeid, size_t portid, uint32_t val) {
    return WRITE_OUTPUT_SCALAR_HELPER(nodeid, portid, val);
}

// UInt64
EXPORT
int outputScalarUInt64Set(size_t nodeid, size_t portid, uint64_t val) {
    return WRITE_OUTPUT_SCALAR_HELPER(nodeid, portid, val);
}


// Generic (template) function to write vector value to a vector output port
template <typename ETYPE>
int write_output_vector_helper(size_t nodeid, size_t portid, const ETYPE* pval, size_t nelems) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    if (portinfo.type != OBNEI_Port_Output) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_OUTPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Vector) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    // Cast the port to the actual object
    MQTTOutput< OBN_PB,obn_vector_raw<ETYPE> > *p = dynamic_cast<MQTTOutput< OBN_PB,obn_vector_raw<ETYPE> >*>(portinfo.port);
    
    if (p) {
        auto &to = *(*p);   // direct access to raw_array_container<ETYPE>
        
        if (!pval || nelems == 0) {
            // if no data is provided, we clear the port's value rather than copying data
            to.clear();
        } else {
            // copy values over
            to.assign(pval, nelems);
        }
        
        return 0;
    } else {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
}


// Float64
EXPORT
int outputVectorDoubleSet(size_t nodeid, size_t portid, double* pval, size_t nelems) {
    return write_output_vector_helper(nodeid, portid, pval, nelems);
}

// C++ bool (1 byte)
EXPORT
int outputVectorBoolSet(size_t nodeid, size_t portid, bool* pval, size_t nelems) {
    return write_output_vector_helper(nodeid, portid, pval, nelems);
}

// Int32
EXPORT
int outputVectorInt32Set(size_t nodeid, size_t portid, int32_t* pval, size_t nelems) {
    return write_output_vector_helper(nodeid, portid, pval, nelems);
}

// Int64
EXPORT
int outputVectorInt64Set(size_t nodeid, size_t portid, int64_t* pval, size_t nelems) {
    return write_output_vector_helper(nodeid, portid, pval, nelems);
}

// UInt32
EXPORT
int outputVectorUInt32Set(size_t nodeid, size_t portid, uint32_t* pval, size_t nelems) {
    return write_output_vector_helper(nodeid, portid, pval, nelems);
}

// UInt64
EXPORT
int outputVectorUInt64Set(size_t nodeid, size_t portid, uint64_t* pval, size_t nelems) {
    return write_output_vector_helper(nodeid, portid, pval, nelems);
}


// Generic (template) function to write matrix value to a matrix output port
template <typename ETYPE>
int write_output_matrix_helper(size_t nodeid, size_t portid, const ETYPE* pval, size_t nrows, size_t ncols) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    if (portinfo.type != OBNEI_Port_Output) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_OUTPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Matrix) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    // Cast the port to the actual object
    MQTTOutput< OBN_PB,obn_matrix_raw<ETYPE> > *p = dynamic_cast<MQTTOutput< OBN_PB,obn_matrix_raw<ETYPE> >*>(portinfo.port);
    
    if (p) {
        auto &to = *(*p);   // direct access to raw_array_container<ETYPE>
        
        if (!pval || nrows == 0 || ncols == 0) {
            // if no data is provided, we clear the port's value rather than copying data
            to.clear();
        } else {
            // copy values over
            to.copy(pval, nrows, ncols);
        }
        
        return 0;
    } else {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
}


// Float64
EXPORT
int outputMatrixDoubleSet(size_t nodeid, size_t portid, double* pval, size_t nrows, size_t ncols) {
    return write_output_matrix_helper(nodeid, portid, pval, nrows, ncols);
}

// C++ bool (1 byte)
EXPORT
int outputMatrixBoolSet(size_t nodeid, size_t portid, bool* pval, size_t nrows, size_t ncols) {
    return write_output_matrix_helper(nodeid, portid, pval, nrows, ncols);
}

// Int32
EXPORT
int outputMatrixInt32Set(size_t nodeid, size_t portid, int32_t* pval, size_t nrows, size_t ncols) {
    return write_output_matrix_helper(nodeid, portid, pval, nrows, ncols);
}

// Int64
EXPORT
int outputMatrixInt64Set(size_t nodeid, size_t portid, int64_t* pval, size_t nrows, size_t ncols) {
    return write_output_matrix_helper(nodeid, portid, pval, nrows, ncols);
}

// UInt32
EXPORT
int outputMatrixUInt32Set(size_t nodeid, size_t portid, uint32_t* pval, size_t nrows, size_t ncols) {
    return write_output_matrix_helper(nodeid, portid, pval, nrows, ncols);
}

// UInt64
EXPORT
int outputMatrixUInt64Set(size_t nodeid, size_t portid, uint64_t* pval, size_t nrows, size_t ncols) {
    return write_output_matrix_helper(nodeid, portid, pval, nrows, ncols);
}


EXPORT
int outputBinarySet(size_t nodeid, size_t portid, const char* pval, size_t nbytes) {
    // Sanity check
    if (nbytes > 0 && !pval) {
        return -1000;
    }
    
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    if (portinfo.type != OBNEI_Port_Output) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_OUTPUT);
        return -3;
    }
    
    if (portinfo.container != OBNEI_Container_Binary) {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
    
    // Cast the port to the actual object
    MQTTOutput<OBN_BIN,bool> *p = dynamic_cast<MQTTOutput<OBN_BIN,bool>*>(portinfo.port);
    
    if (p) {
        p->message(pval, nbytes);
        return 0;
    } else {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
}


EXPORT
int portConnect(size_t nodeid, size_t portid, const char* srcport) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1001;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -1002;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    
    // Request to connect
    auto result = portinfo.port->connect_from_port(srcport);
    
    // Get the result
    if (result.first != 0) {
        reportError(result.second.c_str());
    }
    
    return result.first;
}


EXPORT
int portEnableRcvEvent(size_t nodeid, size_t portid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError(OBNNodeExtInt::StdMsgs::NODE_NOT_EXIST);
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError(OBNNodeExtInt::StdMsgs::INVALID_PORT_ID);
        return -2;
    }
    
    // Obtain the port's info
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    if (portinfo.type != OBNEI_Port_Input) {
        reportError(OBNNodeExtInt::StdMsgs::PORT_NOT_INPUT);
        return -3;
    }
    
    // Cast the port to input port
    InputPortBase* p = dynamic_cast<InputPortBase*>(portinfo.port);
    if (p) {
        // Set the callback for the port
        p->setMsgRcvCallback(std::bind(&MQTTNodeExt::extint_inputport_msgrcvd_callback, pnode, portid), false);
        return 0;
    } else {
        reportError(OBNNodeExtInt::StdMsgs::INTERNAL_PORT_NOT_MATCH_DECL_TYPE);
        return -4;
    }
}





//    // Get full MQTT name of a port
//    MEX_DEFINE(MQTTName) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 2);
//        OutputArguments output(nlhs, plhs, 1);
//
//        MQTTNode *ynode = Session<MQTTNode>::get(input.get(0));
//        MQTTNode::portPtr yport = ynode->portObject(input.get<unsigned int>(1));
//        if (yport != nullptr) {
//            output.set(0, std::string(yport->getName()));
//        }
//        else {
//            mexErrMsgIdAndTxt("MATLAB:mMQTT:enableCallback", "Port doesn't exist.");
//        }
//    }
//
//
//    // Get the ID of a port by its name
//    // Returns ID of port; 0 if port doesn't exist
//    MEX_DEFINE(portID) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 2);
//        OutputArguments output(nlhs, plhs, 1);
//
//        MQTTNode *ynode = Session<MQTTNode>::get(input.get(0));
//        output.set(0, ynode->portID(input.get<string>(1)));
//    }


// Returns the name of the node (without workspace name)
// Args: node object pointer
// Returns: name of the node as a string
//    MEX_DEFINE(nodeName) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 1);
//        OutputArguments output(nlhs, plhs, 1);
//
//        MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
//        output.set(0, ynode->name());
//    }

