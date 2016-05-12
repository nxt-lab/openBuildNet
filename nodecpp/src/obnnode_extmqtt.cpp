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

#include <obnnode.h>    // node.C++ framework
#include <obnnode_extmqtt.h>

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
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_vector,element,true,name);
            } else {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_vector,element,false,name);
            }
            break;
            
        case OBNEI_Container_Matrix:
            if (strict) {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_matrix,element,true,name);
            } else {
                port = YNM_PORT_CLASS_BY_NAME_STRICT(InputPortBase,MQTTInput,obn_matrix,element,false,name);
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
            port = YNM_PORT_CLASS_BY_NAME(MQTTOutputPortBase,MQTTOutput,obn_vector,element,name);
            break;
            
        case OBNEI_Container_Matrix:
            port = YNM_PORT_CLASS_BY_NAME(MQTTOutputPortBase,MQTTOutput,obn_matrix,element,name);
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
        reportError("MQTTNODE:runStep", "Node is in error state; can't continue simulation; please stop the node (restart it) to clear the error state before continuing.");
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
                    _ml_current_event.type = OBNEI_RCV;   // The default event is Message Received; check evt->event_type for other types
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
                    reportError("MQTTNODE:runStep", "Could not open the SMN port. Check the network and the name server.");
                    break;
                }
                
                // Initialize the node's state
                initializeForSimulation();
                
                // Switch to STARTED to wait for INIT message from the SMN
                _node_state = NODE_STARTED;
                break;
                
            default:
                reportError("MQTTNODE:runStep", "Internal error: invalid node's state.");
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
int createOBNNode(const char* name, const char* workspace, const char* addr, size_t* id) {
    if (!name) {
        return -1;      // Empty name
    }
    
    // Check if the node has already existed
    {
        std::string newFullName( (workspace)?(std::string(workspace) + '/' + name):(name) );
        if (OBNNodeExtInt::Session<MQTTNodeExt>::exist([newFullName&](const MQTTNodeExt& node){
            return node.full_name() == newFullName;
        })) {
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
        return -3;      // Error while starting the node object
    }
}


// Delete a node object
int deleteOBNNode(size_t id) {
    return (OBNNodeExtInt::Session<MQTTNodeExt>::destroy(id))?0:-1;
}

// Create a new input port on a node
// Arguments: node ID, port's name, format type, container type, element type, strict or not
// Returns port's id; or negative number if error.
// id is an integer starting from 0.
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
        return -2;      // Invalid port name.
    }
    
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(id);
    if (!pnode) {
        return -3;      // Node doesn't exist
    }
    
    // Try to add the port
    return pnode->createInputPort(portname, format, container, element, strict);
}

// Create a new output port on a node
// Arguments: node ID, port's name, format type, container type, element type
// Returns port's id; or negative number if error.
// id is an integer starting from 0.
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
        return -2;      // Invalid port name.
    }
    
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(id);
    if (!pnode) {
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
int outputSendSync(size_t nodeid, size_t portid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError("Node does not exist.");
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError("Invalid port ID");
        return -2;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    if (portinfo.type != OBNEI_Port_Output) {
        reportError("Given port is not a physical output.");
        return -3;
    }
    
    // Cast the pointer to an output port object and send
    MQTTOutputPortBase *p = dynamic_cast<MQTTOutputPortBase*>(portinfo.port);
    if (p) {
        p->sendSync();
    } else {
        reportError("Internal error: port object type does not match its declared type.");
        return -4;
    }
    
    return 0;
}

// Is there a value pending at an input port?
// Args: node ID, port's ID
// Returns: true if >0, false if =0, error if <0.
int inputPending(size_t nodeid, size_t portid) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError("Node does not exist.");
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError("Invalid port ID");
        return -2;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = pnode->_all_ports[portid];
    if (portinfo.type != OBNEI_Port_Input) {
        reportError("Given port is not an input output.");
        return -3;
    }
    
    // Cast and query the port
    OBNnode::InputPortBase *port = dynamic_cast<OBNnode::InputPortBase*>(portinfo.port);
    if (port) {
        return port->isValuePending()?1:0;
    } else {
        reportError("Internal error: port object is not an input port.");
        return -4;
    }
}

// Returns information about a port.
// Arguments: node ID, port's ID, pointer to a valid OBNEI_PortInfo structure to receive info
// Returns: 0 if successful.
int portInfo(size_t nodeid, size_t portid, OBNEI_PortInfo* pInfo) {
    // Find node
    MQTTNodeExt* pnode = OBNNodeExtInt::Session<MQTTNodeExt>::get(nodeid);
    if (!pnode) {
        reportError("Node does not exist.");
        return -1;
    }
    
    // Find port
    if (portid >= pnode->_all_ports.size()) {
        reportError("Invalid port ID");
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



























#define READ_INPUT_SCALAR_HELPER(A,B) \
MQTTInput<A,obn_scalar<B>,false> *p = dynamic_cast<MQTTInput<A,obn_scalar<B>,false>*>(portinfo.port); \
if (p) { output.set(0, p->get()); } \
else { reportError("MQTTNODE:readInput", "Internal error: port object type does not match its declared type."); }

#define READ_INPUT_SCALAR_HELPER_STRICT(A,B) \
MQTTInput<A,obn_scalar<B>,true> *p = dynamic_cast<MQTTInput<A,obn_scalar<B>,true>*>(portinfo.port); \
if (p) { output.set(0, p->pop()); } \
else { reportError("MQTTNODE:readInput", "Internal error: port object type does not match its declared type."); }

// For vectors, regardless of how vectors are stored in Matlab's mxArray vs. in Eigen, we can simply copy the exact number of values from one to another.
template <typename ETYPE>
mxArray* read_input_vector_helper(std::function<mxArray*(int, int)> FUNC, OBNnode::PortBase *port, bool strict)
{
    if (strict) {
        MQTTInput<OBN_PB,obn_vector<ETYPE>,true> *p = dynamic_cast<MQTTInput<OBN_PB,obn_vector<ETYPE>,true>*>(port);
        if (p) {
            // Get unique_ptr to an Eigen's object
            auto pv = p->pop();
            
            if (pv) {
                // Copy to MxArray
                MxArray ml(FUNC(pv->size(), 1));
                std::copy(pv->data(), pv->data()+pv->size(), ml.getData<ETYPE>());
                return ml.release();
            } else {
                // Empty vector
                MxArray ml(FUNC(0,0));
                return ml.release();
            }
        }
    } else {
        MQTTInput<OBN_PB,obn_vector<ETYPE>,false> *p = dynamic_cast<MQTTInput<OBN_PB,obn_vector<ETYPE>,false>*>(port);
        if (p) {
            // Get direct access to value
            auto value_access = p->lock_and_get();
            const auto& pv = *value_access;
            
            MxArray ml(FUNC(pv.size(), 1));
            std::copy(pv.data(), pv.data()+pv.size(), ml.getData<ETYPE>());
            return ml.release();
        }
    }
    
    // Reach here only if there is error
    reportError("MQTTNODE:readInput", "Internal error: port object type does not match its declared type.");
    return nullptr;
}

// For matrices, the order in which Matlab and Eigen store matrices (column-major or row-major) will affect how data can be copied.  Because both Eigen and Matlab use column-major order by default, we can safely copy the data in memory between them without affecting the data.  For completeness, there are commented code lines that safely transfer data by accessing element-by-element, but this would be slower.
template <typename ETYPE>
mxArray* read_input_matrix_helper(std::function<mxArray*(int, int)> FUNC, OBNnode::PortBase *port, bool strict) {
    if (strict) {
        MQTTInput<OBN_PB,obn_matrix<ETYPE>,true> *p = dynamic_cast<MQTTInput<OBN_PB,obn_matrix<ETYPE>,true>*>(port);
        if (p) {
            // Get unique_ptr to an Eigen's object
            auto pv = p->pop();
            
            if (pv) {
                auto nr = pv->rows(), nc = pv->cols();
                MxArray ml(FUNC(nr, nc));
                
                // The following lines copy the whole chunk of raw data from Eigen to MEX/Matlab, it's faster but requires that Matlab and Eigen must use the same order type
                std::copy(pv->data(), pv->data()+nr*nc, ml.getData<ETYPE>());
                return ml.release();
                
                // // The following lines copy element-by-element: it's safe but slow.
                //        for (std::size_t c = 0; c < nc; ++c) {
                //            for (std::size_t r = 0; r < nr; ++r) {
                //                ml.set(r, c, pv->coeff(r, c));
                //            }
                //        }
            } else {
                // Empty matrix
                MxArray ml(FUNC(0,0));
                return ml.release();
            }
        }
    } else {
        MQTTInput<OBN_PB,obn_matrix<ETYPE>,false> *p = dynamic_cast<MQTTInput<OBN_PB,obn_matrix<ETYPE>,false>*>(port);
        if (p) {
            // Get direct access to value
            auto value_access = p->lock_and_get();
            const auto& pv = *value_access;
            
            auto nr = pv.rows(), nc = pv.cols();
            MxArray ml(FUNC(nr, nc));
            
            // The following lines copy the whole chunk of raw data from Eigen to MEX/Matlab, it's faster but requires that Matlab and Eigen must use the same order type
            std::copy(pv.data(), pv.data()+nr*nc, ml.getData<ETYPE>());
            return ml.release();
            
            // // The following lines copy element-by-element: it's safe but slow.
            //        for (std::size_t c = 0; c < nc; ++c) {
            //            for (std::size_t r = 0; r < nr; ++r) {
            //                ml.set(r, c, pv.coeff(r, c));
            //            }
            //        }
        }
    }
    
    // Reach here only if there is error
    reportError("MQTTNODE:readInput", "Internal error: port object type does not match its declared type.");
    return nullptr;
}


#define WRITE_OUTPUT_SCALAR_HELPER(A,B) \
MQTTOutput<A,obn_scalar<B>> *p = dynamic_cast<MQTTOutput<A,obn_scalar<B>>*>(portinfo.port); \
if (p) { *p = input.get<B>(2); } \
else { reportError("MQTTNODE:writeOutput", "Internal error: port object type does not match its declared type."); }


// To convert a vector from mxArray to Eigen, with possibility that their types are different
// - Use MEXPLUS to convert from mxArray to vector<D> of appropriate type
// - Use Eigen::Map to create an Eigen's vector on the data returned by the vector object.
// - Assign the map object to the Eigen vector object (inside the port object).
template <typename ETYPE>
void write_output_vector_helper(const InputArguments &input, OBNnode::PortBase *port) {
    MQTTOutput<OBN_PB,obn_vector<ETYPE>> *p = dynamic_cast<MQTTOutput<OBN_PB,obn_vector<ETYPE>>*>(port);
    if (p) {
        auto from = MxArray(input.get(2)); // MxArray object
        auto &to = *(*p);
        if (from.isEmpty()) {
            // if "from" is empty then we clear the data in the port, do not copy
            to.resize(0);
        } else {
            // if "from" is not empty then we can copy the data
            to = to.Map(from.getData<ETYPE>(), from.size());
        }
        
        // // Another way to copy
        //        std::vector<ETYPE> v(input.get<std::vector<ETYPE>>(2));
        //        *p = (*(*p)).Map(v.data(), v.size());
    } else {
        reportError("MQTTNODE:writeOutput", "Internal error: port object type does not match its declared type.");
    }
}

// For matrices, the order in which Matlab and Eigen store matrices (column-major or row-major) will affect how data can be copied.  Because both Eigen and Matlab use column-major order by default, we can safely copy the data in memory between them without affecting the data.  For completeness, there are commented code lines that safely transfer data by accessing element-by-element, but this would be slower.
template <typename ETYPE>
void write_output_matrix_helper(const InputArguments &input, OBNnode::PortBase *port) {
    MQTTOutput<OBN_PB,obn_matrix<ETYPE>> *p = dynamic_cast<MQTTOutput<OBN_PB,obn_matrix<ETYPE>>*>(port);
    if (p) {
        auto from = MxArray(input.get(2)); // MxArray object
        auto nr = from.rows(), nc = from.cols();
        auto &to = *(*p);
        
        if (from.isEmpty()) {
            // if "from" is empty then we clear the data in the port, do not copy
            to.resize(nr, nc);
        } else {
            // if "from" is not empty then we can copy the data
            to = to.Map(from.getData<ETYPE>(), nr, nc);
        }
        
        //        // The following lines copy element-by-element: it's safe but slow.
        //        to.resize(nr, nc);
        //        for (std::size_t c = 0; c < nc; ++c) {
        //            for (std::size_t r = 0; r < nr; ++r) {
        //                to(r, c) = from.at<ETYPE>(r, c);
        //            }
        //        }
    } else {
        reportError("MQTTNODE:writeOutput", "Internal error: port object type does not match its declared type.");
    }
}

/* === Port interface === */

// Read the current value of a non-strict input port, or pop the top/front value of a strict input port.
// Args: node object pointer, port's ID
// Returns: value in an appropriate Matlab's type
// For strict ports, if there is no value pending, a default / empty value will be returned.
// If the port contains binary data, the function will return a string containing the binary data.
MEX_DEFINE(readInput) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 2);
    OutputArguments output(nlhs, plhs, 1);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    unsigned int id = input.get<unsigned int>(1);
    if (id >= ynode->_all_ports.size()) {
        reportError("MQTTNODE:readInput", "Invalid port ID to read from.");
        return;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = ynode->_all_ports[id];
    if (portinfo.type != OBNEI_Port_Input) {
        reportError("MQTTNODE:readInput", "Given port is not an input.");
        return;
    }
    
    // Query its value based on its type, casting to the appropriate type
    switch (portinfo.container) {
        case 's':
            switch (portinfo.elementType) {
                case MQTTNodeExt::PortInfo::DOUBLE:
                    if (portinfo.strict) {
                        READ_INPUT_SCALAR_HELPER_STRICT(OBN_PB, double)
                    } else {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, double)
                    }
                    break;
                    
                case MQTTNodeExt::PortInfo::LOGICAL:
                    if (portinfo.strict) {
                        READ_INPUT_SCALAR_HELPER_STRICT(OBN_PB, bool)
                    } else {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, bool)
                    }
                    break;
                    
                case MQTTNodeExt::PortInfo::INT32:
                    if (portinfo.strict) {
                        READ_INPUT_SCALAR_HELPER_STRICT(OBN_PB, int32_t)
                    } else {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, int32_t)
                    }
                    break;
                    
                case MQTTNodeExt::PortInfo::INT64:
                    if (portinfo.strict) {
                        READ_INPUT_SCALAR_HELPER_STRICT(OBN_PB, int64_t)
                    } else {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, int64_t)
                    }
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT32:
                    if (portinfo.strict) {
                        READ_INPUT_SCALAR_HELPER_STRICT(OBN_PB, uint32_t)
                    } else {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, uint32_t)
                    }
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT64:
                    if (portinfo.strict) {
                        READ_INPUT_SCALAR_HELPER_STRICT(OBN_PB, uint64_t)
                    } else {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, uint64_t)
                    }
                    break;
                    
                default:
                    reportError("MQTTNODE:readInput", "Internal error: port's element type is invalid.");
                    break;
            }
            break;
            
        case 'v':
            switch (portinfo.elementType) {
                case MQTTNodeExt::PortInfo::DOUBLE:
                    output.set(0, read_input_vector_helper<double>(MxArray::Numeric<double>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::LOGICAL:
                    output.set(0, read_input_vector_helper<bool>(MxArray::Logical, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::INT32:
                    output.set(0, read_input_vector_helper<int32_t>(MxArray::Numeric<int32_t>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::INT64:
                    output.set(0, read_input_vector_helper<int64_t>(MxArray::Numeric<int64_t>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT32:
                    output.set(0, read_input_vector_helper<uint32_t>(MxArray::Numeric<uint32_t>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT64:
                    output.set(0, read_input_vector_helper<uint64_t>(MxArray::Numeric<uint64_t>, portinfo.port, portinfo.strict));
                    break;
                    
                default:
                    reportError("MQTTNODE:readInput", "Internal error: port's element type is invalid.");
                    break;
            }
            break;
            
        case 'm':
            switch (portinfo.elementType) {
                case MQTTNodeExt::PortInfo::DOUBLE:
                    output.set(0, read_input_matrix_helper<double>(MxArray::Numeric<double>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::LOGICAL:
                    output.set(0, read_input_matrix_helper<bool>(MxArray::Logical, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::INT32:
                    output.set(0, read_input_matrix_helper<int32_t>(MxArray::Numeric<int32_t>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::INT64:
                    output.set(0, read_input_matrix_helper<int64_t>(MxArray::Numeric<int64_t>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT32:
                    output.set(0, read_input_matrix_helper<uint32_t>(MxArray::Numeric<uint32_t>, portinfo.port, portinfo.strict));
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT64:
                    output.set(0, read_input_matrix_helper<uint64_t>(MxArray::Numeric<uint64_t>, portinfo.port, portinfo.strict));
                    break;
                    
                default:
                    reportError("MQTTNODE:readInput", "Internal error: port's element type is invalid.");
                    break;
            }
            break;
            
        case 'b':
            if (portinfo.strict) {
                // A binary string is read, the element type is ignored
                MQTTInput<OBN_BIN,bool,true> *p = dynamic_cast<MQTTInput<OBN_BIN,bool,true>*>(portinfo.port);
                if (p) {
                    output.set(0, p->pop());
                } else {
                    reportError("MQTTNODE:readInput", "Internal error: port object type does not match its declared type.");
                }
            } else {
                // A binary string is read, the element type is ignored
                MQTTInput<OBN_BIN,bool,false> *p = dynamic_cast<MQTTInput<OBN_BIN,bool,false>*>(portinfo.port);
                if (p) {
                    output.set(0, p->get());
                } else {
                    reportError("MQTTNODE:readInput", "Internal error: port object type does not match its declared type.");
                }
            }
            break;
            
        default:    // This should never happen
            reportError("MQTTNODE:readInput", "Internal error: port's container type is invalid.");
            break;
    }
}

// Set the value of a physical output port, but does not send it immediately.
// Usually the value will be sent out at the end of the event callback (UPDATEY).
// Args: node object pointer, port's ID, value (in appropriate Matlab type)
// Returns: none
// This function will return an error if the given port is not a physical output port, or if the given value cannot be converted to the port's type (e.g. write a string to a numeric port).
// If the port contains binary data, the value must be a string containing the binary data.
MEX_DEFINE(writeOutput) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 3);
    OutputArguments output(nlhs, plhs, 0);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    unsigned int id = input.get<unsigned int>(1);
    if (id >= ynode->_all_ports.size()) {
        reportError("MQTTNODE:writeOutput", "Invalid port ID to write to.");
        return;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = ynode->_all_ports[id];
    if (portinfo.type != OBNEI_Port_Output) {
        reportError("MQTTNODE:writeOutput", "Given port is not a physical output.");
        return;
    }
    
    // std::cout << portinfo.container << " of " << portinfo.elementType << std::endl;
    
    // Set its value based on its type, trying to convert from the given Matlab type
    switch (portinfo.container) {
        case 's':
            switch (portinfo.elementType) {
                case MQTTNodeExt::PortInfo::DOUBLE: {
                    WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, double)
                    break;
                }
                    
                case MQTTNodeExt::PortInfo::LOGICAL: {
                    WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, bool)
                    break;
                }
                    
                case MQTTNodeExt::PortInfo::INT32: {
                    WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, int32_t)
                    break;
                }
                    
                case MQTTNodeExt::PortInfo::INT64: {
                    WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, int64_t)
                    break;
                }
                    
                case MQTTNodeExt::PortInfo::UINT32: {
                    WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, uint32_t)
                    break;
                }
                    
                case MQTTNodeExt::PortInfo::UINT64: {
                    WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, uint64_t)
                    break;
                }
                    
                default:
                    reportError("MQTTNODE:writeOutput", "Internal error: port's element type is invalid.");
                    break;
            }
            break;
            
        case 'v':
            switch (portinfo.elementType) {
                case MQTTNodeExt::PortInfo::DOUBLE:
                    write_output_vector_helper<double>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::LOGICAL: {
                    // This case is special because vector<bool> does not have data()
                    MQTTOutput<OBN_PB,obn_vector_raw<bool>> *p = dynamic_cast<MQTTOutput<OBN_PB,obn_vector_raw<bool>>*>(portinfo.port);
                    if (p) {
                        std::vector<bool> v(input.get<std::vector<bool>>(2));
                        (*(*p)).resize(v.size());
                        // Copy element-by-element
                        for (std::size_t i = 0; i < v.size(); ++i) {
                            (*(*p))(i) = v[i];
                        }
                    } else {
                        reportError("MQTTNODE:writeOutput", "Internal error: port object type does not match its declared type.");
                    }
                    break;
                }
                    
                case MQTTNodeExt::PortInfo::INT32:
                    write_output_vector_helper<int32_t>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::INT64:
                    write_output_vector_helper<int64_t>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT32:
                    write_output_vector_helper<uint32_t>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT64:
                    write_output_vector_helper<uint64_t>(input, portinfo.port);
                    break;
                    
                default:
                    reportError("MQTTNODE:writeOutput", "Internal error: port's element type is invalid.");
                    break;
            }
            break;
            
        case 'm':
            switch (portinfo.elementType) {
                case MQTTNodeExt::PortInfo::DOUBLE:
                    write_output_matrix_helper<double>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::LOGICAL:
                    write_output_matrix_helper<bool>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::INT32:
                    write_output_matrix_helper<int32_t>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::INT64:
                    write_output_matrix_helper<int64_t>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT32:
                    write_output_matrix_helper<uint32_t>(input, portinfo.port);
                    break;
                    
                case MQTTNodeExt::PortInfo::UINT64:
                    write_output_matrix_helper<uint64_t>(input, portinfo.port);
                    break;
                    
                default:
                    reportError("MQTTNODE:writeOutput", "Internal error: port's element type is invalid.");
                    break;
            }
            break;
            
        case 'b': {
            // A binary string is written, the element type is ignored
            MQTTOutput<OBN_BIN,bool> *p = dynamic_cast<MQTTOutput<OBN_BIN,bool>*>(portinfo.port);
            if (p) {
                p->message(input.get<std::string>(2));
            } else {
                reportError("MQTTNODE:writeOutput", "Internal error: binary port object type does not match its declared type.");
            }
            break;
        }
            
        default:    // This should never happen
            reportError("MQTTNODE:writeOutput", "Internal error: port's container type is invalid.");
            break;
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



/* === MQTTNode interface === */

// Get the next port event (e.g. message received) with a possible timeout.
// Args: node object pointer, timeout (double in seconds, can be <= 0.0 if no timeout, i.e. returns immediately).
// Returns: [status, event_type, port_index]
// where:
// - status = true if there is an event returned (hence the remaining returned values are valid); = false if there is no event (i.e. timeout).
// - event_type = 0 for Message Received.
// - port_index is the index of the port associated with the event.
MEX_DEFINE(portEvent) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 2);
    OutputArguments output(nlhs, plhs, 3);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    
    // get next PortEvent from node
    auto evt = ynode->getNextPortEvent(input.get<double>(1));
    
    if (!evt) {
        // No event -> returns immediately
        output.set(0, false);
        output.set(1, -1);
        output.set(2, -1);
        return;
    }
    
    // Convert the event to outputs of this MEX function
    output.set(0, true);
    output.set(2, evt->port_index);
    
    switch (evt->event_type) {
        case OBNnode::MQTTNodeExt::PortEvent::RCV:
            output.set(1, 0);
            break;
            
        default:
            reportError("MQTTNODE:portEvent", "Internal error: unrecognized port event type.");
            break;
    }
}


// Runs the node's simulation until the next event, or until the node stops or has errors
// Args: node object pointer, timeout (double in seconds, can be <= 0.0 if no timeout)
// Returns: status (see MQTTNodeExt::runStep()), event type (a string), event argument (of an appropriate data type depending on the event type)
MEX_DEFINE(runStep) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 2);
    OutputArguments output(nlhs, plhs, 3);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    
    // Call node's runStep
    int result = ynode->runStep(input.get<double>(1));
    
    
    // Convert the result to outputs of this MEX function
    output.set(0, result);
    if (ynode->_ml_pending_event) {
        switch (ynode->_ml_current_event.type) {
            case OBNnode::MQTTNodeExt::MLE_Y:
                output.set(1, "Y");
                output.set(2, ynode->_ml_current_event.arg.mask);
                break;
                
            case OBNnode::MQTTNodeExt::MLE_X:
                output.set(1, "X");
                output.set(2, ynode->_ml_current_event.arg.mask);   // This mask is set to the current update mask
                break;
                
            case OBNnode::MQTTNodeExt::MLE_INIT:
                output.set(1, "INIT");
                output.set(2, 0);
                break;
                
            case OBNnode::MQTTNodeExt::MLE_TERM:
                output.set(1, "TERM");
                output.set(2, 0);
                break;
                
            case OBNnode::MQTTNodeExt::MLE_RCV:
                output.set(1, "RCV");
                output.set(2, ynode->_ml_current_event.arg.index);
                break;
                
            default:
                reportError("MQTTNODE:runStep", "Internal error: unrecognized Matlab event type.");
                break;
        }
    } else {
        output.set(1, "");
        output.set(2, 0);
    }
}


// Returns the current simulation time of the node with a desired time unit.
// Args: node object pointer, the time unit
// Returns: current simulation time as a double (real number)
// The time unit is an integer specifying the desired time unit. The allowed values are:
// 0 = second, -1 = millisecond, -2 = microsecond, 1 = minute, 2 = hour
MEX_DEFINE(simTime) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 2);
    OutputArguments output(nlhs, plhs, 1);
    
    int timeunit = input.get<int>(1);
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    
    double timevalue;
    
    switch (timeunit) {
        case 0:
            timevalue = ynode->currentSimulationTime<std::chrono::seconds>();
            break;
            
        case 1:
            timevalue = ynode->currentSimulationTime<std::chrono::minutes>();
            break;
            
        case 2:
            timevalue = ynode->currentSimulationTime<std::chrono::hours>();
            break;
            
        case -1:
            timevalue = ynode->currentSimulationTime<std::chrono::milliseconds>();
            break;
            
        case -2:
            timevalue = ynode->currentSimulationTime<std::chrono::microseconds>();
            break;
            
        default:
            reportError("MQTTNODE:simTime", "Internal error: Unrecognized time unit argument.");
            break;
    }
    
    output.set(0, timevalue);
}

// Returns the current wallclock time of the node.
// Args: node object pointer
// Returns: current wallclock time as a POSIX time value.
MEX_DEFINE(wallclock) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 1);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    output.set(0, ynode->currentWallClockTime());
}


// Request an irregular future update.
// This is a blocking call, possibly with a timeout, that waits until it receives the response from the SMN or until a timeout.
// Args: node object pointer, future time (integer value in the future), update mask of the requested update (int64), timeout (double, can be <= 0)
// Returns: status of the request: 0 if successful (accepted), -1 if timeout (failed), -2 if request is invalid, >0 if other errors (failed, see OBN documents for details).
MEX_DEFINE(futureUpdate) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 4);
    OutputArguments output(nlhs, plhs, 1);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    auto *pCond = ynode->requestFutureUpdate(input.get<simtime_t>(1), input.get<updatemask_t>(2), false);
    if (pCond) {
        output.set(0, ynode->resultFutureUpdate(pCond, input.get<double>(3)));
    } else {
        output.set(0, -2);
    }
}


// Request/notify the SMN to stop, then terminate the node's simulation regardless of whether the request was accepted or not. See MQTTNodeExt::stopSimulation for details.
// Args: node object pointer
// Return: none
MEX_DEFINE(stopSim) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 0);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    ynode->stopSimulation();
}

// This method requests the SMN/GC to stop the simulation (by sending a request message to the SMN).
// See MQTTNodeExt::requestStopSimulation() for details.
// Args: node object pointer
// Return: none
MEX_DEFINE(requestStopSim) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 0);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    ynode->requestStopSimulation();
}


// Check if the current state of the node is ERROR
// Args: node object pointer
// Returns: true/false
MEX_DEFINE(isNodeErr) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 1);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    output.set(0, ynode->hasError());
}

// Check if the current state of the node is RUNNING
// Args: node object pointer
// Returns: true/false
MEX_DEFINE(isNodeRunning) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 1);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    output.set(0, ynode->nodeState() == OBNnode::MQTTNode::NODE_RUNNING);
}

// Check if the current state of the node is STOPPED
// Args: node object pointer
// Returns: true/false
MEX_DEFINE(isNodeStopped) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 1);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    output.set(0, ynode->nodeState() == OBNnode::MQTTNode::NODE_STOPPED);
}



// Enable the message received event at an input port
// Args: node object pointer, port's ID
// Returns: true if successful; false otherwise
MEX_DEFINE(enableRcvEvent) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 2);
    OutputArguments output(nlhs, plhs, 1);
    output.set(0, false);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    unsigned int id = input.get<unsigned int>(1);
    if (id >= ynode->_all_ports.size()) {
        reportError("MQTTNODE:enableRcvEvent", "Invalid port ID.");
        return;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = ynode->_all_ports[id];
    if (portinfo.type != OBNEI_Port_Input) {
        reportError("MQTTNODE:enableRcvEvent", "Given port is not an input.");
        return;
    }
    
    // Cast the port to input port
    InputPortBase* p = dynamic_cast<InputPortBase*>(portinfo.port);
    if (p) {
        // Set the callback for the port
        p->setMsgRcvCallback(std::bind(&MQTTNodeExt::matlab_inputport_msgrcvd_callback, ynode, id), false);
        output.set(0, true);
    } else {
        reportError("MQTTNODE:enableRcvEvent", "Internal error: port object is not an input.");
        return;
    }
}


// Request to connect a given port to a port on this node.
// Arguments: node object pointer, port's ID, source port's name (string)
// Returns: result (int), message (string)
MEX_DEFINE(connectPort) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 3);
    OutputArguments output(nlhs, plhs, 2);
    
    MQTTNodeExt *ynode = Session<MQTTNodeExt>::get(input.get(0));
    unsigned int id = input.get<unsigned int>(1);
    if (id >= ynode->_all_ports.size()) {
        reportError("MQTTNODE:connectPort", "Invalid port ID.");
        return;
    }
    
    // Obtain the port
    MQTTNodeExt::PortInfo portinfo = ynode->_all_ports[id];
    
    // Request to connect
    auto result = portinfo.port->connect_from_port(input.get<string>(2));
    
    output.set(0, result.first);
    output.set(1, result.second);
}

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

