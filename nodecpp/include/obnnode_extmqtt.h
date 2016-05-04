/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_extmqtt.h
 * \brief Generic external interface for MQTTNode of the openBuildNet simulation framework.
 *
 * Requires MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_EXTMQTT_H_
#define OBNNODE_EXTMQTT_H_

#ifndef OBNNODE_COMM_MQTT
#error This file must be compiled with MQTT enabled
#endif

#include <cstdio>       // printf
#include <vector>

#include <obnnode.h>    // node.C++ framework
#include <obnnode_ext.h>    // The external interface: all types and function definitions used by external language


/* IMPORTANT NOTE:
 *
 * Because of the tight inter-dependency between a node object and its port objects, care must be taken to make sure that when a node is destroyed, the port objects are also be destroyed and not orphaned.
 * For this reason, unlike the node.C++ framework, the node object will manage all its port objects explicitly, i.e. port objects are not managed by the external language at all.
 * The node object will provide an interface for accessing the port objects managed by it, indirectly through some kind of IDs, not via pointers.
 * Similarly, it is dangerous for the external language to have direct access to the node object pointers, so this library will manage its collection of node objects, and provide access through IDs or names rather than pointers.
 */


// Report an error and (usually) terminate.
void reportError(const char* msgID, const char* msg);

// Report a warning
void reportWarning(const char* msgID, const char* msg);


/** The main node class for external interface. */
class MQTTNodeExt: public OBNnode::MQTTNodeBase {
public:
    
    /** Structure containing info about a port in this node. */
    struct PortInfo {
        OBNnode::PortBase *port;
        enum { INPUTPORT, OUTPUTPORT, DATAPORT } type;
        char container;     ///< 's', 'v', 'm', 'b'
        enum { NONE, DOUBLE, LOGICAL, INT32, UINT32, INT64, UINT64 } elementType;
        bool strict;    ///< Only for input ports
    };
    
    /** Vector of all port objects belonging to this node, which are explicitly managed by the node object. */
    std::vector<PortInfo> _all_ports;
    
    /** \brief Meta-function for creating all kinds of input ports supported by this class. */
    int createInputPort(char container, const std::string &element, const std::string &name, bool strict);
    
    /** \brief Meta-function for creating all kinds of output ports supported by this class. */
    int createOutputPort(char container, const std::string &element, const std::string &name);
    
    MQTTNodeExt(const std::string& name, const std::string& ws = ""): MQTTNodeBase(name, ws) {
    }
    
    virtual ~MQTTNodeExt();
    
    
    // === Events between the C++ node and external interface ===
    
    /** The current event (returned by runStep) */
    struct {
        OBNEI_EventType type;
        OBNEI_EventArg arg;
    } _ml_current_event;
    
    bool _ml_pending_event = false;     ///< if there is a external interface event pending
    bool _node_is_stopping = false;     ///< true if the node is going to stop (node's state is already STOPPED but we still need to push the TERM event to external interface)
    
    /** This variable stores the current node event in the event queue. This is because while this node is running, whenever it needs to execute a callback in external interface, which is usually in the middle of an event's execution, it must return to external interface, so later on, when the node is called again, it must resume the current event's execution. Therefore we must save the event object to return to it (to run its post-execution. */
    std::shared_ptr<NodeEvent> _current_node_event;
    
    /** Port events get special treatment. They are queued in a separate queue and are given higher priority than other node events. */
    struct PortEvent {
        enum {
            RCV    // Message received at the port
        } event_type;
        std::size_t port_index; // Index of the port
    };
    shared_queue<PortEvent> m_port_events;
    
    /** \brief Run the node until it stops or until a callback event. */
    int runStep(double timeout);
    
    /** \brief Get the next port event; often used to process port events inside node event callback. */
    std::unique_ptr<PortEvent> getNextPortEvent(double timeout);
    
    /** Override stopSimulation. */
    void stopSimulation() {
        MQTTNodeBase::stopSimulation();
        _ml_pending_event = false;
        _current_node_event.reset();
    }
    
public:
    /* =========== Simulation callbacks =========== */
    
    /** \brief Callback for UPDATE_Y event */
    virtual void onUpdateY(OBNnode::updatemask_t m) override {
        // Post an external interface event for SIM_Y
        _ml_current_event.type = OBNEI_Y;
        _ml_current_event.arg.mask = m;
        _ml_pending_event = true;
        // mexPrintf("UpdateY for node %s mask = %d\n", name().c_str(), m);
    }
    
    /** \brief Callback for UPDATE_X event */
    virtual void onUpdateX(OBNnode::updatemask_t m) override {
        // Post an external interface event for SIM_X
        _ml_current_event.type = OBNEI_X;
        _ml_current_event.arg.mask = m;
        _ml_pending_event = true;
        // mexPrintf("UpdateX for node %s mask = %d\n", name().c_str(), m);
    }
    
    /** \brief Callback to initialize the node before each simulation. */
    virtual void onInitialization() override {
        // Post an external interface event for SIM_INIT
        _ml_current_event.type = OBNEI_INIT;
        _ml_pending_event = true;
        _node_is_stopping = false;
    }
    
    
    /** \brief Callback before the node's current simulation is terminated. */
    virtual void onTermination() override {
        // Post an external interface event for SIM_TERM
        _ml_current_event.type = OBNEI_TERM;
        _ml_pending_event = true;
        _node_is_stopping = true;
    }
    
    /* =========== Callbacks for other events ============ */
    
    /** Callback function for message received event at input ports.
     This callback simply pushes a new port event to the queue for port events.
     This callback is run on the communication thread.
     */
    void extint_inputport_msgrcvd_callback(const std::size_t idx) {
        m_port_events.push(new PortEvent{PortEvent::RCV, idx});
    }
    
    
    /* =========== Callback Methods for errors ============= */
    
    /** Callback for error when parsing the raw binary data into a structured message (e.g. ProtoBuf or JSON) */
    virtual void onRawMessageError(const OBNnode::PortBase * port, const std::string& info) override {
        _node_state = NODE_ERROR;
        auto msg = "Error while parsing the raw message from port: " + port->fullPortName() + " (" + info + ")";
        reportError("MQTTNODE:communication", msg.c_str());
    }
    
    /** Callback for error when reading the values from a structured message (e.g. ProtoBuf or JSON), e.g. if the type or dimension is invalid. */
    virtual void onReadValueError(const OBNnode::PortBase * port, const std::string& info) override {
        _node_state = NODE_ERROR;
        auto msg = "Error while extracting value from message for port: " + port->fullPortName() + " (" + info + ")";
        reportError("MQTTNODE:communication", msg.c_str());
    }
    
    /** Callback for error when sending the values (typically happens when serializing the message to be sent). */
    virtual void onSendMessageError(const OBNnode::PortBase * port, const std::string& info) override {
        _node_state = NODE_ERROR;
        auto msg = "Error while sending a value from port: " + port->fullPortName() + " (" + info + ")";
        reportError("MQTTNODE:communication", msg.c_str());
    }
    
    /** Callback for error interacting with the SMN and openBuildNet system.  Used for serious errors.
     \param msg A string containing the error message.
     */
    virtual void onOBNError(const std::string& msg) override {
        _node_state = NODE_ERROR;
        reportError("MQTTNODE:openBuildNet", msg.c_str());
    }
    
    /** Callback for warning issues interacting with the SMN and openBuildNet system, e.g. an unrecognized system message from the SMN.  Usually the simulation may continue without any serious consequence.
     \param msg A string containing the warning message.
     */
    virtual void onOBNWarning(const std::string& msg) override {
        reportWarning("MQTTNODE:openBuildNet", msg.c_str());
    }
};


#endif /* OBNNODE_EXTMQTT_H_ */
