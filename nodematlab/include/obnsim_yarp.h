/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnsim_yarp.h
 * \brief MEX interface for YarpNode of the openBuildNet simulation framework.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSIM_YARP_H_
#define OBNSIM_YARP_H_


#include <cstdio>       // printf
#include <vector>

#include <obnnode.h>    // node.C++ framework


/* IMPORTANT NOTE:
 *
 * Because of the tight inter-dependency between a node object and its port objects, and the way Matlab's MEX works, care must be taken to make sure that when a node is destroyed, the port objects are also be destroyed and not orphaned.
 * For this reason, unlike the node.C++ framework, the node object will manage all its port objects explicitly, i.e. port objects are not managed by Matlab at all.
 * The node object will provide an interface for accessing the port objects managed by it, indirectly through some kind of IDs, not via pointers.
 */

/// Report an error and (usually) terminate.
void reportError(const char* msgID, const char* msg) {
    mexErrMsgIdAndTxt(msgID, msg);
}


/// Report a warning
void reportWarning(const char* msgID, const char* msg) {
    mexWarnMsgIdAndTxt(msgID, msg);
}

namespace OBNnode {
    /** The main node class for Matlab. */
    class YarpNodeMatlab: public OBNnode::YarpNodeBase {
    public:
        
        /** Structure containing info about a port in this node. */
        struct PortInfo {
            OBNnode::YarpPortBase *port;
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
        
        YarpNodeMatlab(const std::string& name, const std::string& ws = ""): YarpNodeBase(name, ws) {
        }
        
        virtual ~YarpNodeMatlab();
        
        
        // === Events between the C++ node and Matlab ===
        
        /** Type to pass arguments of an event */
        union MLEventArg {
            OBNnode::updatemask_t mask;
        };
        
        /** The event type */
        enum MLEventType {
            MLE_INIT = OBNSimMsg::SMN2N_MSGTYPE_SIM_INIT,
            MLE_Y = OBNSimMsg::SMN2N_MSGTYPE_SIM_Y,
            MLE_X = OBNSimMsg::SMN2N_MSGTYPE_SIM_X,
            MLE_TERM = OBNSimMsg::SMN2N_MSGTYPE_SIM_TERM
        };
        
        /** The current event (returned by runStep) */
        struct {
            MLEventType type;
            MLEventArg arg;
        } _ml_current_event;
        
        bool _ml_pending_event = false;     ///< if there is a Matlab event pending
        bool _node_is_stopping = false;     ///< true if the node is going to stop (node's state is already STOPPED but we still need to push the TERM event to Matlab)
        
        /** This variable stores the current node event in the event queue. This is because while this node is running, whenever it needs to execute a callback in Matlab, which is usually in the middle of an event's execution, it must return to Matlab, so later on, when the node is called again, it must resume the current event's execution. Therefore we must save the event object to return to it (to run its post-execution. */
        std::shared_ptr<NodeEvent> _current_node_event;
        
        /** \brief Run the node until it stops or until a callback event. */
        int runStep(double timeout);
        
        /** Override stopSimulation. */
        void stopSimulation() {
            YarpNodeBase::stopSimulation();
            _ml_pending_event = false;
            _current_node_event.reset();
        }
        
    public:
        /* =========== Simulation callbacks =========== */
        
        /** \brief Callback for UPDATE_Y event */
        virtual void onUpdateY(updatemask_t m) override {
            // Post a Matlab event for SIM_Y
            _ml_current_event.type = MLE_Y;
            _ml_current_event.arg.mask = m;
            _ml_pending_event = true;
            // mexPrintf("UpdateY for node %s mask = %d\n", name().c_str(), m);
        }
        
        /** \brief Callback for UPDATE_X event */
        virtual void onUpdateX(updatemask_t m) override {
            // Post a Matlab event for SIM_X
            _ml_current_event.type = MLE_X;
            _ml_current_event.arg.mask = m;
            _ml_pending_event = true;
            // mexPrintf("UpdateX for node %s mask = %d\n", name().c_str(), m);
        }
        
        /** \brief Callback to initialize the node before each simulation. */
        virtual void onInitialization() override {
            // Post a Matlab event for SIM_INIT
            _ml_current_event.type = MLE_INIT;
            _ml_pending_event = true;
            _node_is_stopping = false;
        }
        
        
        /** \brief Callback before the node's current simulation is terminated. */
        virtual void onTermination() override {
            // Post a Matlab event for SIM_TERM
            _ml_current_event.type = MLE_TERM;
            _ml_pending_event = true;
            _node_is_stopping = true;
        }
        
        /* =========== Callback Methods for errors ============= */
        
        /** Callback for error when parsing the raw binary data into a structured message (e.g. ProtoBuf or JSON) */
        virtual void onRawMessageError(const YarpPortBase * port, const std::string& info) override {
            _node_state = NODE_ERROR;
            auto msg = "Error while parsing the raw message from port: " + port->fullPortName() + " (" + info + ")";
            reportError("YARPNODE:communication", msg.c_str());
        }
        
        /** Callback for error when reading the values from a structured message (e.g. ProtoBuf or JSON), e.g. if the type or dimension is invalid. */
        virtual void onReadValueError(const YarpPortBase * port, const std::string& info) override {
            _node_state = NODE_ERROR;
            auto msg = "Error while extracting value from message for port: " + port->fullPortName() + " (" + info + ")";
            reportError("YARPNODE:communication", msg.c_str());
        }
        
        /** Callback for error when sending the values (typically happens when serializing the message to be sent). */
        virtual void onSendMessageError(const YarpPortBase * port, const std::string& info) override {
            _node_state = NODE_ERROR;
            auto msg = "Error while sending a value from port: " + port->fullPortName() + " (" + info + ")";
            reportError("YARPNODE:communication", msg.c_str());
        }
        
        /** Callback for error interacting with the SMN and openBuildNet system.  Used for serious errors.
         \param msg A string containing the error message.
         */
        virtual void onOBNError(const std::string& msg) override {
            _node_state = NODE_ERROR;
            reportError("YARPNODE:openBuildNet", msg.c_str());
        }
        
        /** Callback for warning issues interacting with the SMN and openBuildNet system, e.g. an unrecognized system message from the SMN.  Usually the simulation may continue without any serious consequence.
         \param msg A string containing the warning message.
         */
        virtual void onOBNWarning(const std::string& msg) override {
            reportWarning("YARPNODE:openBuildNet", msg.c_str());
        }
    };
}


#endif /* OBNSIM_YARP_H_ */