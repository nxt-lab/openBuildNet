/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basic definitions for node.cpp framework.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_BASIC_H
#define OBNNODE_BASIC_H

#include <cmath>
#include <iostream>
#include <memory>               // shared_ptr
//#include <unordered_map>         // std::unordered_map
#include <forward_list>
#include <functional>
#include <vector>
#include <exception>

#include <obnsim_basic.h>

#include <obnsim_msg.pb.h>

namespace OBNnode {
    typedef OBNsim::simtime_t simtime_t;  ///< Simulation time type, as number of nano-seconds from beginning.
    typedef OBNsim::updatemask_t updatemask_t;  ///< Output group mask type: each bit corresponds to one group, so the width of the type is the maximum number of groups.
    
    
    class NodeBase;
    
    /** \brief Base class for an openBuildNet port, contains name, mode, etc.
     */
    class PortBase {
    protected:
        std::string m_name; ///< The name of the port in the node (i.e. without the node name prefix
        NodeBase* m_node;   ///< The node managing/owning this port (where event queue and callback interface are handled)
        
        /** Attach the port to make it a valid port */
        virtual bool attach(NodeBase* node) {
            assert(node != nullptr);
            m_node = node;
            return true;
        }
        
        /** Detach the port from its current node, making it invalid. Close the port if necessary. */
        virtual void detach() {
            if (isValid()) {
                close();
                m_node = nullptr;
            }
        }
        
        /** Configure the port (e.g. to set its callback, type of communication, etc.) */
        virtual bool configure() {
            return true;
        }

        /** Close the port. */
        virtual void close() = 0;

        /** Open the port given a full network name. */
        virtual bool open(const std::string& full_name) = 0;
        
        /** Open the port with name obtained from the node object it's attached to.
         MAKE SURE THAT the port is valid, i.e. it is already attached to a valid node object, before calling this function.
         */
        bool open();
        
        friend class NodeBase;
        
    public:
        PortBase(const std::string& t_name): m_name(t_name), m_node(nullptr) { }
        
        virtual ~PortBase();
        
        const std::string& getPortName() const {
            return m_name;
        }
        
        /** \brief Whether the port is valid (i.e. attached to a node and can be used properly)
         */
        bool isValid() const {
            return (m_node != nullptr);
        }
        
        /** \brief Returns the full port name in the communication network (not the name inside the ndoe). */
        virtual std::string fullPortName() const = 0;
    };
    
    /** \brief Base class for an openBuildNet output port.
     */
    class OutputPortBase: public PortBase {
    protected:
        /** Whether the value of this output has been changed (and not yet sent out).
         The subclass should set this variable to true whenever a value is assigned, and set this variable to false whenever it sends the value out.
         */
        bool m_isChanged;
    public:
        OutputPortBase(const std::string& t_name): PortBase(t_name), m_isChanged(false) { }
        virtual ~OutputPortBase();  // This destructor is important to detach the output port from the node. DO NOT REMOVE IT.
        
        bool isChanged() const {
            return m_isChanged;
        }
        
        /** Send the data out in a synchronous manner.
         The function should wait until the data has been sent out successfully and, if ACK is required, all ACKs have been received.
         For asynchronous sending (does not wait until writing is complete and/or all ACKs have been received), \see sendAsync().
         The method does not return the status of the sending (successful or failed), but it may call the error handlers of the node object (which centralize the error handling of each node).
         */
        virtual void sendSync() = 0;
    };
    
    
    /** \brief Base class for a node (in whatever communication framework). */
    class NodeBase {
    public:
        /** Type of the node's state. */
        enum NODE_STATE {
            NODE_STOPPED = 0,   ///< Node hasn't run yet, or has stopped
            NODE_STARTED = 1,   ///< Node has been started (from STOPPED) but not yet running, it must be initialized first
            NODE_RUNNING = 2,   ///< The node is running (simulation is going on)
            NODE_ERROR = 3     ///< The node has had an error, which caused it to stop. The error must be cleared before the node can restart
        };
        
        /** Returns true if the node has had an error causing it to stop. An error must be cleared for the node to restart. */
        bool hasError() const {
            return (_node_state == NODE_ERROR);
        }
        
        /** Clear the error and put the node to STOPPED. */
        void clearError() {
            if (_node_state == NODE_ERROR) {
                _node_state = NODE_STOPPED;
            }
        }
        
        /** \brief Construct a node object. */
        NodeBase(const std::string& _name, const std::string& ws = "");
        virtual ~NodeBase();
        
        const std::string& name() const {
            return _nodeName;
        }
        
        /** Return full path to a port in this node. */
        std::string fullPortName(const std::string& portName) const {
            return _workspace + _nodeName + '/' + portName;
        }
        
        /** Return the current simulation time. */
        simtime_t currentSimulationTime() const {
            return _current_sim_time;
        }
        
        /** Return the current wallclock time, as std::time_t value, rounded downward to a second. */
        std::time_t currentWallClockTime() const {
            return static_cast<std::time_t>(std::floor( _initial_wallclock + (_timeunit * 1.0e-6) * _current_sim_time ));
        }
        
        /** Returns the current state of the node. */
        NODE_STATE nodeState() const {
            return _node_state;
        }
        
        /** \brief Run the node (simulation in the network) */
        void run(double timeout = -1.0);
        
        /** \brief Stop the simulation if it's running. */
        void stopSimulation();
        
        /** \brief Request the SMN/GC to stop the current simulation. */
        void requestStopSimulation();
        
        /** Opens the port on this node to communication with the SMN, if it hasn't been opened.
         \return true if successful.
         */
        virtual bool openSMNPort() = 0;
        
        /* ========== Methods to add ports ============ */
        
        /** \brief Add an existing (physical) input port to the node. */
        bool addInput(PortBase* port, bool owned=false);
        
        /** \brief Add an existing (physical) output port to the node. */
        bool addOutput(OutputPortBase* port, bool owned=false);
        
        /** \brief Remove a non-output port from this node, i.e. detach it, without deleting it. */
        void removePort(PortBase* port);
        
        /** \brief Remove an output port from this node, i.e. detach it, without deleting it. */
        void removePort(OutputPortBase* port);  // Do not remove this method as it's required to remove output ports.
        
        /** \brief Delay by a short amount of time before shutting down the node.
         
         When a simulation system has too many nodes/ports (hundreds) which all terminate at approximately the same time
         the nameserver can be overloaded with requests to close ports from all the nodes. This may cause some ports fail
         to close.
         
         A simple, but not every elegant, solution is to make each node delay for different short amounts of time before
         shutting down, hence reducing the load on the nameserver.
         This function implements a mechanism for such a delay.
         There are several ways to determine the delay amount of each node, e.g. randomly.
         In this implementation, a simple mechanism is used, which delays a node accordingly to its node ID
         (with some rounding).
         */
        void delayBeforeShutdown() const;

    protected:
        /** A local N2SMN message to be sent from the node to the SMN. */
        OBNSimMsg::N2SMN _n2smn_message;
        
        /** Send the current message in _n2smn_message via the GC port. */
        virtual void sendN2SMNMsg() = 0;
        
        /** Convenient methods to send an ACK message to the SMN. */
        void sendACK(OBNSimMsg::N2SMN::MSGTYPE type);
        void sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I);
        
        /** Name of the node. */
        std::string _nodeName;
        
        /** The workspace, must either '/' or of the form "/workspace/".
         The workspace is prepended to all names in this node. */
        std::string _workspace;
        
        /** List of physical input ports: the second bool field specifies if the node owns the port object and should delete it when done. */
        std::forward_list< std::pair<PortBase*, bool> > _input_ports;
        
        /** List of physical output ports: the second bool field specifies if the node owns the port object and should delete it when done. */
        std::forward_list< std::pair<OutputPortBase*, bool> > _output_ports;
        
        /** Attach a port object to this node. */
        bool attachAndOpenPort(PortBase * port);
        
        /** Current state of the node. */
        NODE_STATE _node_state;
        
        /** The ID of the node in the network (assigned by the GC in its messages to the node) */
        int32_t _node_id;
        
        /** Current simulation time. */
        simtime_t _current_sim_time;
        
        /** The initial wallclock time. */
        std::time_t _initial_wallclock = 0;
        
        /** The simulation time unit, in microseconds. */
        simtime_t _timeunit = 1;
        
        /** \brief Initialize node for simulation. */
        void initializeForSimulation();
        
        /** Check the given message against the list of wait-for conditions. */
        virtual void checkWaitForCondition(const OBNSimMsg::SMN2N&) = 0;
        
        
        /* ================== Support for Node Events ================= */
    protected:
        /** Event abstract class.
         An event object is just an executable object (by calling execute()).
         A child class is created for each type of events, with the appropriate implementation of execute() and data.
         Any specialized event class should be declared as a friend of the node class.
         The event will be executed on the main thread only. Events are posted to the queue by the threads of the ports.
         The execution of the event handler consists of two functions, in the following order: executeMain, executePost.
         */
        struct NodeEvent {
            virtual void executeMain(NodeBase*) = 0;     ///< Main Execution of the event
            virtual void executePost(NodeBase*) = 0;     ///< Post-Execution of the event
        };
        
        /* An implementation of a node should manage a queue of NodeEvent objects, and implements the following methods. */
        
        /** \brief Push an event object to the event queue.
         
         This function must be implemented in a thread-safe manner w.r.t. the communication library used because it's usually called in a callback of the communication library (e.g. a Yarp thread or MQTT thread).
         */
        virtual void eventqueue_push(NodeEvent *) = 0;
        
        /** \brief Push an event object to the front of the event queue.
         
         This function must be implemented in a thread-safe manner w.r.t. the communication library used because it's usually called in a callback of the communication library (e.g. a Yarp thread or MQTT thread).
         */
        virtual void eventqueue_push_front(NodeEvent *) = 0;
        
        /** \brief Wait until an event exists in the queue and pop it; may wait forever.
         
         This function is called from the main thread, not from the communication callback.
         */
        virtual std::shared_ptr<NodeEvent> eventqueue_wait_and_pop() = 0;
        
        /** \brief Wait until an event exists in the queue and pop it; may time out.
         
         \param timeout In seconds.
         This function is called from the main thread, not from the communication callback.
         */
        virtual std::shared_ptr<NodeEvent> eventqueue_wait_and_pop(double timeout) = 0;
        
        /** Parent event class for SMN events. */
        struct NodeEventSMN: public NodeEvent {
            simtime_t _time;
            bool _hasID;
            int32_t _id;
            
            /** Populate basic info of an event from an SMN2N message. */
            NodeEventSMN(const OBNSimMsg::SMN2N& msg) {
                _time = msg.time();
                if ((_hasID = msg.has_id())) { _id = msg.id(); }
            }
            
            /** Method to process basic details of an SMN event; should be called in the execute() methods of child classes. */
            void basic_processing(NodeBase* p) {
                p->_current_sim_time = _time;
                if (_hasID) {
                    p->_node_id = _id;
                }
            }
        };
        
        /** Event class for cosimulation's UPDATE_Y messages. */
        struct NodeEvent_UPDATEY: public NodeEventSMN {
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            updatemask_t _updates;
            NodeEvent_UPDATEY(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                _updates = msg.has_i()?msg.i():0;
            }
        };
        friend NodeEvent_UPDATEY;
        
        /** Event class for cosimulation's UPDATE_X messages. */
        struct NodeEvent_UPDATEX: public NodeEventSMN {
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            updatemask_t _updates;
            NodeEvent_UPDATEX(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                _updates = msg.has_i()?msg.i():0;
            }
        };
        friend NodeEvent_UPDATEX;
        
        /** Event class for cosimulation's INITIALIZE messages. */
        struct NodeEvent_INITIALIZE: public NodeEventSMN {
            std::time_t _wallclock;
            simtime_t _timeunit;
            bool _has_wallclock = false;
            bool _has_timeunit = false;
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            NodeEvent_INITIALIZE(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                if (msg.has_data()) {
                    if ((_has_wallclock = msg.data().has_t())) {
                        _wallclock = msg.data().t();
                    }
                    if ((_has_timeunit = msg.has_i())) {
                        _timeunit = msg.i();
                    }
                }
            }
        };
        friend NodeEvent_INITIALIZE;
        
        /** Event class for cosimulation's TERMINATE messages. */
        struct NodeEvent_TERMINATE: public NodeEventSMN {
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            NodeEvent_TERMINATE(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_TERMINATE;
        
        
        /** Event class for system's SYS_PORT_CONNECT messages. */
        struct NodeEvent_PORT_CONNECT: public NodeEventSMN {
            std::string _myport;
            std::string _otherport;
            bool _valid_msg;  ///< true if the received request message is valid
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            NodeEvent_PORT_CONNECT(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                // Extract the names of the ports
                _valid_msg = msg.has_data() && msg.data().has_i() && msg.data().has_b();
                if (_valid_msg) {
                    const std::string& names = msg.data().b();
                    auto i = msg.data().i();
                    if ((_valid_msg = (i > 0 && names.length() > i))) {
                        _myport.assign(names, 0, i);
                        _otherport.assign(names, i, std::string::npos);
                    }
                }
            }
        };
        friend NodeEvent_PORT_CONNECT;
        
        
        /** Event class for any exception (error) thrown anywhere in the program but must be caught by the main thread. */
        struct NodeEventException: public NodeEvent {
            std::exception_ptr m_exception;
            
            NodeEventException(std::exception_ptr e): m_exception(e) {
                assert(e);  // the exception should not be NULL
            }
            
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override { }
        };
        
        
    public:
        /** Post a system openBuildNet event to the end of the queue (from an SMN2N message).
         
         This function is generally called from a callback of the communication library (e.g. Yarp thread or MQTT thread).
         */
        void postEvent(const OBNSimMsg::SMN2N& msg);
        
        
        /* Methods for pushing events of other types (node internal events) will be put here */
        void postExceptionEvent(std::exception_ptr e) {
            eventqueue_push_front(new NodeEventException(e));
        }
        
    public:
        /* =========== Simulation callbacks =========== */
        
        /** \brief Callback for UPDATE_Y event
         
         This callback is called whenever the node receives an UPDATE_Y event.
         A node class should always override this callback and implement the node's computation for updating its outputs.
         The default callback is empty. The current simulation time can be obtained from the node object.
         There is a mechanism to define updates using template methods, which can be used instead of directly implementing this callback.
         Typcially values assigned to output ports in this callback are not sent immediately, but will be sent after this callback finishes. So an output port can be set multiple times in this callback without causing communication overheads. If you want to send a message immediately, use a generic data port or use an appropriate method to send messages from output ports.
         \param m The update type mask.
         */
        virtual void onUpdateY(updatemask_t m) { }
        
        /** \brief Callback for UPDATE_X event
         
         This callback is called whenever the node receives an UPDATE_X event.
         If a node should update its state using the UPDATE_X event, its class should override this callback and implement the node's state update.
         The default callback is empty. The current simulation time can be obtained from the node object.
         \param m The update type mask.
         */
        virtual void onUpdateX(updatemask_t m) { }
        
        /** \brief Callback to initialize the node before each simulation.
         
         This callback is called when the node is initialized before each simualtion (when it receives the INITIALIZE message from the SMN. Note that during the life of a node process, it can be simulated multiple times (by restarting the simulation), each time this callback will be called. This callback is often used to initialize the node's state, outputs, etc.
         After this callback finishes, all output ports will send out their current values, so if the node needs to initialize its outputs, it should do so in this callback.
         */
        virtual void onInitialization() { }
        
        
        /** \brief Callback before the node's current simulation is terminated.
         
         This callback is called when the node's current simulation is about to be terminated (after it has received the TERMINATE message from the SMN).
         */
        virtual void onTermination() { }
        
        /* =========== Callback Methods for errors ============= */
        
        /** Callback for error when parsing the raw binary data into a structured message (e.g. ProtoBuf or JSON) */
        virtual void onRawMessageError(const PortBase * port, const std::string& info) {
            // Report error and try to exit gracefully
            std::cerr << "ERROR: error while parsing the raw message from port " << port->fullPortName() << ": "
            << info << std::endl;
            
            stopSimulation();
        }
        
        /** Callback for error when reading the values from a structured message (e.g. ProtoBuf or JSON), e.g. if the type or dimension is invalid. */
        virtual void onReadValueError(const PortBase * port, const std::string& info) {
            // Report error and try to exit gracefully
            std::cerr << "ERROR: error while reading the value from port " << port->fullPortName() << ": "
            << info << std::endl;
            
            stopSimulation();
        }
        
        /** Callback for error when sending the values (typically happens when serializing the message to be sent). */
        virtual void onSendMessageError(const PortBase * port, const std::string& info) {
            // Report error and try to exit gracefully
            std::cerr << "ERROR: error while sending to port " << port->fullPortName() << ": "
            << info << std::endl;
            
            stopSimulation();
        }
        
        /** Callback for timeout error when running the node's simulation.
         \sa run() with timeout.
         */
        virtual void onRunTimeout() {
            // Currently doing nothing
        }
        
        /** Callback for error interacting with the SMN and openBuildNet system.  Used for serious errors.
         \param msg A string containing the error message.
         */
        virtual void onOBNError(const std::string& msg) {
            // Report error and try to exit gracefully
            std::cerr << "ERROR: OpenBuildNet system error: " << msg << std::endl;
            
            stopSimulation();
        }
        
        /** Callback for warning issues interacting with the SMN and openBuildNet system, e.g. an unrecognized system message from the SMN.  Usually the simulation may continue without any serious consequence.
         \param msg A string containing the warning message.
         */
        virtual void onOBNWarning(const std::string& msg) {
            std::cout << "WARNING: OpenBuildNet system warning: " << msg << std::endl;
        }
        
        /* =========== Misc callbacks ============= */
        
        /** Callback to report information (e.g. when a simulation start, when a simulation stops). */
        virtual void onReportInfo(const std::string& msg) {
            std::cout << msg << std::endl;
        }
    };
}


#endif // OBNNODE_BASIC_H

