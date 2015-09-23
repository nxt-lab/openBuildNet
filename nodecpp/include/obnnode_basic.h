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
#include <obnsim_io.pb.h>

#include <Eigen/Core>

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
        
        
        /** \brief Request to establish a connection from a given port to this port.
         
         \param source The full path of the source port.
         \return A pair of the connection result and an optional error message.
         See the message N2SMN:SYS_PORT_CONNECT_ACK for details.
         */
        virtual std::pair<int, std::string> connect_from_port(const std::string& source) = 0;
        
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
        
        virtual std::pair<int, std::string> connect_from_port(const std::string& source) override {
            // Connection to an output port is forbidden
            return std::make_pair(-1, "An output port can't accept an incoming connection.");
        }
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
        class NodeEvent {
        public:
            virtual ~NodeEvent() { }                    // VERY IMPORTANT because NodeEvent will be derived; this is to make sure child classes will be destroyed cleanly
            virtual void executeMain(NodeBase*) = 0;    ///< Main Execution of the event
            virtual void executePost(NodeBase*) { }     ///< Post-Execution of the event
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
        class NodeEventSMN: public NodeEvent {
        protected:
            simtime_t _time;
            bool _hasID;
            int32_t _id;
            
            /** Method to process basic details of an SMN event; should be called in the execute() methods of child classes. */
            void basic_processing(NodeBase* p) {
                p->_current_sim_time = _time;
                if (_hasID) {
                    p->_node_id = _id;
                }
            }
        public:
            /** Populate basic info of an event from an SMN2N message. */
            NodeEventSMN(const OBNSimMsg::SMN2N& msg) {
                _time = msg.time();
                if ((_hasID = msg.has_id())) { _id = msg.id(); }
            }
        };
        
        /** Event class for cosimulation's UPDATE_Y messages. */
        class NodeEvent_UPDATEY: public NodeEventSMN {
            updatemask_t _updates;
        public:
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            NodeEvent_UPDATEY(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                _updates = msg.has_i()?msg.i():0;
            }
        };
        friend NodeEvent_UPDATEY;
        
        /** Event class for cosimulation's UPDATE_X messages. */
        class NodeEvent_UPDATEX: public NodeEventSMN {
            updatemask_t _updates;
        public:
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            NodeEvent_UPDATEX(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                _updates = msg.has_i()?msg.i():0;
            }
        };
        friend NodeEvent_UPDATEX;
        
        /** Event class for cosimulation's INITIALIZE messages. */
        class NodeEvent_INITIALIZE: public NodeEventSMN {
            std::time_t _wallclock;
            simtime_t _timeunit;
            bool _has_wallclock = false;
            bool _has_timeunit = false;
        public:
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
        class NodeEvent_TERMINATE: public NodeEventSMN {
        public:
            virtual void executeMain(NodeBase*) override;
            virtual void executePost(NodeBase*) override;
            NodeEvent_TERMINATE(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_TERMINATE;
        
        
        /** Event class for system's SYS_PORT_CONNECT messages. */
        class NodeEvent_PORT_CONNECT: public NodeEventSMN {
            std::string _myport;
            std::string _otherport;
            bool _valid_msg;  ///< true if the received request message is valid
        public:
            virtual void executeMain(NodeBase*) override;
            
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
        class NodeEventException: public NodeEvent {
            std::exception_ptr m_exception;
        public:
            NodeEventException(std::exception_ptr e): m_exception(e) {
                assert(e);  // the exception should not be NULL
            }
            
            virtual void executeMain(NodeBase*) override;
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
    
    /** Define all details of an update type. */
    struct UpdateType {
        bool enabled = false;   ///< If this update is enabled
        double T_in_us = -1.0;  ///< Sampling time in microseconds, <=0 if non-periodic
        std::string name;       ///< Name of this update (optional)
        
        typedef std::vector<std::string> OUTPUT_LIST;
        OUTPUT_LIST outputs;   ///< List of outputs of this update
        
        typedef std::vector<std::pair<std::string, bool> > INPUT_LIST;
        INPUT_LIST inputs;  ///< List of inputs of this update and their direct-feedthrough property
        
        // Callbacks
        typedef std::function<void ()> UPDATE_CALLBACK;
        UPDATE_CALLBACK y_callback; ///< Callback for UPDATE_Y, initially empty
        UPDATE_CALLBACK x_callback; ///< Callback for UPDATE_X, initially empty
    };
    
    
    /** \brief Main OBN Node class, with support for specifying updates and __info__ port.
     NB is the base class for a node, which must extend the NodeBase class.
     */
    template <typename NB>
    class OBNNodeBase: public NB {
        static_assert(std::is_base_of<NodeBase, NB>::value, "Template parameter NB must extend OBNnode::NodeBase");
    protected:
        /** Vector of all updates in this node. */
        std::vector<UpdateType> m_updates;
        
        template<typename T>
        bool does_port_exists(const T& t_list, const std::string& t_name) const {
            for (const auto p: t_list) {
                if (p.first->getPortName() == t_name) {
                    return true;
                }
            }
            return false;
        }
        
    public:
        OBNNodeBase(const std::string& _name, const std::string& ws = ""): NB(_name, ws) { }
        
        /** Add a new update to the node. */
        int addUpdate(UpdateType::UPDATE_CALLBACK t_ycallback = UpdateType::UPDATE_CALLBACK(), UpdateType::UPDATE_CALLBACK t_xcallback = UpdateType::UPDATE_CALLBACK(), double t_T = -1.0, const UpdateType::INPUT_LIST& t_inputs = UpdateType::INPUT_LIST(), const UpdateType::OUTPUT_LIST& t_outputs = UpdateType::OUTPUT_LIST(), const std::string& t_name = "");
        
        /** Add a new update to the node with given index. */
        int addUpdate(int t_idx, UpdateType::UPDATE_CALLBACK t_ycallback = UpdateType::UPDATE_CALLBACK(), UpdateType::UPDATE_CALLBACK t_xcallback = UpdateType::UPDATE_CALLBACK(), double t_T = -1.0, const UpdateType::INPUT_LIST& t_inputs = UpdateType::INPUT_LIST(), const UpdateType::OUTPUT_LIST& t_outputs = UpdateType::OUTPUT_LIST(), const std::string& t_name = "");
        
        /** Add a new update to the node. */
        int addUpdate(const UpdateType& t_update) {
            return addUpdate(t_update.y_callback, t_update.x_callback,
                             t_update.T_in_us, t_update.inputs, t_update.outputs, t_update.name);
        }
        
        /** Add a new update to the node with given index. */
        int addUpdate(int t_idx, const UpdateType& t_update) {
            return addUpdate(t_idx, t_update.y_callback, t_update.x_callback,
                             t_update.T_in_us, t_update.inputs, t_update.outputs, t_update.name);
        }
        
        /** Remove an update. */
        bool removeUpdate(int t_idx) {
            if (0 <= t_idx && t_idx < m_updates.size()) {
                if (m_updates[t_idx].enabled) {
                    m_updates[t_idx].enabled = false;
                    return true;
                }
            }
            return false;
        }
        
        virtual void onUpdateY(updatemask_t m) override {
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
        
        virtual void onUpdateX(updatemask_t m) override {
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
        
        // Some constants for specifying the update's sampling time
        static constexpr double MILLISECOND = 1e3;
        static constexpr double SECOND = 1e6;
        static constexpr double MINUTE = 60*SECOND;
        static constexpr double HOUR = 60*MINUTE;
        static constexpr double DAY = 24*HOUR;
    };

    
    
    ///////////////////////////////////////////////////////////////////////////////
    // Auxiliary types used in defining template classes for input/output ports
    ///////////////////////////////////////////////////////////////////////////////
    
    // Types of encoding format: predefined ProtoBuf, User-defined ProtoBuf, Binary
    class OBN_PB {};
    class OBN_PB_USER {};
    class OBN_BIN {};
    
    /** Template class to define the ProtoBuf message class for a certain data type */
    template <typename T> struct obn_scalar_PB_message_class;
    template <typename T> struct obn_vector_PB_message_class;
    template <typename T> struct obn_matrix_PB_message_class;
    
    template <> struct obn_scalar_PB_message_class<bool> { using theclass = OBNSimIOMsg::ScalarBool; };
    template <> struct obn_scalar_PB_message_class<int32_t> { using theclass = OBNSimIOMsg::ScalarInt32; };
    template <> struct obn_scalar_PB_message_class<uint32_t> { using theclass = OBNSimIOMsg::ScalarUInt32; };
    template <> struct obn_scalar_PB_message_class<int64_t> { using theclass = OBNSimIOMsg::ScalarInt64; };
    template <> struct obn_scalar_PB_message_class<uint64_t> { using theclass = OBNSimIOMsg::ScalarUInt64; };
    template <> struct obn_scalar_PB_message_class<float> { using theclass = OBNSimIOMsg::ScalarFloat; };
    template <> struct obn_scalar_PB_message_class<double> { using theclass = OBNSimIOMsg::ScalarDouble; };
    
    template <> struct obn_vector_PB_message_class<bool> { using theclass = OBNSimIOMsg::VectorBool; };
    template <> struct obn_vector_PB_message_class<int32_t> { using theclass = OBNSimIOMsg::VectorInt32; };
    template <> struct obn_vector_PB_message_class<uint32_t> { using theclass = OBNSimIOMsg::VectorUInt32; };
    template <> struct obn_vector_PB_message_class<int64_t> { using theclass = OBNSimIOMsg::VectorInt64; };
    template <> struct obn_vector_PB_message_class<uint64_t> { using theclass = OBNSimIOMsg::VectorUInt64; };
    template <> struct obn_vector_PB_message_class<float> { using theclass = OBNSimIOMsg::VectorFloat; };
    template <> struct obn_vector_PB_message_class<double> { using theclass = OBNSimIOMsg::VectorDouble; };
    
    template <> struct obn_matrix_PB_message_class<bool> { using theclass = OBNSimIOMsg::MatrixBool; };
    template <> struct obn_matrix_PB_message_class<int32_t> { using theclass = OBNSimIOMsg::MatrixInt32; };
    template <> struct obn_matrix_PB_message_class<uint32_t> { using theclass = OBNSimIOMsg::MatrixUInt32; };
    template <> struct obn_matrix_PB_message_class<int64_t> { using theclass = OBNSimIOMsg::MatrixInt64; };
    template <> struct obn_matrix_PB_message_class<uint64_t> { using theclass = OBNSimIOMsg::MatrixUInt64; };
    template <> struct obn_matrix_PB_message_class<float> { using theclass = OBNSimIOMsg::MatrixFloat; };
    template <> struct obn_matrix_PB_message_class<double> { using theclass = OBNSimIOMsg::MatrixDouble; };
    
    
    /** \brief Template class for input data as a scalar of a given type. */
    template <typename T>
    class obn_scalar {
    public:
        /** The input data type, e.g. a vector or a scalar or a matrix. */
        using input_data_type = T;
        
        /** Initializer for the input type. */
        static const T init_input_data;
        
        /** The output data type, from variable to encoded message. */
        using output_data_type = T;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_scalar_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.set_value(data);
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_type& data, const PB_message_class& msg) {
            data = msg.value();
            return true;
        }
    };
    template<typename T> const T obn_scalar<T>::init_input_data(0); // = static_cast<T>(0);
    
    
    /** \brief Template class for input data as a vector of a given type, using Eigen library. */
    template <typename T>
    class obn_vector {
    public:
        /** The input data type for reading from an encoded format (e.g. ProtoBuf) into the given type.
         We use Eigen::Map to avoid copying data, i.e. we directly access the internal data of the encoded message,
         therefore the message must be permanent (it can't be a temporary variable that will be destroyed) and
         anytime the message changes, the vector variable must be updated. */
        using input_data_type = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1> >; // const because this is read-only
        
        /** Initializer for the input type. */
        static const input_data_type init_input_data;
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, Eigen::Dynamic, 1>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_vector_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message.
         It works with raw arrays as much as possible because speed is important.
         */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            auto sz = data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            std::copy_n(data.data(), sz, dest->begin());
        }
        
        /** Static function to read data from a ProtoBuf message.
         It works with raw arrays as much as possible because speed is important.
         */
        static bool readPBMessage(input_data_type& data, const PB_message_class& msg) {
            auto sz = msg.value_size();
            new (&data) input_data_type(msg.value().data(), sz);
            
            //            data.resize(sz, 1);   // resize the vector to match the size of msg
            //            if (sz != data.size()) return false;
            //            if (sz > 0) {
            //                auto itfrom = msg.value().begin();
            //                auto itto = data.data();
            //                for (int i = 0; i < sz; ++i) {
            //                    *(itto++) = *(itfrom++);
            //                }
            //            }
            return true;
        }
    };
    template<typename T> const typename obn_vector<T>::input_data_type obn_vector<T>::init_input_data(nullptr, 0);
    
    /** \brief Template class for input data as a dynamic-sized matrix of a given type. */
    template <typename T>
    class obn_matrix {
    public:
        /** The input data type for reading from an encoded format (e.g. ProtoBuf) into the given type.
         We use Eigen::Map to avoid copying data, i.e. we directly access the internal data of the encoded message,
         therefore the message must be permanent (it can't be a temporary variable that will be destroyed) and
         anytime the message changes, the matrix variable must be updated. */
        using input_data_type = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> >;  // const because this is read-only
        
        /** Initializer for the input type. */
        static const input_data_type init_input_data;
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_matrix_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.set_nrows(data.rows());
            msg.set_ncols(data.cols());
            auto sz = data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            std::copy_n(data.data(), sz, dest->begin());
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_type& data, const PB_message_class& msg) {
            auto nrows = msg.nrows();
            auto ncols = msg.ncols();
            auto sz = nrows * ncols;
            if (msg.value_size() < sz) return false;
            
            new (&data) input_data_type(msg.value().data(), nrows, ncols);
            
            //            data.resize(nrows, ncols);   // resize the matrix to match the size of msg
            //            if (data.rows() != nrows || data.cols() != ncols) return false;
            //
            //            if (sz > 0) {
            //                auto itfrom = msg.value().begin();
            //                auto itto = data.data();
            //                for (int i = 0; i < sz; ++i) {
            //                    *(itto++) = *(itfrom++);
            //                }
            //            }
            return true;
        }
    };
    template<typename T> const typename obn_matrix<T>::input_data_type obn_matrix<T>::init_input_data(nullptr, 0, 0);
    
    
    /** \brief Template class for input data as a fixed-length vector of a given type. */
    template <typename T, const std::size_t N>
    class obn_vector_fixed {
    public:
        static_assert(N > 0, "Vector length must be positive.");
        
        /** The input data type: a vector. For fixed size types, to be safe, we use an actual vector, not a Map. This is because the user can access the input port when it hasn't been initialized (by an input message), potentially crashing the program. */
        using input_data_type = Eigen::Matrix<T, N, 1, Eigen::DontAlign>; // Disable alignment to be safe
        
        /** Initializer for the input type. */
        static const std::size_t init_input_data;
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, N, 1, Eigen::DontAlign>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_vector_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            auto dest = msg.mutable_value();
            dest->Resize(N, T());    // resize the field in msg to hold the values
            std::copy_n(data.data(), N, dest->begin());
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_type& data, const PB_message_class& msg) {
            if (msg.value_size() < N) return false;
            std::copy_n(msg.value().begin(), N, data.data());
            return true;
        }
    };
    template<typename T, const std::size_t N> const std::size_t obn_vector_fixed<T, N>::init_input_data = N;
    
    
    /** \brief Template class for input data as a fixed-size matrix of a given type. */
    template <typename T, const std::size_t NR, const std::size_t NC>
    class obn_matrix_fixed {
    public:
        static_assert((NR > 0) && (NC > 0), "Matrix dimensions must be positive.");
        
        /** The input data type: a matrix. For fixed size types, to be safe, we use an actual matrix, not a Map. This is because the user can access the input port when it hasn't been initialized (by an input message), potentially crashing the program. */
        using input_data_type = Eigen::Matrix<T, NR, NC, Eigen::DontAlign>; // Disable alignment to be safe
        
        /** Initializer for the input type. */
        static const input_data_type init_input_data;
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, NR, NC, Eigen::DontAlign>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_matrix_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.set_nrows(NR);
            msg.set_ncols(NC);
            auto sz = data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            std::copy_n(data.data(), sz, dest->begin());
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_type& data, const PB_message_class& msg) {
            if (msg.nrows() != NR || msg.ncols() != NC) return false;
            auto sz = data.size();
            if (msg.value_size() < sz) return false;
            std::copy_n(msg.value().begin(), sz, data.data());
            return true;
        }
    };
    template<typename T, const std::size_t NR, const std::size_t NC> const typename obn_matrix_fixed<T,NR,NC>::input_data_type obn_matrix_fixed<T,NR,NC>::init_input_data(NR, NC);
    
    
    /** This templated type is the wrapper class for the data type, e.g. obn_scalar<D> or obn_vector<D>.
     It defines input_data_type, PB_message_class, and read and write functions.
     */
    template <typename D>
    using OBN_DATA_TYPE_CLASS = typename std::conditional<std::is_arithmetic<D>::value, obn_scalar<D>, D>::type;
    
    
    /** This class allows thread-safe access to the port's value by locking the mutex upon its creation, and unlock the mutex when it's deleted. */
    template <typename V, typename M>
    class LockedAccess {
        M* m_mutex;
        const V* m_value;
        
    public:
        LockedAccess(V* t_value, M* t_mutex): m_mutex(t_mutex), m_value(t_value) {
            m_mutex->lock();
            //std::cout << "Locked\n";
        }
        
        LockedAccess(const LockedAccess& other) = delete;   // no copy constructor, so we force it to move
        LockedAccess() = delete;    // no default
        LockedAccess(LockedAccess&& rvalue) = default;
        
        ~LockedAccess() {
            m_mutex->unlock();
            //std::cout << "Unlocked\n";
        }
        
        const V& operator*() const {
            return *m_value;
        }
        
        const V* operator->() const {
            return m_value;
        }
    };
}


/**
 \param t_idx The desired index of the new update.
 \param t_T The sampling time in microseconds, <= 0 if non-periodic.
 \param t_ycallback Output callback function for UPDATE_Y message of this update.
 \param t_xcallback State callback function for UPDATE_X message of this update.
 \param t_name Optional name of the update.
 \return The index of this update, or an error code: -1 if the index is out of range, -2 if an update already exists at that index, -3 if an input port does not exist, -4 if an output port does not exist.
 */
template <typename NB>
int OBNnode::OBNNodeBase<NB>::addUpdate(int t_idx,
                                    OBNnode::UpdateType::UPDATE_CALLBACK t_ycallback,
                                    OBNnode::UpdateType::UPDATE_CALLBACK t_xcallback,
                                    double t_T,
                                    const OBNnode::UpdateType::INPUT_LIST& t_inputs,
                                    const OBNnode::UpdateType::OUTPUT_LIST& t_outputs,
                                    const std::string& t_name)
{
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
        if (!does_port_exists(this->_input_ports, p.first)) {
            return -3;
        }
    }
    
    for (const auto &p: t_outputs) {
        if (!does_port_exists(this->_output_ports, p)) {
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
template <typename NB>
int OBNnode::OBNNodeBase<NB>::addUpdate(OBNnode::UpdateType::UPDATE_CALLBACK t_ycallback,
                                        OBNnode::UpdateType::UPDATE_CALLBACK t_xcallback,
                                        double t_T,
                                        const OBNnode::UpdateType::INPUT_LIST& t_inputs,
                                        const OBNnode::UpdateType::OUTPUT_LIST& t_outputs,
                                        const std::string& t_name)
{
    
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
        if (!does_port_exists(this->_input_ports, p.first)) {
            return -3;
        }
    }
    
    for (const auto &p: t_outputs) {
        if (!does_port_exists(this->_output_ports, p)) {
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

#endif // OBNNODE_BASIC_H

