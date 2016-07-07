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
#include <atomic>
#include <mutex>
//#include <unordered_map>         // std::unordered_map
#include <forward_list>
#include <functional>
#include <vector>
#include <deque>
#include <exception>

#include <obnsim_basic.h>

#include <obnsim_msg.pb.h>
#include <obnsim_io.pb.h>

#include <Eigen/Core>

namespace OBNnode {
    typedef OBNsim::simtime_t simtime_t;  ///< Simulation time type, as number of nano-seconds from beginning.
    typedef OBNsim::updatemask_t updatemask_t;  ///< Output group mask type: each bit corresponds to one group, so the width of the type is the maximum number of groups.
    
    /** Communication protocol/platform selection. */
    enum CommProtocol {
        COMM_YARP,
        COMM_MQTT
    };
    
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
        virtual void close() { }

        /** Open the port given a full network name. */
        virtual bool open(const std::string& full_name) = 0;
        
        /** Open the port with name obtained from the node object it's attached to.
         MAKE SURE THAT the port is valid, i.e. it is already attached to a valid node object, before calling this function.
         */
        bool open();
        
        friend class NodeBase;
        
    public:
        PortBase(const std::string& t_name): m_name(t_name), m_node(nullptr) { }
        
        // Important that this base class has a virtual destructor for its descendents to be destroyed properly
        virtual ~PortBase() {
            //std::cout << "~PortBase:" << m_name << std::endl;
        }
        
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
        
        
        /** \brief Request to establish a connection from a given port to this port.
         
         \param source The full path of the source port.
         \return A pair of the connection result and an optional error message.
         See the message N2SMN:SYS_PORT_CONNECT_ACK for details.
         */
        virtual std::pair<int, std::string> connect_from_port(const std::string& source) = 0;
    };
    
    /** \brief Base class for an openBuildNet input port.
     
     It supports callbacks when receiving messages.
     */
    class InputPortBase: public PortBase {
    public:
        /** Callback function type for message received events. */
        typedef std::function<void ()> MSGRCV_CALLBACK;
        
        /** \brief Set the callback function for message received events.
         
         \param f The callback function (empty function to disable callback).
         \param onMainThread Set to true if the callback should be called on the main thread (by means of an event posted to the main queue); otherwise, it will be called on the communication thread (hence thread-safety measures must be implemented).
         */
        void setMsgRcvCallback(const MSGRCV_CALLBACK& f, bool onMainThread);
        
        /** \brief Clear the callback function for message received events.
         */
        void clearMsgRcvCallback();
        
        /** \brief Trigger the callback for message received events.
         
         This method won't trigger the callback if the callback is empty; so it's safe to call this method even if the callback has not been set.
         If the callback is set to run on the main thread and this port is associated with a node, an appropriate event will be posted to the node's main thread; if this port is not associated with any node, the trigger will fail (but does not cause any error).
         Otherwise, the callback will be called immediately (on the caller's thread).
         */
        void triggerMsgRcvCallback();
        
        /** \brief Check if there is a pending value / message at the port. */
        virtual bool isValuePending() const = 0;
        
        InputPortBase(const std::string& t_name): PortBase(t_name) { }
        virtual ~InputPortBase();
        
    protected:
        MSGRCV_CALLBACK m_msgrcv_callback{};  ///< The callback function for message received events
        bool m_msgrcv_callback_on_mainthread{true};
        std::mutex m_msgrcv_callback_mutex;     ///< Mutex for accessing the callback function
        
        /* \brief Call the callback function for message received events.
         
         If the callback is empty, it won't be called.
         This method always locks the access to the callback function before calling it, so it can be called directly on the communication thread.
         */
        //void callMsgRcvCallback();
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
        virtual std::pair<int, std::string> connect_from_port(const std::string& source) override {
            // Connection to an output port is forbidden
            return std::make_pair(-1, "An output port can't accept an incoming connection.");
        }

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
        
        /** \brief The name of the node, e.g. "node1". */
        const std::string& name() const {
            return _nodeName;
        }
        
        /** \brief The full name of the node, with workspace prefix, e.g. "powerdemo/node1". */
        std::string full_name() const {
            return _workspace + _nodeName;
        }
        
        /** Return full path to a port in this node. */
        std::string fullPortName(const std::string& portName) const {
            return _workspace + _nodeName + '/' + portName;
        }
        
        /** Returns the time unit, an integer in microseconds. */
        simtime_t timeunit() const {
            return _timeunit;
        }
        
        /** Return the current simulation time.
         This is an integer number of clock ticks from the start of the simulation.
         One clock tick is equivalent to one time unit.
         For example if time unit is 1 millisecond, then a simulation time of 1000 is 1 second.
         See the template   version of this method for a more convenient access to the simulation time using a desired time unit.
         */
        simtime_t currentSimulationTime() const {
            return _current_sim_time;
        }
        
        /** Return the current simulation time as a real value in the given unit.
         The template argument must be a duration type, e.g. std::chrono:seconds.
         Example: currentSimulationTime<std::chrono::seconds>() returns the current simulation time as a real value in seconds.
         */
        template <typename U>
        double currentSimulationTime() const {
            return 1.0e-6 * _current_sim_time * _timeunit * U::period::den / U::period::num;
        }
        
        /** Return the current wallclock time, as UNIX / POSIX time value, rounded downward to a second. */
        int64_t currentWallClockTime() const {
            return static_cast<int64_t>(std::floor( _initial_wallclock + (_timeunit * 1.0e-6) * _current_sim_time ));
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
        virtual bool addInput(InputPortBase* port, bool owned=false);
        
        /** \brief Add an existing (physical) output port to the node. */
        virtual bool addOutput(OutputPortBase* port, bool owned=false);
        
        /** \brief Remove a non-output port from this node, i.e. detach it, without deleting it. */
        virtual void removePort(InputPortBase* port);
        
        /** \brief Remove an output port from this node, i.e. detach it, without deleting it. */
        virtual void removePort(OutputPortBase* port);  // Do not remove this method as it's required to remove output ports.
        
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
        std::forward_list< std::pair<InputPortBase*, bool> > _input_ports;
        
        /** List of physical output ports: the second bool field specifies if the node owns the port object and should delete it when done. */
        std::forward_list< std::pair<OutputPortBase*, bool> > _output_ports;
        
        /** Attach a port object to this node. */
        bool attachAndOpenPort(PortBase * port);
        
        /** Current state of the node. */
        std::atomic<NODE_STATE> _node_state;
        
        /** The ID of the node in the network (assigned by the GC in its messages to the node) */
        int32_t _node_id;
        
        /** Current simulation time, an integer number of clock ticks from the start.
         One tick is equivalent to one time unit (see below). */
        simtime_t _current_sim_time;
        
        /** The initial wallclock time. */
        int64_t _initial_wallclock = 0;
        
        /** The simulation time unit, in microseconds. */
        simtime_t _timeunit = 1;
        
        /** \brief Initialize node for simulation. */
        virtual bool initializeForSimulation();
        
        /** Check the given message against the list of wait-for conditions. */
        virtual void checkWaitForCondition(const OBNSimMsg::SMN2N&) = 0;
        
        /** \brief Run the node (simulation) until a predicate is true, or timeout, or the simnulation stops.
         
         This method should only be called when the node is already running or started; otherwise it will cause an error.
         This method should only be used internally in the node, not to be called from outside; use run() to run the node's simulation from outside.
         
         \param pred The predicate function, must be valid (i.e. callable).
         \param timeout The timeout value, non-positive if there is no timeout.
         \return 0 if the predicate was true; >0 if the simulation stops (because of final simulation time, because of an error...); <0 if timeout occured.
         */
        int runUntil(std::function<bool ()> pred, double timeout = -1.0);
        
        /** \brief Run the node until messages have been received on each and every ports listed in the given array.
         
         At any time, only one instance of this method can be running.
         This method will replace the callback functions of the given ports, and when the event has happened on a port, it will clear the callback function.
         
         \param ports An array of valid pointers to input port objects.
         \param nPorts The number of ports in the array.
         \param timeout The timeout value, non-positive if there is no timeout.
         \param func A optional function to be called after the events have been set up but before the waiting starts, e.g. to send a broadcast message to some nodes.
         \return 0 if the compound event happened; >0 if the simulation stops (because of final simulation time, because of an error...); <0 if timeout occured (-1) or if this method is already running (-2).
         */
        int runUntilMsgRcv(InputPortBase* ports[], std::size_t nPorts, double timeout = -1.0, std::function<void ()> func = nullptr);
        
        std::atomic_bool m_runUntilMsgRcv_running{false};
        std::size_t m_runUntilMsgRcv_counter{0};
        std::vector<bool> m_runUntilMsgRcv_bits;        ///< Bit set for recording the events, accessed from the main thread so no thread-safety measure is needed.
        
        /* ================== Support for Node Events ================= */
    protected:
        /** Event parent class.
         An event object is just an executable object (by calling execute()).
         A child class is created for each type of events, with the appropriate implementation of execute() and data.
         Any specialized event class should be declared as a friend of the node class.
         The event will be executed on the main thread only. Events are posted to the queue by the threads of the ports.
         The execution of the event handler consists of two functions, in the following order: executeMain, executePost.
         */
        class NodeEvent {
        public:
            virtual ~NodeEvent() { }                    // VERY IMPORTANT because NodeEvent will be derived; this is to make sure child classes will be destroyed cleanly
            virtual void executeMain(NodeBase*) { }     ///< Main Execution of the event
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
            int64_t _wallclock;
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
        
        /** Event class for a callback, e.g. a communication event callback, to be called on the main thread. */
        class NodeEventCallback: public NodeEvent {
            std::function<void ()> m_callback_func;
        public:
            NodeEventCallback(std::function<void ()> f): m_callback_func(f) {
                assert(m_callback_func);    // Must contain a callable target
            }
            
            virtual void executeMain(NodeBase*) override;
        };
        
    public:
        /** Post a system openBuildNet event to the end of the queue (from an SMN2N message).
         
         This function is generally called from a callback of the communication library (e.g. Yarp thread or MQTT thread), so consider thread-safety seriously.
         */
        void postEvent(const OBNSimMsg::SMN2N& msg);
        
        
        /* Methods for pushing events of other types (node internal events) will be put here */
        
        /** Create an event of an exception / error at the front of the queue. */
        void postExceptionEvent(std::exception_ptr e) {
            eventqueue_push_front(new NodeEventException(e));
        }
        
        /** Create a callback event, which will call a given function when it's processed, at the front of the queue. */
        void postCallbackEvent(std::function<void ()> f) {
            eventqueue_push_front(new NodeEventCallback(f));
        }
        
        /** Post an empty event in order to wake up the thread. */
        void postWakeupEvent() {
            eventqueue_push(new NodeEvent());
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
        
        /** Callback for permanent communication lost error (e.g. the communication server has shut down).
         \param comm The communication protocol/platform that has lost.
         The node should stop its simulation (if comm is not used by the GC) and exit as cleanly as possible.
         */
        virtual void onPermanentCommunicationLost(CommProtocol comm) = 0;
        
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
    
    const UpdateType::UPDATE_CALLBACK NULL_UPDATE_CALLBACK{};
    
    
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
        int addUpdate(const UpdateType::UPDATE_CALLBACK& t_ycallback = UpdateType::UPDATE_CALLBACK(), const UpdateType::UPDATE_CALLBACK& t_xcallback = UpdateType::UPDATE_CALLBACK(), double t_T = -1.0, const UpdateType::INPUT_LIST& t_inputs = UpdateType::INPUT_LIST(), const UpdateType::OUTPUT_LIST& t_outputs = UpdateType::OUTPUT_LIST(), const std::string& t_name = "");
        
        /** Add a new update to the node with given index. */
        int addUpdate(int t_idx, const UpdateType::UPDATE_CALLBACK& t_ycallback = UpdateType::UPDATE_CALLBACK(), const UpdateType::UPDATE_CALLBACK& t_xcallback = UpdateType::UPDATE_CALLBACK(), double t_T = -1.0, const UpdateType::INPUT_LIST& t_inputs = UpdateType::INPUT_LIST(), const UpdateType::OUTPUT_LIST& t_outputs = UpdateType::OUTPUT_LIST(), const std::string& t_name = "");

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
    
    
    /** \brief Utility structure that manages raw arrays. */
    template <typename T>
    struct raw_array_container {
        // Dafault constructor
        raw_array_container(): m_data(nullptr), m_len(0), m_owned(false) { }
        
        // Copy from const data
        raw_array_container(const T* p, std::size_t n) {
            copy_data(p, n);
        }
        
        // Copy or attach data, depending on "copy"
        raw_array_container(T* p, std::size_t n, bool copy) {
            if (copy) {
                copy_data(p, n);
            } else {
                attach_data(p, n);
            }
        }
        
        // Copy constructor always copy the data
        raw_array_container(const raw_array_container& c) {
            copy_data(c.data(), c.size());
        }
        
        ~raw_array_container() {
            destroy_data();
        }
        
        // Copy new const array to the container
        template<typename InputIter>
        T* assign(InputIter p, std::size_t n) {
            destroy_data();             // destroy current data if needs to
            copy_data(p, n);            // get new data
            return m_data;
        }
        
        // Copy or attach new data
        T* assign(T* p, std::size_t n, bool copy) {
            destroy_data();             // destroy current data if needs to
            if (copy) {
                copy_data(p, n);
            } else {
                attach_data(p, n);
            }
            return m_data;
        }
        
        // Clear data
        void clear() {
            destroy_data();
            m_owned = false;
            m_data = nullptr;
            m_len = 0;
        }
        
        T* data() { return m_data; }
        const T* data() const { return m_data; }
        
        std::size_t size() const { return m_len; }
    private:
        T* m_data;  // pointer to the array
        std::size_t m_len;  // length of the array
        bool m_owned;   // whether the container owns the data (if true, it must delete the pointer when it's not used anymore)

        void attach_data(T* p, std::size_t n) {
            if (p == nullptr || n == 0) {
                m_data = nullptr;
                m_len = 0;
                m_owned = false;
            } else {
                // Just store the pointer
                m_len = n;
                m_data = p;
                m_owned = false;
            }
        }
        
        template <typename InputIter>
        void copy_data(InputIter p, std::size_t n) {
            if (n == 0) {
                m_data = nullptr;
                m_len = 0;
                m_owned = false;
            } else {
                // Copy the data over to our own array
                m_len = n;
                m_data = new T[n];
                m_owned = true;
                std::copy_n(p, n, m_data);
            }
        }
        
        void destroy_data() {
            if (m_owned && m_data != nullptr) {
                delete[] m_data;
            }
        }
    };
    
    /** \brief Template class for input data as a scalar of a given type. */
    template <typename T>
    class obn_scalar {
    public:
        /** The input data type, e.g. a vector or a scalar or a matrix. */
        using input_data_type = T;
        
        /** Container and Initializer for the input type. */
        struct input_data_container {
            using data_type = input_data_type;
            data_type v{0};
        };
        
        /** The output data type, from variable to encoded message. */
        using output_data_type = T;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_scalar_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            // simple message, no need to clear: msg.Clear();
            msg.set_value(data);
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_container& data, const PB_message_class& msg) {
            data.v = msg.value();
            return true;
        }
        
        /** The type for the queue in strict ports. */
        using input_queue_elem_type = input_data_type;
        using input_queue_type = std::deque< input_queue_elem_type >;
        
        /** Static function to read data from a ProtoBuf message to queue. */
        static bool readPBMessageStrict(input_queue_type& data, const PB_message_class& msg) {
            data.push_back(msg.value());
            return true;
        }
    };
    
    
    /** \brief Template class for input data as a vector of a given type, using Eigen library. */
    template <typename T>
    class obn_vector {
    public:
        /** The input data type for reading from an encoded format (e.g. ProtoBuf) into the given type. */
        using input_data_type = Eigen::Matrix<T, Eigen::Dynamic, 1>;
        
        /** Container and Initializer for the input type.
         We use Eigen::Map to avoid copying data, i.e. we directly access the internal data of the encoded message,
         therefore the message must be permanent (it can't be a temporary variable that will be destroyed) and
         anytime the message changes, the vector variable must be updated. */
        struct input_data_container {
            using data_type = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1> >;  // const because this is read-only
            data_type v{nullptr, 0};
        };
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, Eigen::Dynamic, 1>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_vector_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message.
         It works with raw arrays as much as possible because speed is important.
         */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.Clear();
            auto sz = data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            if (sz > 0) {
                std::copy_n(data.data(), sz, dest->begin());
            }
        }
        
        /** Static function to read data from a ProtoBuf message.
         It works with raw arrays as much as possible because speed is important.
         */
        static bool readPBMessage(input_data_container& data, const PB_message_class& msg) {
            auto sz = msg.value_size();
            new (&data.v) typename input_data_container::data_type(msg.value().data(), sz);
            
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
        
        /** The type for the queue in strict ports. */
        using input_queue_elem_type = std::unique_ptr<input_data_type>;
        using input_queue_type = std::deque<input_queue_elem_type>;
        
        /** Static function to read data from a ProtoBuf message to the queue. */
        static bool readPBMessageStrict(input_queue_type& data, const PB_message_class& msg) {
            auto sz = msg.value_size();
            data.emplace_back(new input_data_type(sz, 1));   // create a vector with the right size
            auto& elem = data.back();
            if (sz != elem->size()) {
                data.pop_back();
                return false;
            }
            if (sz > 0) {
                std::copy_n(msg.value().begin(), sz, elem->data());
            }
            return true;
        }
    };
    
    /** \brief Template class for input data as a vector of a given type, using raw array in column-major. */
    template <typename T>
    class obn_vector_raw {
    public:
        /** The input data type for reading from an encoded format (e.g. ProtoBuf) into the given type. */
        using input_data_type = raw_array_container<T>; // for vectors, size() returns the number of elements
        
        /** Container and Initializer for the input type.
         We directly access the internal data of the encoded message if strict = false, otherwise copy the data over,
         therefore the message must be permanent (it can't be a temporary variable that will be destroyed) and
         anytime the message changes, the vector variable must be updated. */
        struct input_data_container {
            using data_type = input_data_type;
            data_type v{nullptr, 0};
        };
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf). */
        using output_data_type = raw_array_container<T>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_vector_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message.
         It works with raw arrays as much as possible because speed is important.
         */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.Clear();
            auto sz = data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            if (sz > 0) {
                std::copy_n(data.data(), sz, dest->begin());
            }
        }
        
        /** Static function to read data from a ProtoBuf message.
         It works with raw arrays as much as possible because speed is important.
         */
        static bool readPBMessage(input_data_container& data, PB_message_class& msg) {
            auto sz = msg.value_size();
            data.v.assign(msg.mutable_value()->begin(), sz, false);
            return true;
        }
        
        /** The type for the queue in strict ports. */
        using input_queue_elem_type = std::unique_ptr<input_data_type>;
        using input_queue_type = std::deque<input_queue_elem_type>;
        
        /** Static function to read data from a ProtoBuf message to the queue. */
        static bool readPBMessageStrict(input_queue_type& data, const PB_message_class& msg) {
            data.emplace_back(new input_data_type(msg.value().data(), msg.value_size()));   // copy the data from msg
            return true;
        }
    };

    
    /** \brief Template class for input data as a dynamic-sized matrix of a given type. */
    template <typename T>
    class obn_matrix {
    public:
        /** The input data type for reading from an encoded format (e.g. ProtoBuf) into the given type. */
        using input_data_type = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
        
        /** Container and Initializer for the input type.
         We use Eigen::Map to avoid copying data, i.e. we directly access the internal data of the encoded message,
         therefore the message must be permanent (it can't be a temporary variable that will be destroyed) and
         anytime the message changes, the matrix variable must be updated. */
        struct input_data_container {
            using data_type = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> >;  // const because this is read-only
            data_type v{nullptr, 0, 0};
        };
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_matrix_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.Clear();
            msg.set_nrows(data.rows());
            msg.set_ncols(data.cols());
            auto sz = data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            if (sz > 0) {
                std::copy_n(data.data(), sz, dest->begin());
            }
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_container& data, const PB_message_class& msg) {
            auto nrows = msg.nrows();
            auto ncols = msg.ncols();
            auto sz = nrows * ncols;
            if (msg.value_size() < sz) return false;
            
            new (&data.v) typename input_data_container::data_type(msg.value().data(), nrows, ncols);
            
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
        
        /** The type for the queue in strict ports. */
        using input_queue_elem_type = std::unique_ptr<input_data_type>;
        using input_queue_type = std::deque<input_queue_elem_type>;
        
        /** Static function to read data from a ProtoBuf message which really copies the data. */
        static bool readPBMessageStrict(input_queue_type& data, const PB_message_class& msg) {
            auto nrows = msg.nrows();
            auto ncols = msg.ncols();
            auto sz = nrows * ncols;
            if (msg.value_size() < sz) return false;
            
            data.emplace_back(new input_data_type(nrows, ncols));   // create a matrix of the given size
            auto& elem = data.back();
            if (elem->rows() != nrows || elem->cols() != ncols) {
                data.pop_back();
                return false;
            }
            
            if (sz > 0) {
                std::copy_n(msg.value().begin(), sz, elem->data());
            }
            return true;
        }
    };
    
    /** \brief Template class for input data as a 2-D matrix of a given type, using raw array in column-major. */
    template <typename T>
    class obn_matrix_raw {
    public:
        /** The input data type for reading from an encoded format (e.g. ProtoBuf) into the given type. */
        struct raw_matrix_data_type {
            // for matrices, we need numbers of rows and columns, and nrows*ncols = data.size()
            std::size_t nrows, ncols;
            raw_array_container<T> data;
            
            // default constructor
            raw_matrix_data_type(): nrows(0), ncols(0), data(nullptr, 0) { }
            
            // constructor that copies data
            raw_matrix_data_type(const T* p, std::size_t trows, std::size_t tcols): nrows(trows), ncols(tcols), data(p, trows*tcols) { }
            
            // constructor that copies or attaches data
            raw_matrix_data_type(T* p, std::size_t trows, std::size_t tcols, bool copy): nrows(trows), ncols(tcols), data(p, trows*tcols, copy) { }
            
            // Copy constructor
            raw_matrix_data_type(const raw_matrix_data_type& c): nrows(c.nrows), ncols(c.ncols), data(c.data) { }
            
            // Clear the data
            void clear() {
                nrows = 0;
                ncols = 0;
                data.clear();
            }
            
            // Copy data
            void copy(const T* p, std::size_t trows, std::size_t tcols) {
                nrows = trows;
                ncols = tcols;
                data.assign(p, trows*tcols);
            }
        };
        
        using input_data_type = raw_matrix_data_type;
        
        /** Container and Initializer for the input type.
         We directly access the internal data of the encoded message if strict = false, otherwise copy the data over,
         therefore the message must be permanent (it can't be a temporary variable that will be destroyed) and
         anytime the message changes, the vector variable must be updated. */
        struct input_data_container {
            using data_type = raw_matrix_data_type;
            data_type v;
        };
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf). */
        using output_data_type = raw_matrix_data_type;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_matrix_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.Clear();
            msg.set_nrows(data.nrows);
            msg.set_ncols(data.ncols);
            auto sz = data.data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            if (sz > 0) {
                std::copy_n(data.data.data(), sz, dest->begin());
            }
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_container& data, PB_message_class& msg) {
            int nrows = msg.nrows();
            int ncols = msg.ncols();
            int sz = nrows * ncols;
            if (msg.value_size() < sz) return false;
            
            data.v.data.assign(msg.mutable_value()->begin(), sz, false);
            data.v.nrows = nrows;
            data.v.ncols = ncols;
            
            return true;
        }
        
        /** The type for the queue in strict ports. */
        using input_queue_elem_type = std::unique_ptr<input_data_type>;
        using input_queue_type = std::deque<input_queue_elem_type>;
        
        /** Static function to read data from a ProtoBuf message which really copies the data. */
        static bool readPBMessageStrict(input_queue_type& data, const PB_message_class& msg) {
            auto nrows = msg.nrows();
            auto ncols = msg.ncols();
            auto sz = nrows * ncols;
            if (msg.value_size() < sz) return false;
            
            data.emplace_back(new input_data_type(msg.value().data(), nrows, ncols));   // create a matrix of the given size and assign data
            return true;
        }
    };
    
    
    /** \brief Template class for input data as a fixed-length vector of a given type. */
    template <typename T, const std::size_t N>
    class obn_vector_fixed {
    public:
        static_assert(N > 0, "Vector length must be positive.");
        
        /** The input data type: a vector. For fixed size types, to be safe, we use an actual vector, not a Map. This is because the user can access the input port when it hasn't been initialized (by an input message), potentially crashing the program. */
        using input_data_type = Eigen::Matrix<T, N, 1, Eigen::DontAlign>; // Disable alignment to be safe
        
        /** Container and Initializer for the input type. */
        struct input_data_container {
            using data_type = input_data_type;
            data_type v;
        };
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, N, 1, Eigen::DontAlign>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_vector_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.Clear();
            auto dest = msg.mutable_value();
            dest->Resize(N, T());    // resize the field in msg to hold the values
            std::copy_n(data.data(), N, dest->begin());
        }
        
        /** The type for the queue in strict ports. */
        using input_queue_elem_type = std::unique_ptr<input_data_type>;
        using input_queue_type = std::deque<input_queue_elem_type>;
        
        /** Static function to read data from a ProtoBuf message which really copies the data. */
        static bool readPBMessageStrict(input_queue_type& data, const PB_message_class& msg) {
            if (msg.value_size() < N) return false;
            data.emplace_back(new input_data_type());   // Create the vector whose size is fixed
            std::copy_n(msg.value().begin(), N, data.back()->data());
            return true;
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_container& data, const PB_message_class& msg) {
            if (msg.value_size() < N) return false;
            std::copy_n(msg.value().begin(), N, data.v.data());
            return true;
        }
    };
    
    
    /** \brief Template class for input data as a fixed-size matrix of a given type. */
    template <typename T, const std::size_t NR, const std::size_t NC>
    class obn_matrix_fixed {
    public:
        static_assert((NR > 0) && (NC > 0), "Matrix dimensions must be positive.");
        
        /** The input data type: a matrix. For fixed size types, to be safe, we use an actual matrix, not a Map. This is because the user can access the input port when it hasn't been initialized (by an input message), potentially crashing the program. */
        using input_data_type = Eigen::Matrix<T, NR, NC, Eigen::DontAlign>; // Disable alignment to be safe
        
        /** Container and Initializer for the input type. */
        struct input_data_container {
            using data_type = input_data_type;
            data_type v;
        };
        
        /** The output data type for writing from the given type to an encoded format (e.g. ProtoBuf).
         We use an Eigen variable as a buffer, and will only write it to the message on demand. */
        using output_data_type = Eigen::Matrix<T, NR, NC, Eigen::DontAlign>;
        
        /** The class type of the ProtoBuf message. */
        using PB_message_class = typename obn_matrix_PB_message_class<T>::theclass;
        
        /** Static function to write data to a ProtoBuf message. */
        static void writePBMessage(const output_data_type& data, PB_message_class& msg) {
            msg.Clear();
            msg.set_nrows(NR);
            msg.set_ncols(NC);
            auto sz = data.size();
            auto dest = msg.mutable_value();
            dest->Resize(sz, T());    // resize the field in msg to hold the values
            if (sz > 0) {
                std::copy_n(data.data(), sz, dest->begin());
            }
        }
        
        /** The element type for the queue in strict ports. */
        using input_queue_elem_type = std::unique_ptr<input_data_type>;
        using input_queue_type = std::deque<input_queue_elem_type>;
        
        /** Static function to read data from a ProtoBuf message which really copies the data. */
        static bool readPBMessageStrict(input_queue_type& data, const PB_message_class& msg) {
            if (msg.nrows() != NR || msg.ncols() != NC || msg.value_size() < NR*NC) return false;
            data.emplace_back(new input_data_type());
            auto& elem = data.back();
            auto sz = elem->size();
            std::copy_n(msg.value().begin(), sz, elem->data());
            return true;
        }
        
        /** Static function to read data from a ProtoBuf message. */
        static bool readPBMessage(input_data_container& data, const PB_message_class& msg) {
            if (msg.nrows() != NR || msg.ncols() != NC) return false;
            auto sz = data.v.size();
            if (msg.value_size() < sz) return false;
            std::copy_n(msg.value().begin(), sz, data.v.data());
            return true;
        }
        
    };
    
    
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
        LockedAccess(V* t_value, M* t_mutex): m_mutex{t_mutex}, m_value{t_value} {
            // Lock the mutex to access the value
            if (m_mutex) m_mutex->lock();
        }
        
        LockedAccess(const LockedAccess&) = delete;   // no copy constructor, so we force it to move
        LockedAccess() = delete;    // no default constructor
        LockedAccess& operator=(const LockedAccess&) = delete;  // no assignment
        LockedAccess& operator=(LockedAccess&&) = delete;  // no move-assignment
        
        // Move constructor that:
        // - Moves all members from rvalue to this object.
        // - Marks the original object (rvalue) as invalid / unused, so that it won't unlock the mutex when it's destroyed.
        LockedAccess(LockedAccess&& rvalue): m_mutex{std::move(rvalue.m_mutex)}, m_value{std::move(rvalue.m_value)}
        {
            rvalue.m_mutex = nullptr;   // NULL mutex pointer means ununsed / invalid
        }
        
        ~LockedAccess() {
            // Only unlock the mutex if the pointer m_mutex is not NULL (i.e., a valid / used LockedAccess object)
            if (m_mutex) {
                m_mutex->unlock();
            }
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
                                    const OBNnode::UpdateType::UPDATE_CALLBACK& t_ycallback,
                                    const OBNnode::UpdateType::UPDATE_CALLBACK& t_xcallback,
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
int OBNnode::OBNNodeBase<NB>::addUpdate(const OBNnode::UpdateType::UPDATE_CALLBACK& t_ycallback,
                                        const OBNnode::UpdateType::UPDATE_CALLBACK& t_xcallback,
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

