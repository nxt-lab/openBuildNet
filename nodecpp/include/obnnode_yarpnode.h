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

#ifndef OBNNODE_YARPNODE_H_
#define OBNNODE_YARPNODE_H_

#include <iostream>
#include <memory>               // shared_ptr
//#include <unordered_map>         // std::unordered_map
#include <forward_list>
#include <functional>

#include <obnnode_basic.h>
#include <obnnode_yarpportbase.h>
#include <obnsim_msg.pb.h>

#include <sharedqueue_yarp.h>


namespace OBNnode {
    
    /* ============ YARP Node (managing Yarp ports) Interface ===============*/
    class YarpNode {
    public:
        static yarp::os::Network yarp_network;      ///< The Yarp network object for Yarp's API
        
        /** Type of the node's state. */
        enum NODE_STATE {
            NODE_STOPPED,   ///< Node hasn't run yet, or has stopped
            NODE_STARTED,   ///< Node has been started (from STOPPED) but not yet running, it must be initialized first
            NODE_RUNNING,   ///< The node is running (simulation is going on)
            NODE_ERROR      ///< The node has had an error, which caused it to stop. The error must be cleared before the node can restart
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
        YarpNode(const std::string& _name, const std::string& ws = "");
        virtual ~YarpNode();
        
        
        /** Return full path to a port in this node. */
        std::string fullPortName(const std::string& portName) const {
            return _workspace + _nodeName + '/' + portName;
        }
        
        /** Opens the port on this node to communication with the SMN, if it hasn't been opened.
         \return true if successful.
         */
        bool openSMNPort();
        
        /** Connect the SMN port on this node to the SMN.
         The connection is started from this node, however it can also be started from the SMN.
         If the port is not yet opened, it will be opened first.
         \param carrier Optional parameter to specify the carrier for the connections.
         \return true if successful.
         */
        bool connectWithSMN(const char *carrier = nullptr);
        
        /** Returns the current state of the node. */
        NODE_STATE nodeState() const {
            return _node_state;
        }
        
        /** Run the node (simulation in the network) */
        void run(double timeout = -1.0);
        
        /* ========== Methods to add ports ============ */
        
        /** \brief Add an existing (physical) input port to the node. */
        bool addInput(YarpPortBase* port);
        
        /** \brief Create and add a new (physical) input port to the node. */
        
        
        /** \brief Add an existing (physical) output port to the node. */
        bool addOutput(YarpOutputPortBase* port);
        
        /** \brief Create and add a new (physical) output port to the node. */
        
        /** \brief Add an existing data port (not managed by GC) to the node. */
        
        /** \brief Remove a port (of any type) from this node, i.e. detach it. */
        void removePort(YarpPortBase* port);
        void removePort(YarpOutputPortBase* port);
        
        /* Create a Yarp port managed by this manager.
         \param name Name of the new port, must be a valid and unique name.
         \param callback true [default] if the port uses callback (so that events are caught); if false, no events will be generated for this port.
         \param strict Reading of this port is strict or not (default: false)
         \return The ID of the new port, an unsigned integer.
         */
        //unsigned int createPort(const std::string& name, bool callback=true, bool strict=false);
        
    protected:
        /** A YARP message for communicating with the SMN. */
        typedef YARPMsgPB<OBNSimMsg::N2SMN, OBNSimMsg::SMN2N> SMNMsg;
        
        /** The SMN port on a node to communicate with the SMN. */
        class SMNPort: public yarp::os::BufferedPort<SMNMsg> {
            YarpNode* _the_node;                ///< Pointer to the node to which this port is attached
            OBNSimMsg::SMN2N _smn_message;      ///< The SMN2N message received at the node
            virtual void onRead(SMNMsg& b);
        public:
            SMNPort(YarpNode* node) {
                assert(node);
                _the_node = node;
            }
        };
        
        /** A local N2SMN message to be sent from the node to the SMN. */
        OBNSimMsg::N2SMN _n2smn_message;
        
        /** Convenient methods to send an ACK message to the SMN. */
        void sendACK(OBNSimMsg::N2SMN::MSGTYPE type);
        void sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I);
        void sendACK(OBNSimMsg::N2SMN::MSGTYPE type, int64_t I, int64_t IX);
        
        /** Name of the node. */
        std::string _nodeName;
        
        /** The workspace, must either '/' or of the form "/workspace/".
         The workspace is prepended to all names in this node. */
        std::string _workspace;

        /** List of physical input ports. */
        std::forward_list<YarpPortBase*> _input_ports;
        
        /** List of physical output ports. */
        std::forward_list<YarpOutputPortBase*> _output_ports;
        
        /** The Global Clock port to communicate with the SMN. */
        SMNPort _smn_port;
        
        /** Attach a port object to this node. */
        bool attachAndOpenPort(YarpPortBase * port);
        
        /** Current state of the node. */
        NODE_STATE _node_state;
        
        /** The ID of the node in the network (assigned by the GC in its messages to the node) */
        int32_t _node_id;
        
        /** Current simulation time. */
        simtime_t _current_sim_time;
        
        /** The current update type mask, of the latest UPDATE_Y. */
        updatemask_t _current_updates;
        
        
        
        /* ================== Support for asynchronuous waiting for conditions ================== */
        /* We use a vector of fields: bool inuse, a semaphore, and a function object std::function<bool (...)>.
         * The function object can be assigned to a function or lambda (most of the cases) or a functor (if memory is needed).
         * We will not create and delete condition objects all the time. We create new conditions in the list/vector but do not delete them. Instead, reuse them with inuse (= true if being used, = false if not and can be reused now), so as to minimize the number of creating/deleting objects => much better for memory. Only create new object when no one can be reused.
         */
    public:
        class WaitForCondition {
            /** Status of the condition: active (waiting for), cleared (but not yet fetched), inactive (can be reused) */
            enum { ACTIVE, CLEARED, INACTIVE } status;
            yarp::os::Event _event;     ///< The event condition to wait on
            OBNSimMsg::MSGDATA _data;   ///< The data record (if available) of the message that cleared the condition
            /** Returns true if the condition is cleared. */
            typedef std::function<bool (const OBNSimMsg::SMN2N&)> the_checker;
            the_checker _check_func;    ///< The function to check for the condition

            friend class YarpNode;
            
            /** Make the condition inactive, to be reused later. */
            void reset() {
                status = INACTIVE;
                _event.reset();
                _data.Clear();
            }
        public:
            template<typename F>
            WaitForCondition(F f): status(ACTIVE), _event(false), _check_func(f) { }
            
            // virtual ~WaitForCondition() { std::cout << "~WaitForCondition" << std::endl; }

            /** Wait (blocking) for the condition to hold. */
            void wait() { _event.wait(); }
            
            /** Return the data of the message that cleared the condition. Make sure that the condition was cleared before accessing the data. */
            const OBNSimMsg::MSGDATA& getData() const { return _data; }
        };
        
        /** Check if an wait-for condition is cleared. */
        bool isWaitForCleared(const WaitForCondition& c) {
            yarp::os::LockGuard lock(_waitfor_conditions_mutex);
            return c.status == WaitForCondition::CLEARED;
        }
        
        /** Reset wait-for condition. If the condition isn't reset implicitly, it should be reset by calling this function after it was cleared and its result has been used. */
        void resetWaitFor(WaitForCondition& c) {
            yarp::os::LockGuard lock(_waitfor_conditions_mutex);
            c.reset();
        }

        /** \brief Request a future irregular update from the Global Clock. */
        WaitForCondition* requestFutureUpdate(simtime_t t, updatemask_t m, bool waiting=true);
        
        /** \brief Get the result of a pending request for a future update. */
        int64_t resultFutureUpdate(WaitForCondition*);
        
        /** \brief Wait until a wait-for condition cleared and returns its I field. */
        int64_t resultWaitForCondition(WaitForCondition*);
        
    protected:
        std::forward_list<WaitForCondition> _waitfor_conditions;
        yarp::os::Mutex _waitfor_conditions_mutex;
        
        /** Check the given message against the list of wait-for conditions. */
        void checkWaitForCondition(const OBNSimMsg::SMN2N&);


        /* ================== Support for Node Events ================= */
    protected:
        /** Event abstract class.
         An event object is just an executable object (by calling execute()).
         A child class is created for each type of events, with the appropriate implementation of execute() and data.
         Any specialized event class should be declared as a friend of the node class.
         The event will be executed on the main thread only. Events are posted to the queue by the threads of the ports.
         */
        struct NodeEvent {
            virtual void execute(YarpNode*) = 0;     ///< Execute the event
        };
        
        /** The event queue, which contains smart pointers to event objects. */
        shared_queue<NodeEvent> _event_queue;
        
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
            void basic_processing(YarpNode* p) {
                p->_current_sim_time = _time;
                if (_hasID) {
                    p->_node_id = _id;
                }
            }
        };
        
        /** Event class for cosimulation's UPDATE_Y messages. */
        struct NodeEvent_UPDATEY: public NodeEventSMN {
            virtual void execute(YarpNode*);
            updatemask_t _updates;
            NodeEvent_UPDATEY(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                _updates = (msg.has_data() && msg.data().has_i())?msg.data().i():0;
            }
        };
        friend NodeEvent_UPDATEY;
        
        /** Event class for cosimulation's UPDATE_X messages. */
        struct NodeEvent_UPDATEX: public NodeEventSMN {
            virtual void execute(YarpNode*);
            NodeEvent_UPDATEX(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_UPDATEX;
        
        /** Event class for cosimulation's INITIALIZE messages. */
        struct NodeEvent_INITIALIZE: public NodeEventSMN {
            virtual void execute(YarpNode*);
            NodeEvent_INITIALIZE(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_INITIALIZE;
        
        /** Event class for cosimulation's TERMINATE messages. */
        struct NodeEvent_TERMINATE: public NodeEventSMN {
            virtual void execute(YarpNode*);
            NodeEvent_TERMINATE(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_TERMINATE;
        
    public:
        /** Post a system openBuildNet event to the end of the queue (from an SMN2N message). */
        void postEvent(const OBNSimMsg::SMN2N& msg);
        
        /* Methods for pushing events of other types (node internal events) will be put here */
   
        
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
         The default callback is empty. The current simulation time can be obtained from the node object. The last update type mask (see UPDATE_Y events) can be obtained from the object.
         */
        virtual void onUpdateX() { }
        
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
        virtual void onRawMessageError(YarpPortBase * port) {
            // Currently doing nothing
        }
        
        /** Callback for error when reading the values from a structured message (e.g. ProtoBuf or JSON), e.g. if the type or dimension is invalid. */
        virtual void onReadValueError(YarpPortBase * port) {
            // Currently doing nothing
        }
        
        /** Callback for error when sending the values (typically happens when serializing the message to be sent). */
        virtual void onSendMessageError(YarpOutputPortBase * port) {
            // Currently doing nothing
        }
        
        /** Callback for error interacting with the SMN and openBuildNet system.  Used for serious errors.
         \param msg A string containing the error message.
         */
        virtual void onOBNError(const std::string& msg) {
            
        }
        
        /** Callback for warning issues interacting with the SMN and openBuildNet system, e.g. an unrecognized system message from the SMN.  Usually the simulation may continue without any serious consequence.
         \param msg A string containing the warning message.
         */
        virtual void onOBNWarning(const std::string& msg) {
            
        }
        
        /* =========== Misc callbacks ============= */
        
        /** Callback to report information (e.g. when a simulation start, when a simulation stops). */
        virtual void onReportInfo(const std::string& msg) {
            std::cout << msg << std::endl;
        }
    };
    
    
}

/* Below is a mechanism to define methods for individual updates to perform their computations of UPDATE_Y. */
#define OBN_DECLARE_UPDATES(...) template<const int N> void doUpdate(); \
    template<const bool temp, const int Arg1, const int... Args> void performUpdates(OBNnode::updatemask_t& m) { \
    static_assert(Arg1 >= 0 && Arg1 <= OBNnode::MAX_UPDATE_INDEX, "Update index must be positive."); \
    if (m & (1 << Arg1)) { doUpdate<Arg1>(); m ^= (1 << Arg1); } \
    performUpdates<temp, Args...>(m); } \
    template<const bool temp> void performUpdates(OBNnode::updatemask_t& m) { } \
    virtual void onUpdateY(OBNnode::updatemask_t m) { performUpdates<true, __VA_ARGS__>(m); }

#define OBN_DEFINE_UPDATE(CLS,N) template<> void CLS::doUpdate<N>()

/* Below is a mechanism to define methods for individual updates to perform their computations of UPDATE_X. */
#define OBN_DECLARE_UPDATES_X(...) template<const int N> void doUpdateX(); \
template<const bool temp, const int Arg1, const int... Args> void performUpdatesX(OBNnode::updatemask_t& m) { \
static_assert(Arg1 >= 0 && Arg1 <= OBNnode::MAX_UPDATE_INDEX, "Update index must be positive."); \
if (m & (1 << Arg1)) { doUpdateX<Arg1>(); m ^= (1 << Arg1); } \
performUpdatesX<temp, Args...>(m); } \
template<const bool temp> void performUpdatesX(OBNnode::updatemask_t& m) { } \
virtual void onUpdateX() { performUpdatesX<true, __VA_ARGS__>(_current_updates); }

#define OBN_DEFINE_UPDATE_X(CLS,N) template<> void CLS::doUpdateX<N>()

#endif /* OBNNODE_YARPNODE_H_ */