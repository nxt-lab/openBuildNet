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

#include <memory>               // shared_ptr
//#include <unordered_map>         // std::unordered_map
#include <forward_list>

#include <obnnode_basic.h>
#include <obnnode_yarpportbase.h>
#include <obnsim_msg.pb.h>

#include <sharedqueue_yarp.h>


namespace OBNnode {
    
    /** A YARP message for communicating with the SMN. */
    typedef YARPMsgPB<OBNSimMsg::N2SMN, OBNSimMsg::SMN2N> smnMsg;
    
    /** A buffered YARP port for communication with the SMN. */
    typedef yarp::os::BufferedPort<smnMsg> smnPort;
    
    
    /* ============ YARP Node (managing Yarp ports) Interface ===============*/
    class YarpNode {
    public:
        /** Type of the node's state. */
        enum NODE_STATE {
            NODE_STOPPED,   ///< Node hasn't run yet, or has stopped
            NODE_STARTED,   ///< Node has been started (from STOPPED) but not yet running, it must be initialized first
            NODE_RUNNING    ///< The node is running (simulation is going on)
        };
        
        
    protected:
        /** Name of the node. */
        std::string _nodeName;

        /** List of physical input ports. */
        std::forward_list<YarpPortBase*> _input_ports;
        
        /** List of physical output ports. */
        std::forward_list<YarpOutputPortBase*> _output_ports;
        
        /** Attach a port object to this node. */
        bool attachAndOpenPort(YarpPortBase * port);
        
        /** Current state of the node. */
        NODE_STATE _node_state;
        
        /** The ID of the node in the network (assigned by the GC in its messages to the node) */
        int32_t _node_id;


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
        };
        
        struct NodeEvent_UPDATEY: public NodeEventSMN {
            virtual void execute(YarpNode*);
            updatemask_t _updates;
            NodeEvent_UPDATEY(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) {
                _updates = (msg.has_data() && msg.data().has_i())?msg.data().i():0;
            }
        };
        friend NodeEvent_UPDATEY;
        
        struct NodeEvent_UPDATEX: public NodeEventSMN {
            virtual void execute(YarpNode*);
            NodeEvent_UPDATEX(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_UPDATEX;
        
        struct NodeEvent_INITIALIZE: public NodeEventSMN {
            virtual void execute(YarpNode*);
            NodeEvent_INITIALIZE(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_INITIALIZE;
        
        struct NodeEvent_TERMINATE: public NodeEventSMN {
            virtual void execute(YarpNode*);
            NodeEvent_TERMINATE(const OBNSimMsg::SMN2N& msg): NodeEventSMN(msg) { }
        };
        friend NodeEvent_TERMINATE;
        
    public:
        /** Post a system openBuildNet event to the end of the queue (from an SMN2N message). */
        void postEvent(const OBNSimMsg::SMN2N& msg);
        
        /* Methods for pushing events of other types (node internal events) will be put here */
        
        
        YarpNode(const std::string _name);
        virtual ~YarpNode();
        
        
        /** Return full path to a port in this node. */
        std::string fullPortName(const std::string& portName) const {
            return '/' + _nodeName + '/' + portName;
        }
        
        
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
   
        
    public:
        /* =========== Simulation callbacks =========== */
//        virtual bool onUpdateY(ARGUMENTS HERE time update type, etc) {
//            
//        }
//        
//        virtual bool onUpdateX(time ) {
//            
//        }
//        
//        virtual bool onInitialization() {
//            
//        }
        
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
        virtual void onOBNError(const std::string msg) {
            
        }
        
        /** Callback for warning issues interacting with the SMN and openBuildNet system, e.g. an unrecognized system message from the SMN.  Usually the simulation may continue without any serious consequence.
         \param msg A string containing the warning message.
         */
        virtual void onOBNWarning(const std::string msg) {
            
        }
    };
}

#endif /* OBNNODE_YARPNODE_H_ */