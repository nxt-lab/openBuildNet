/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Header file for the global clock (GC).
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * Header file for the Global Clock thread.  GC controls the entire
 * simulation and dispatches events to other systems of the SMN
 * (e.g. logging, exception handling).
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */


// TODO TODO TODO
// - Add status variable to GC, which is thread-safe to access.
// - Modify the insertNode() method to make sure that it's safe: it does not modify _nodes when the GC is running, because it will change the system.

#ifndef OBNSIM_GC_H_
#define OBNSIM_GC_H_

#include <vector>
#include <memory>
#include <chrono>
#include <utility>  // pair
#include <ctime>    // For wall-clock time

#include <sharedqueue.h>    // Thread-safe shared event queue
#include <obnsmn_event.h>
#include <obnsim_msg.pb.h>  // Protobuf-generated code for OBN-Sim messages
#include <obnsmn_node.h>
#include <obnsmn_nodegraph.h>
#include <obnsim_msg.pb.h>


namespace OBNsmn {
    /** \brief Define a GC thread.
     
         This class defines a GC thread for one simulation. It contains the main thread function, the event queue, system request support, etc.
     */
    class GCThread {
    public:
        
        GCThread(): OBNEventQueue(mWakeupCondition), rtNodeGraph(nullptr) {}
        
        virtual ~GCThread() {
            if (_gcthread) {
                // Although we can detach the thread, it's not a good idea because the thread's main procedure uses members of this object; once this object is deleted, the thread may not be able to run anymore. So in this destructor, we need to finish the thread's execution before we can destroy the object.
                
                if (_gcthread->joinable()) _gcthread->join();
                _gcthread->detach();
                delete _gcthread;
            }
        }
        
        // ========= Event queue =========
        
        /** \brief Push an SMN event to the queue.
         
         To push node events to the queue from N2SMN messages, use pushNodeEvent() instead.
         It also checks the message to ensure validity.
         \param ev The event object, which must be dynamically allocated or a shared_ptr.
         */
        template <typename T>
        void pushEvent(T ev) {
            OBNEventQueue.push(ev);
        }
        
        /** \brief Create a node event from an N2SMN message and push it to the queue. */
        bool pushNodeEvent(const OBNSimMsg::N2SMN& msg, int defaultID, bool overrideID = false);

        
        // ========== System Request ===========
        
        /** \brief System request type.
         
         This type specifies a system request (from the main thread) to the simulation thread (GC).
         Different requests are defined, like STOP, TERMINATE, PAUSE, etc.
         */
        enum OBNSysRequestType {
            SYSREQ_NONE,        ///< no request, keep continuing
            SYSREQ_STOP,        ///< stop the simulation nicely (after the current time step)
            SYSREQ_TERMINATE,   ///< stop the simulation immediately, even in the middle of a time step
            SYSREQ_PAUSE,       ///< pause the simulation (immediately?)
            SYSREQ_RESUME,      ///< resume simulation from paused / terminating state, but not from terminated state
            SYSREQ_STEP         ///< run simulation for one step then go back to paused
        };
        
        /** \brief Set system request to GC.
         
         Set the system request to the GC, which overrides the current system request. Thread safe.
         \param r The system request.
         */
        void setSysRequest(OBNSysRequestType r) {
            std::unique_lock<std::mutex> mlock(mSysReq);
            _SysRequest = r;
            mlock.unlock();
            mWakeupCondition.notify_one();
        }
        
        /** \brief Return current system request.
         
         Returns the current system request. Thread safe.
         */
        OBNSysRequestType getSysRequest() {
            std::unique_lock<std::mutex> mlock(mSysReq);
            return _SysRequest;
        }
        
        /** \brief Reset the system request to NONE.
         
         Reset the system request to NONE (i.e. no request).
         */
        void resetSysRequest() {
            std::unique_lock<std::mutex> mlock(mSysReq);
            _SysRequest = SYSREQ_NONE;
        }
        
        
        // ========== Configuration ============
        
        /** Timeout, in milliseconds, for waiting for acknowledgement from nodes (UPDATE_Y, UPDATE_X).
         If the timeout is non-positive, no timeout will be used (i.e. waiting indefinitely).
         */
        int ack_timeout = 0;
        
        /** Set the simulation time unit.
         \param T The simulation time unit, in number of microseconds.
         \return true if successful.
         */
        bool setSimulationTimeUnit(simtime_t T) {
            // Only set it when the GC is not (yet) running.
            if ((T > 0) && (!_gcthread)) {
                sim_time_unit = T;
                return true;
            }
            
            return false;
        }
        
        /** Set the final simulation time.
         \return true if successful.
         */
        bool setFinalSimulationTime(simtime_t T) {
            // Only set final time when the GC is not (yet) running.
            if ((T > 0) && (!_gcthread)) {
                final_sim_time = T;
                return true;
            }
            
            return false;
        }
        
        /** Set the initial wall-clock time.
         \param T The initial time as Epoch/UNIX time.
         \return true if successful.
         */
        bool setInitialWallclock(std::time_t T) {
            // Only set the time when the GC is not (yet) running.
            if (!_gcthread) {
                initial_wallclock = T;
                return true;
            }
            
            return false;
        }
        
        
        ///@{
        /** Set the node dependency graph.
         \return true if successful.
         */
        bool setDependencyGraph(NodeDepGraph* p) {
            // Only set when the GC is not (yet) running.
            if (!_gcthread) {
                _nodeGraph.reset(p);
                return true;
            }
            return false;
        }
        
        bool setDependencyGraph(std::unique_ptr<NodeDepGraph> p) {
            // Only set when the GC is not (yet) running.
            if (!_gcthread) {
                _nodeGraph = std::move(p);
                return true;
            }
            return false;
        }
        ///@}
        
        /** Insert a new node to the list of nodes.
         \param p Pointer to a node object.
         \return a pair of <bool success,size_t index> where success=true if successful, then index is the ID of the newly added node
         */
        std::pair<bool, std::size_t> insertNode(OBNNode* p) {
            if (!_gcthread) {
                //_nodes.push_back(std::unique_ptr<OBNNode>(p));
                _nodes.emplace_back(p);
                return std::make_pair(true, _nodes.size()-1);
            }
            return std::make_pair(false, 0);
        }
        
        /** Return number of nodes in the node list. */
        int numberOfNodes() const { return _nodes.size(); }
        
        
        // ========== Control the thread =============
        
        /** \brief Start the GC thread.
         
         Start the GC thread if it is not running already. Only one thread is allowed to run at any moment.
         \return True if successful; false otherwise.
         */
        
        bool startThread() {
            if (_gcthread) return false;
            
            // Initialize the simulation
            initialize();

            _gcthread = new std::thread(&GCThread::GCThreadMain, this);
            
            return true;
        }
        
        /** \brief Join GC thread to current thread.
         
         Join the GC thread (if one is running) to the current thread, which will be blocked until the GC thread ends.
         This is useful for the main thread to wait for the GC thread to stop.
         \return True if successful; false otherwise.
         */
        bool joinThread() {
            if (!_gcthread) return false;
            
            _gcthread->join();
            // We can now safely destroy _gcthread
            delete _gcthread;
            _gcthread = nullptr;
            return true;
        }
        
        
        // =========== STATUS AND SIGNALS ==============
        // These variables and methods are used to control the status of the GC / simulation,
        // and also used by the GC to signal other objects to change their execution (e.g. to stop other threads running together with the GC).
        
        /** \brief Simple signal variable to stop simple threads running to support the GC.
         
         This is a simple mechanism for the GC to tell simple CPU-looping threads to terminate.
         This bool variable is only set by the GC thread. It should NEVER be set by any other thread.
         Other threads can read this variable to determine when the GC signals them to terminate.
         The main use of this variable is to signal supporting threads, for example communication threads, to stop when the GC terminates its execution.
         USE THIS VARIABLE CAREFULLY. AND MAKE SURE YOU NEVER WRITE TO THIS VARIABLE OUTSIDE THE GC THREAD.
         */
        volatile bool simple_thread_terminate = false;
        
    private:
        // =========== Event queue ============

        typedef shared_queue<OBNsmn::SMNNodeEvent> OBNEventQueueType;
        /** \brief The main event queue.
         
         This is the main event queue. GC is the sole consumer which reads
         and processes the events in this queue, mainly to drive the
         simulation. Other threads (communication, main) pushes to this
         queue so that GC can process them in order.
         
         \note This shared queue contains smart pointers to objects of type SMNEvent.
         When pushing new objects, remember to dynamically create the objects, not a local scope object, e.g. using make_shared.
         Because this uses smart pointers, after popping out an object (actually a pointer to an object),
         there is no need to delete the object.
         */
        OBNEventQueueType OBNEventQueue;
        

        // ============ Thread control =============
        OBNSysRequestType _SysRequest;
        mutable std::mutex mSysReq;     // mutex for thread-safe access to _SysRequest
        
        std::condition_variable mWakeupCondition;   // condition variable for the main thread to wait idly and be waken up
        
        std::thread * _gcthread = nullptr;
        
        void GCThreadMain();    ///< This function is the entry point for the GC thread. Do not call it directly.
        
        // ============ Node management ==============
        
        /** \brief List of nodes in the network, their indices will be their IDs. */
        std::vector< std::unique_ptr<OBNNode> > _nodes;
        
        /** Maximum valid ID of node (number of nodes - 1). */
        int maxID;
        
        /** \brief Graph of nodes' dependency. */
        std::unique_ptr<NodeDepGraph> _nodeGraph;
        
        // ============ Simulation control =============
        
        /** The time unit of simulation time, in number of microseconds. */
        simtime_t sim_time_unit = 1;
        
        /** The current simulation time (not wall clock). */
        simtime_t current_sim_time;
        
        /** The final simulation time. Every simulation must have a stop time. */
        simtime_t final_sim_time;
        
        /** The initial wall clock time at the start of the simulation. */
        std::time_t initial_wallclock = 0;
        
        /** \brief Initialize the simulation before it can start. */
        bool initialize();
        
        
    private:
        // ========================================================================
        // ============ Everything related to the main GC algorithm ===============
        // ========================================================================
        
        /** Execution states / modes of the GC, e.g. running, paused.
         Refer to the state machine diagrams.
         */
        enum GCExecStateType {
            GCSTATE_RUNNING,    //< the simulation is running in normal mode
            GCSTATE_PAUSED,     //< the simulation is paused, can be resumed to normal mode
            GCSTATE_TERMINATING //< the simulation is being terminated, can be resumed to normal mode
        };
        
        GCExecStateType gc_exec_state;
        
        // ============ Helper functions to process events ============
        
        /** \brief Wait for the next event of the main GC algorithm. */
        bool gc_wait_for_next_event(OBNEventQueueType::item_type& ev, OBNSysRequestType& req);
        
        /** \brief Block until wait-for event done, while processing all events. */
        template <class F>
        bool gc_wait_for_ack(F f);
        
        bool gc_wait_for_ack() { return gc_wait_for_ack([](const OBNsmn::SMNNodeEvent* ev) {return true;});}
        
        /** \brief Default node event processing function. */
        bool gc_process_node_events(OBNsmn::SMNNodeEvent* pEv);
        
        /** \brief Process system request, mostly to switch between modes. */
        void gc_process_sysreq(OBNSysRequestType req);
        
        
        // ============ Data and methods related to one simulation iteration ===========
        
        /** \name UpdateListGroup
         *  Manage the list of updating nodes, using a pre-allocated array of NodeUpdateInfo to specify the current updates.
         *  See file comment for more details.
         */
        ///@{
        /** A pre-allocated array of update info list. */
        NodeUpdateInfoList gc_update_list;
        
        /** Actual size of the update info list = Total number of updating nodes.
         \sa gc_update_list
         */
        size_t gc_update_size;

        /* Number of irregular updates.
        size_t gc_update_irregular_size; */
        
        /* Number of regular updates
        size_t gc_update_regular_size; */
        
        /** \brief Start the next update. */
        bool startNextUpdate();
        ///@}
        
        
        /** Pointer to the run-time node graph, created in each simulation iteration. */
        RTNodeDepGraph* rtNodeGraph;
        
        /** \brief Send UPDATEY to certain nodes and start wait-for for them. */
        bool gc_send_update_y();
        
        /* \brief Send irregular UPDATEY to certain nodes and start wait-for for them.
        bool gc_send_update_y_irregular(); */

        /** \brief Send UPDATEX to certain nodes and start wait-for for them. */
        bool gc_send_update_x();
        
        /** \brief Send a given simple message to all nodes without waiting for ACKs. */
        bool gc_send_to_all(simtime_t t, OBNSimMsg::SMN2N::MSGTYPE msgtype, OBNSimMsg::MSGDATA *pData = NULL);
        
        /** \brief Send a given simple message to all nodes and start wait-for event. */
        bool gc_send_to_all(simtime_t t, OBNSimMsg::SMN2N::MSGTYPE msgtype, OBNSimMsg::N2SMN::MSGTYPE acktype,  OBNSimMsg::MSGDATA *pData = NULL);
        
        
        // ============ Wait-for event for the GC algorithm =============
        /** Indicate if a wait-for event is active */
        bool gc_waitfor_active;
        
        /** Bit vector to store the status of nodes' ACK for wait-for event. */
        std::vector<bool> gc_waitfor_bits;
        
        /** Number of nodes from which we are waiting for ACK = number of 0 bits in gc_waitfor_bits. */
        int gc_waitfor_num;
        
        /** Type of the ACK expected for the wait-for event. */
        OBNSimMsg::N2SMN::MSGTYPE gc_waitfor_type;
        
        /** Start a new wait-for event.
         \param nodes List of indices of nodes expected to send ACKs. It's ASSUMED WITHOUT CHECKING that these indices are unique.
         \param type Type of the expected ACK message.
         \return true if successful, false if not (a wait-for is going on, simulation is not going on).
         */
        template <class L>
        bool gc_waitfor_start(const L & nodes, OBNSimMsg::N2SMN::MSGTYPE type) {
            if (gc_waitfor_active) {
                return false;
            }
            
            gc_waitfor_active = true;
            gc_waitfor_type = type;
            for (const auto & it: nodes) {
                gc_waitfor_bits[it.first] = false;
            }
            gc_waitfor_num = nodes.size();
            return true;
        }
        
        /** Mark bit n as done. */
        void gc_waitfor_mark(int n) {
            if (gc_waitfor_active && !gc_waitfor_bits[n]) {
                gc_waitfor_bits[n] = true;
                gc_waitfor_num--;
            }
        }
        
        /** Check if the waitfor is done. */
        bool gc_waitfor_alldone() const {
            return gc_waitfor_active && (gc_waitfor_num == 0);
            // std::all_of(gc_waitfor_bits.begin(), gc_waitfor_bits.end(), [](bool b) { return b; });
        }
       
        // ============ Timer event for the GC algorithm =============
        /** Indicate if a timer event is active. */
        bool gc_timer_active;
        
        /** The end time of the current timer event. */
        std::chrono::steady_clock::time_point gc_timer_endtime;
        
        /** \brief Start a new timer event, even if one is still running.
         \param dur The duration of the timer event in ms counting from now.
         */
        void gc_timer_start(unsigned int ms) {
            gc_timer_active = true;
            gc_timer_endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        }
        
        /** Reset/stop current timer event. */
        void gc_timer_reset() {
            gc_timer_active = false;
        }
    };
}

#endif /* OBNSIM_GC_H_ */


