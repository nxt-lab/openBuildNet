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

#include <cmath>
#include <iostream>
#include <memory>               // shared_ptr
//#include <unordered_map>         // std::unordered_map
#include <forward_list>
#include <functional>
#include <vector>
#include <exception>
#include <thread>               // sleep
#include <chrono>               // timing values

#include <obnnode_basic.h>
#include <obnnode_yarpportbase.h>
#include <obnsim_msg.pb.h>

#include <sharedqueue_yarp.h>


namespace OBNnode {
    /* ============ YARP Node (managing Yarp ports) Interface ===============*/
    /** \brief Basic YARP Node. */
    class YarpNodeBase: public NodeBase {
    public:
        static yarp::os::Network yarp_network;      ///< The Yarp network object for Yarp's API
        
        /** \brief Construct a node object. */
        YarpNodeBase(const std::string& _name, const std::string& ws = ""): NodeBase(_name, ws), _smn_port(this) { }
        //virtual ~YarpNodeBase();
        
        /** Opens the port on this node to communication with the SMN, if it hasn't been opened.
         \return true if successful.
         */
        virtual bool openSMNPort() override;
        
        /** Connect the SMN port on this node to the SMN.
         The connection is started from this node, however it can also be started from the SMN.
         If the port is not yet opened, it will be opened first.
         \param carrier Optional parameter to specify the carrier for the connections.
         \return true if successful.
         */
        bool connectWithSMN(const char *carrier = nullptr);
        
        /** Callback for permanent communication lost error (e.g. the communication server has shut down).
         \param comm The communication protocol/platform that has lost.
         The node should stop its simulation (if comm is not used by the GC) and exit as cleanly as possible.
         */
        virtual void onPermanentCommunicationLost(CommProtocol comm) override {
            auto error_message = std::string("Permanent connection lost for protocol ") + (comm==COMM_YARP?"YARP":"MQTT");
            std::cerr << "ERROR: " << error_message << " => Terminate." << std::endl;
            
            if (comm != COMM_YARP) {
                // We can still try to stop the simulation as the GC's communication is not affected
                stopSimulation();
                
                // Wait a bit
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // Push an error event to the main thread
            postExceptionEvent(std::make_exception_ptr(std::runtime_error(error_message)));
        }

    protected:
        /** A YARP message for communicating with the SMN. */
        typedef YARPMsgPB<OBNSimMsg::N2SMN, OBNSimMsg::SMN2N> SMNMsg;
        
        /** The SMN port on a node to communicate with the SMN. */
        class SMNPort: public yarp::os::BufferedPort<SMNMsg> {
            YarpNodeBase* _the_node;                ///< Pointer to the node to which this port is attached
            OBNSimMsg::SMN2N _smn_message;      ///< The SMN2N message received at the node
            virtual void onRead(SMNMsg& b) override ;
        public:
            SMNPort(YarpNodeBase* node) {
                assert(node);
                _the_node = node;
            }
        };
        
        /** The Global Clock port to communicate with the SMN. */
        SMNPort _smn_port;
        
        /** Send the current message in _n2smn_message via the GC port. */
        virtual void sendN2SMNMsg() override;
        
        /* ================== Support for asynchronuous waiting for conditions ================== */
        /* We use a vector of fields: bool inuse, a semaphore, and a function object std::function<bool (...)>.
         * The function object can be assigned to a function or lambda (most of the cases) or a functor (if memory is needed).
         * We will not create and delete condition objects all the time. We create new conditions in the list/vector but do not delete them. Instead, reuse them with inuse (= true if being used, = false if not and can be reused now), so as to minimize the number of creating/deleting objects => much better for memory. Only create new object when no one can be reused.
         */
    public:
        class WaitForCondition {
            /** Status of the condition: active (waiting for), cleared (but not yet fetched), inactive (can be reused) */
            enum { ACTIVE, CLEARED, INACTIVE } status;
            yarp::os::Semaphore _event;     ///< The event condition to wait on
            OBNSimMsg::MSGDATA _data;   ///< The data record (if available) of the message that cleared the condition
            /** Returns true if the condition is cleared. */
            typedef std::function<bool (const OBNSimMsg::SMN2N&)> the_checker;
            the_checker _check_func;    ///< The function to check for the condition

            friend class YarpNodeBase;
            
            /** Make the condition inactive, to be reused later. */
            void reset() {
                status = INACTIVE;
                // reset the semaphore by decreasing it until it becomes 0
                while (_event.check()) { }  // _event.reset();
                _data.Clear();
            }
        public:
            template<typename F>
            WaitForCondition(F f): status(ACTIVE), _event(0), _check_func(f) { }
            
            // virtual ~WaitForCondition() { std::cout << "~WaitForCondition" << std::endl; }

            /** \brief Wait (blocking or with timeout) for the condition to hold.
             \param timeout Timeout in seconds, or non-positive if no timeout
             \return true if successful; false if timeout
             */
            bool wait(double timeout) {
                return (timeout <= 0.0)?(_event.wait(),true):_event.waitWithTimeout(timeout);
            }
            
            /** Return the data of the message that cleared the condition. Make sure that the condition was cleared before accessing the data. */
            const OBNSimMsg::MSGDATA& getData() const { return _data; }
        };
        
        /** Check if an wait-for condition is cleared. */
        bool isWaitForCleared(const WaitForCondition* c) {
            assert(c);
            yarp::os::LockGuard lock(_waitfor_conditions_mutex);
            return c->status == WaitForCondition::CLEARED;
        }
        
        /** Reset wait-for condition. If the condition isn't reset implicitly, it should be reset by calling this function after it was cleared and its result has been used. */
        void resetWaitFor(WaitForCondition* c) {
            assert(c);
            yarp::os::LockGuard lock(_waitfor_conditions_mutex);
            c->reset();
        }

        /** \brief Request a future irregular update from the Global Clock. */
        WaitForCondition* requestFutureUpdate(simtime_t t, updatemask_t m, bool waiting=true);
        
        /** \brief Get the result of a pending request for a future update. */
        int64_t resultFutureUpdate(WaitForCondition*, double timeout=-1.0);
        
        /** \brief Wait until a wait-for condition cleared and returns its I field. */
        int64_t resultWaitForCondition(WaitForCondition*, double timeout=-1.0);
        
    protected:
        std::forward_list<WaitForCondition> _waitfor_conditions;
        yarp::os::Mutex _waitfor_conditions_mutex;
        
        /** Check the given message against the list of wait-for conditions. */
        virtual void checkWaitForCondition(const OBNSimMsg::SMN2N&) override;

        /** The event queue, which contains smart pointers to event objects. */
        shared_queue_yarp<NodeEvent> _event_queue;
        
        /** \brief Push an event object to the event queue.
         
         This function must be implemented in a thread-safe manner w.r.t. the communication library used because it's usually called in a callback of the communication library (e.g. a Yarp thread or MQTT thread).
         */
        virtual void eventqueue_push(NodeEvent *pev) override {
            _event_queue.push(pev);
        }
        
        /** \brief Push an event object to the front of the event queue.
         
         This function must be implemented in a thread-safe manner w.r.t. the communication library used because it's usually called in a callback of the communication library (e.g. a Yarp thread or MQTT thread).
         */
        virtual void eventqueue_push_front(NodeEvent *pev) override {
            _event_queue.push_front(pev);
        }
        
        /** \brief Wait until an event exists in the queue and pop it; may wait forever.
         
         This function is called from the main thread, not from the communication callback.
         */
        virtual std::shared_ptr<NodeEvent> eventqueue_wait_and_pop() override {
            return _event_queue.wait_and_pop();
        }
        
        /** \brief Wait until an event exists in the queue and pop it; may time out.
         
         \param timeout In seconds.
         This function is called from the main thread, not from the communication callback.
         */
        virtual std::shared_ptr<NodeEvent> eventqueue_wait_and_pop(double timeout) override {
            return _event_queue.wait_and_pop_timeout(timeout);
        }
    };
    
    
    /** The main YarpNode class, which supports defining updates, _info_ port, etc. */
    typedef OBNNodeBase<YarpNodeBase> YarpNode;
    
}

/* OLD CODE: NOT TO BE USED ANYMORE!!!
 
// Below is a mechanism to define methods for individual updates to perform their computations of UPDATE_Y.
#define OBN_DECLARE_UPDATES(...) template<const int N> void doUpdate(); \
    template<const bool temp, const int Arg1, const int... Args> void performUpdates(OBNnode::updatemask_t& m) { \
    static_assert(Arg1 >= 0 && Arg1 <= OBNsim::MAX_UPDATE_INDEX, "Update index must be positive."); \
    if (m & (1 << Arg1)) { doUpdate<Arg1>(); m ^= (1 << Arg1); } \
    performUpdates<temp, Args...>(m); } \
    template<const bool temp> void performUpdates(OBNnode::updatemask_t& m) { } \
    virtual void onUpdateY(OBNnode::updatemask_t m) { performUpdates<true, __VA_ARGS__>(m); }

#define OBN_DEFINE_UPDATE(CLS,N) template<> void CLS::doUpdate<N>()

// Below is a mechanism to define methods for individual updates to perform their computations of UPDATE_X.
#define OBN_DECLARE_UPDATES_X(...) template<const int N> void doUpdateX(); \
    template<const bool temp, const int Arg1, const int... Args> void performUpdatesX(OBNnode::updatemask_t& m) { \
    static_assert(Arg1 >= 0 && Arg1 <= OBNsim::MAX_UPDATE_INDEX, "Update index must be positive."); \
    if (m & (1 << Arg1)) { doUpdateX<Arg1>(); m ^= (1 << Arg1); } \
    performUpdatesX<temp, Args...>(m); } \
    template<const bool temp> void performUpdatesX(OBNnode::updatemask_t& m) { } \
    virtual void onUpdateX() { performUpdatesX<true, __VA_ARGS__>(_current_updates); }

#define OBN_DEFINE_UPDATE_X(CLS,N) template<> void CLS::doUpdateX<N>()
*/

#endif /* OBNNODE_YARPNODE_H_ */