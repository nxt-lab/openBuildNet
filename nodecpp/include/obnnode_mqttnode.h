/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief MQTT node class for the C++ node interface.
 *
 * Implement the main node class for a C++ node.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_MQTTNODE_H_
#define OBNNODE_MQTTNODE_H_

#include <cmath>
#include <iostream>
#include <memory>               // shared_ptr
//#include <unordered_map>         // std::unordered_map
#include <forward_list>
#include <functional>
#include <vector>
#include <exception>
#include <thread>               // sleep
#include <condition_variable>
#include <mutex>
#include <chrono>               // timing values

#include <sharedqueue_std.h>
#include <obnnode_basic.h>
#include <obnnode_mqttport.h>

#include <obnsim_msg.pb.h>

namespace OBNnode {
    /* ============ MQTT Node Interface ===============*/
    /** \brief Basic MQTT Node. */
    class MQTTNodeBase: public NodeBase {
        bool m_mqtt_client_initialized{false};  ///< If the Client object has been initialized
    public:
        OBNnode::MQTTClient mqtt_client;      ///< The MQTT client object for all MQTT communications of this node
        
        /** Set the MQTT's client ID. */
        void setClientID(const std::string& _clientID) {
            mqtt_client.setClientID(_clientID);
        }
        
        /** Set the MQTT server address */
        void setServerAddress(const std::string& addr) {
            mqtt_client.setServerAddress(addr);
        }
        
        /** Start the MQTT communication. */
        bool startMQTT();
        
        /** \brief Construct a node object. */
        MQTTNodeBase(const std::string& _name, const std::string& ws = ""): NodeBase(_name, ws),
        mqtt_client(this), m_smn_port(this)
        {
            mqtt_client.setClientID(full_name());
            m_smn_topic = _workspace + "_smn_/_gc_";    // The topic of the SMN's GC port; all nodes publish to this topic
        }
        
        //virtual ~MQTTNodeBase();
        
        // Override these methods to also set the MQTT client of the new port
        virtual bool addInput(InputPortBase* port, bool owned=false) override;
        virtual bool addOutput(OutputPortBase* port, bool owned=false) override;
        
        // This method removes the port and also unsubscribes from the associated topics of the port
        virtual void removePort(InputPortBase* port) override;
        
        /** Opens the port on this node to communication with the SMN, if it hasn't been opened.
         \return true if successful.
         */
        virtual bool openSMNPort() override;
        
        /** Callback for permanent communication lost error (e.g. the communication server has shut down).
         \param comm The communication protocol/platform that has lost.
         The node should stop its simulation (if comm is not used by the GC) and exit as cleanly as possible.
         */
        virtual void onPermanentCommunicationLost(CommProtocol comm) override;

    protected:
        /** The Global Clock port to communicate with the SMN. */
        OBNnode::MQTTGCPort m_smn_port;
        
        std::string m_smn_topic;    ///< Topic of the SMN's main port for nodes to send to
            
        OBNsim::ResizableBuffer m_gcbuffer;   ///< The buffer for sending messages to SMN
            
        /** Send the current message in _n2smn_message via the GC port. */
        virtual void sendN2SMNMsg() override;
        
        /** Initialize the simulation, even before we start receiving the INIT message. */
        virtual bool initializeForSimulation() override;
        
        /* ================== Support for asynchronuous waiting for conditions ================== */
        /* We use a vector of fields: bool inuse, a semaphore, and a function object std::function<bool (...)>.
         * The function object can be assigned to a function or lambda (most of the cases) or a functor (if memory is needed).
         * We will not create and delete condition objects all the time. We create new conditions in the list/vector but do not delete them. Instead, reuse them with inuse (= true if being used, = false if not and can be reused now), so as to minimize the number of creating/deleting objects => much better for memory. Only create new object when no one can be reused.
         */
    public:
        class WaitForCondition {
            /** Status of the condition: active (waiting for), cleared (but not yet fetched), inactive (can be reused) */
            enum { ACTIVE, CLEARED, INACTIVE } status;
            
            std::condition_variable _event; ///< The event condition to wait on
            bool _waitfor_done;   ///< If the event is actually done
            std::mutex _mutex;  ///< The mutex for this object
            
            OBNSimMsg::MSGDATA _data;   ///< The data record (if available) of the message that cleared the condition
            
            /** Returns true if the condition is cleared. */
            typedef std::function<bool (const OBNSimMsg::SMN2N&)> the_checker;
            the_checker _check_func;    ///< The function to check for the condition

            friend class MQTTNodeBase;
            
            /** Make the condition inactive, to be reused later. */
            void reset() {
                std::lock_guard<std::mutex> lockcond(_mutex);
                status = INACTIVE;
                _waitfor_done = false;
                _data.Clear();
            }
        public:
            template<typename F>
            WaitForCondition(F f): status(ACTIVE), _waitfor_done(false), _check_func(f) { }
            
            // virtual ~WaitForCondition() { std::cout << "~WaitForCondition" << std::endl; }

            /** \brief Wait (blocking or with timeout) for the condition to hold.
             \param timeout Timeout in seconds, or non-positive if no timeout
             \return true if successful; false if timeout
             */
            bool wait(double timeout) {
                std::unique_lock<std::mutex> lck(_mutex);
                
                if (timeout <= 0.0) {
                    _event.wait(lck, [this]{ return this->_waitfor_done; });
                    return true;
                } else {
                    return _event.wait_for(lck,
                                           std::chrono::milliseconds(int(timeout * 1000)),
                                           [this]{ return this->_waitfor_done; });
                }
            }
            
            /** Return the data of the message that cleared the condition. Make sure that the condition was cleared before accessing the data. */
            const OBNSimMsg::MSGDATA& getData() const { return _data; }
        };
        
        /** Check if an wait-for condition is cleared. */
        bool isWaitForCleared(const WaitForCondition* c) {
            assert(c);
            std::lock_guard<std::mutex> lock(_waitfor_conditions_mutex);
            return c->status == WaitForCondition::CLEARED;
        }
        
        /** Reset wait-for condition. If the condition isn't reset implicitly, it should be reset by calling this function after it was cleared and its result has been used. */
        void resetWaitFor(WaitForCondition* c) {
            assert(c);
            std::lock_guard<std::mutex> lock(_waitfor_conditions_mutex);
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
        std::mutex _waitfor_conditions_mutex;
        
        /** Check the given message against the list of wait-for conditions. */
        virtual void checkWaitForCondition(const OBNSimMsg::SMN2N&) override;

        /** The event queue, which contains smart pointers to event objects. */
        shared_queue<NodeEvent> _event_queue;
        
        /** \brief Push an event object to the event queue.
         
         This function must be implemented in a thread-safe manner w.r.t. the communication library used because it's usually called in a callback of the communication library (e.g. a MQTT thread or MQTT thread).
         */
        virtual void eventqueue_push(NodeEvent *pev) override {
            _event_queue.push(pev);
        }
        
        /** \brief Push an event object to the front of the event queue.
         
         This function must be implemented in a thread-safe manner w.r.t. the communication library used because it's usually called in a callback of the communication library (e.g. a MQTT thread or MQTT thread).
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
    
    
    /** The main MQTTNode class, which supports defining updates, _info_ port, etc. */
    typedef OBNNodeBase<MQTTNodeBase> MQTTNode;
    
}

#endif /* OBNNODE_MQTTNODE_H_ */