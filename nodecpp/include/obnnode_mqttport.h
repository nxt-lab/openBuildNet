/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief MQTT communication interface.
 *
 * Implement the communication interface with MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_MQTTPORT_H
#define OBNNODE_MQTTPORT_H

#ifndef OBNNODE_COMM_MQTT
#error To use this library the program must be compiled with MQTT support.
#endif

#include <cassert>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_map>
#include <vector>

#include "MQTTAsync.h"

#include "obnnode_basic.h"


namespace OBNnode {
    /** \brief Interface of an MQTT input port
     */
    class IMQTTInputPort {
    public:
        /** Parse a binary message into the port.
         If there is an error, this function should throw an exception (see obnnode_exceptions.h for predefined errors).
         The exception can be caught and delegated to the main thread, which will handle them properly.
         \param msg Pointer to the start of the message data.
         \param msglen Number of bytes of the message data.
         */
        virtual void parse_message(void* msg, int msglen) = 0;
    };
    
    /** \brief The object that manages all MQTT communications (i.e. the MQTT communication thread).
     
     Uses the Async communication interface of Paho MQTT library.
     */
    class MQTTClient {
    private:
        static const int QOS;   // The desired QOS
        
        NodeBase* m_node;  ///< The node object to which this client is attached
        
        MQTTAsync m_client;     ///< The MQTT client, used for all communication needs
        
        // std::atomic_bool m_running{false};  ///< Status of this client
        
        // The result variable, mutex and condition variable is used by MQTT callbacks to notify the main execution.
        std::atomic_int m_notify_result{0}; ///< Result of the action that was notified
        int m_notify_done;  // Only done when this value is 0
        std::mutex m_notify_mutex;
        std::condition_variable m_notify_var;
        
        /** Notify that one action is done.
         \param setDone true [default] if the function should set m_notify_done = 0 (so that wait on m_notify_var will end); false if the function should only decrease m_notify_done.
         */
        void notify_done(bool setDone = true) {
            std::lock_guard<std::mutex> mylock(m_notify_mutex);
            if (setDone) {
                m_notify_done = 0;
            } else {
                --m_notify_done;
            }
            m_notify_var.notify_all();
        }
        
        /** Client ID */
        std::string m_client_id;
        
        /** MQTT server address. */
        std::string m_server_address{"tcp://localhost:1883"};
        
        /** Map of topics to list of subscribing input ports. */
        std::unordered_map< std::string, std::vector<IMQTTInputPort*> > m_topics;
        
        std::mutex m_topics_mutex;  ///< Mutex to access the list of topics
        
        /** \brief Subscribe to all topics of the current input ports.
         \param resubscribe Set to true if this is a resubscription request => if still fails, it's communication error
         */
        void subscribeAllTopics(bool resubscribe = false);
        
        /** \brief Subscribe to a given topic.
         The subscription is asynchronous; when it's done, notify_done() is called and m_notify_result contains the result (0 = successful).
         */
        void subscribeTopic(const std::string& topic);
        
        /** \brief Unsubscribe from the given topic. */
        void unsubscribeTopic(const std::string& topic);
        
    public:
        /**
         Construct the MQTT client object, associated with a given GC.
         \param _gc Pointer to a valid GC thread, with which this thread is associated.
         */
        MQTTClient(NodeBase* pnode = nullptr): m_node(pnode) { }
        MQTTClient(const MQTTClient&) = delete;
        MQTTClient(MQTTClient&&) = delete;
        
        //virtual
        ~MQTTClient() {
            // Close the client, shutdown thread, close the port
            stop();
        }
        
        bool isRunning() const {
            return MQTTAsync_isConnected(m_client);
        }
        
        /** Set the client ID. */
        void setClientID(const std::string& _clientID) {
            assert(!_clientID.empty());
            m_client_id = _clientID;
        }
        
        /** Set the server address */
        void setServerAddress(const std::string& addr) {
            assert(!addr.empty());
            m_server_address = addr;
        }
        
        /** Set the associated node object. */
        void setNodeObject(NodeBase* pnode) {
            m_node = pnode;
        }
        
        /** \brief Subscribe a given input port to a given topic (i.e. output port in MQTT).
         
         If the topic already exists, the given port will be added to the vector associated with that topic; otherwise a new topic is added.
         \return integer code that has the same meaning as the return code of system message SYS_PORT_CONNECT_ACK.
         */
        int addSubscription(IMQTTInputPort* port, const std::string& topic);
        
        /** \brief Remove a given port from all subscriptions.
         */
        void removeSubscription(IMQTTInputPort* port);
        
        /** \brief Send data to a given topic.
         \param data Pointer to the data to be sent.
         \param size The number of bytes of the data.
         \param topic The topic to send to.
         \return true if successful.
         */
        bool sendData(void *data, int size, const std::string& topic);
        
        /** \brief Start the MQTT client (thread).
         
         Start the client / thread if it is not running already. Only one is allowed to run at any moment.
         \return True if successful; false otherwise.
         */
        bool start();
        
        /** Stop the MQTT client. */
        void stop();
        
        
    private:
        /////////////////
        // Callbacks
        /////////////////
        
        /** Called when the connection with the server is lost. */
        static void on_connection_lost(void *context, char *cause);
        
        /** Called whenever a message is received. */
        static int on_message_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message *message);
        
        /** Called when the connection with the server is established successfully. */
        static void onConnect(void* context, MQTTAsync_successData* response);
        
        /** Called when a connection attempt failed. */
        static void onConnectFailure(void* context, MQTTAsync_failureData* response);
        
        /** Called when subscription succeeds. */
        static void onSubscribe(void* context, MQTTAsync_successData* response);
        
        /** Called when subscription fails. */
        static void onSubscribeFailure(void* context, MQTTAsync_failureData* response);
        
        /** Called when the re-connection with the server is established successfully. */
        static void onReconnect(void* context, MQTTAsync_successData* response);
        
        /** Called when a re-connection attempt failed. */
        static void onReconnectFailure(void* context, MQTTAsync_failureData* response);
        
        /** Called when resubscription fails. */
        static void onReSubscribeFailure(void* context, MQTTAsync_failureData* response);
        
        /** Connection is permanently lost. Need to stop!!! */
        void onPermanentConnectionLost() {
            stop();
            // Notify that a critical error has happened
            if (m_node) {
                m_node->onPermanentCommunicationLost(COMM_MQTT);
            } else {
                // Throw an error
                throw std::runtime_error("Permanent connection lost for protocol MQTT");
            }
        }
        
        /** Called when the disconnection with the server is successful. */
        static void onDisconnect(void* context, MQTTAsync_successData* response);
    };

    
    //////////////////////////////////////////////////////////////////////
    // Definitions of MQTT ports
    //////////////////////////////////////////////////////////////////////
    
    /** The GC/SMN port in MQTT; it's just an input port. */
    class MQTTGCPort: public IMQTTInputPort {
        NodeBase* m_node;       // The node object to which the GC port will push events
    public:
    };
}

#endif // OBNNODE_MQTTPORT_H
