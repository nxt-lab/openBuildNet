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

#ifndef OBNSIM_COMM_MQTT_H
#define OBNSIM_COMM_MQTT_H

#ifndef OBNSIM_COMM_MQTT
#error To use this library the program must be compiled with MQTT support.
#endif

#include <cassert>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_set>

#include <regex>    // For checking topic names

#include <obnsmn_node.h>
#include <obnsmn_report.h>
#include <obnsmn_gc.h>

#include "MQTTAsync.h"


namespace OBNsmn {
    namespace MQTT {
        
        class OBNNodeMQTT;
        
        /** \brief The object that manages all MQTT communications (i.e. the MQTT communication thread).

         Uses the Async communication interface of Paho MQTT library.
         */
        class MQTTClient {
        private:
            static const int QOS = 2;   // The desired QOS for the main connection (main GC topic)
            
            OBNsmn::GCThread::TSendMsgToSysPortFunc m_prev_gc_sendmsg_to_sys_port;
            MQTTAsync m_client;     ///< The MQTT client, used for all communication needs
            std::atomic_bool m_running{false};     ///< Whether the MQTT client is running
            
            // The result variable, mutex and condition variable is used by MQTT callbacks to notify the main execution.
            bool m_notify_done;
            int m_notify_result;    // Result of the action, typically 0 means success
            std::mutex m_notify_mutex;
            std::condition_variable m_notify_var;
            void notify_done(int result = 0) {
                std::lock_guard<std::mutex> mylock(m_notify_mutex);
                m_notify_done = true;
                m_notify_result = result;
                m_notify_var.notify_all();
            }
            
            /** The GC object with which this communication thread is associated. */
            GCThread *pGC;
            
            /** Name of the incoming MQTT port. */
            std::string m_portName;
            
            /** Client ID */
            std::string m_client_id;
            
            /** MQTT server address. */
            std::string m_server_address;
            
            /** The N2SMN message used for receiving data from GC port. */
            OBNSimMsg::N2SMN m_n2smn_msg;
            
            /** List of nodes that have announced their availability. */
            std::unordered_set<std::string> m_online_nodes;
            std::string m_online_nodes_topic;
            bool m_listening_for_arrivals{false};
            std::mutex m_online_nodes_mutex;
            static std::regex online_nodes_topic_regex;
            
            friend class OBNNodeMQTT;
            
        public:
            /**
             Construct the MQTT client object, associated with a given GC.
             \param _gc Pointer to a valid GC thread, with which this thread is associated.
             */
            MQTTClient(GCThread* _gc): pGC(_gc) {
                // Set the function to send a message to the system port
                m_prev_gc_sendmsg_to_sys_port = pGC->getSendMsgToSysPortFunc();     // Save the current function to call it -> form a chain of function calls
                pGC->setSendMsgToSysPortFunc(std::bind(&MQTTClient::sendMessageToGC, this, std::placeholders::_1));
            }
            
            //virtual
            ~MQTTClient() {
                // Close the client, shutdown thread, close the port
                stop();
            }
            
            bool isRunning() const {
                return m_running;
            }
            
            /** \brief Send an SMN2N message to the system port. */
            bool sendMessageToGC(const OBNSimMsg::SMN2N &msg);
            
            /** \brief Send an SMN2N message to a given topic.
             \param retained If this message should be retained on the broker.
             \return The return code of MQTT's sendMessage().
             */
            int sendMessage(const OBNSimMsg::SMN2N &msg, const std::string& topic, int retained = 0);
            
            /** \brief Send a raw message to a given topic.
             \param retained If this message should be retained on the broker.
             \return The return code of MQTT's sendMessage().
             */
            int sendMessage(char* msg, std::size_t msglen, const std::string& topic, int retained = 0);
            
            /** Set the port's name. */
            void setPortName(const std::string &t_port) {
                assert(!t_port.empty());
                m_portName = t_port;
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
            
            /** \brief Open the port.
             \return True if successful.
             */
            bool openPort() { return true; } //port.open(portName); }
            
            /** \brief Close the port (stop activities). */
            void closePort() { } //port.interrupt(); port.close(); }
            
            /** \brief Start the MQTT client (thread).
             
             Start the client / thread if it is not running already. Only one is allowed to run at any moment.
             \return True if successful; false otherwise.
             */
            
            bool start();
            
            /** Stop the MQTT client. */
            void stop();
            
            /** Start waiting for nodes to announce their arrivals.
             \param t_workspace The workspace name; either "" (default) or of the form "workspace/" (note the / at the end)
             */
            bool startListeningForArrivals(const std::string& t_workspace);
            
            /** Stop waiting for nodes to announce their arrivals. */
            void stopListeningForArrivals();
            
            /** Check if a given node has announced its arrival. */
            bool checkNodeOnline(const std::string& name);
            
            /** Clear the list of online nodes. */
            void clearListOfOnlineNodes();
            
//            /** \brief Join the thread to current thread.
//             
//             Join the thread (if one is running) to the current thread, which will be blocked until the thread ends.
//             This is useful for the main thread to wait for the thread to stop.
//             \return True if successful; false otherwise.
//             */
//            bool joinThread() {
//                if (!pThread) return false;
//                
//                pThread->join();
//                // We can now safely destroy _gcthread
//                delete pThread;
//                pThread = nullptr;
//                return true;
//            }
            
        private:
            /////////////////
            // Callbacks
            /////////////////
            
            /** Called when the connection with the server is lost. */
            static void on_connection_lost(void *context, char *cause);
            
            /** Called whenever a message is received. */
            static int on_message_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message *message);
            
            /** Called whenever a sent message has been delivered. */
            static void on_message_delivered(void *context, MQTTAsync_token token);

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
                // Notify the GC that a critical error has happened
                pGC->criticalErrorExit();
            }
            
            /** Called when the disconnection with the server is successful. */
            static void onDisconnect(void* context, MQTTAsync_successData* response);
        };
        
        
        /** Node implementation using MQTT for communication. */
        class OBNNodeMQTT: public OBNNode {
        public:
            
            /** \brief Construct an MQTT node with a given port.
             
             Construct a MQTT node with a given name and number of output groups, associated with an MQTTClient object.
             \param _name The name of the node.
             \param t_nUpdates The number of computating tasks/update types.
             \param t_topic The topic of the GC port of this node, typically of the form "workspace_name/node_name/_gc_".
             \param t_client Pointer to the MQTTClient.
             The port is only used for sending messages out.
             */
            OBNNodeMQTT(const std::string& _name, int t_nUpdates, const std::string &t_topic, MQTTClient* t_client):
            OBNNode(_name, t_nUpdates), m_topic(t_topic), m_client(t_client)
            {
                assert(t_client);
                assert(!t_topic.empty());
            }
            
            virtual ~OBNNodeMQTT() {
                if (m_buffer) {
                    delete [] m_buffer;
                }
            }
            
            /** Make sure that the buffer (for sending messages) has at least newsize bytes. */
            void allocateBuffer(size_t newsize);
            
            /** \brief Asynchronously send a message to a node. */
            virtual bool sendMessage(int nodeID, OBNSimMsg::SMN2N &msg) override;
            
            /* \brief Synchronously send a message to a node.  */
            // virtual bool sendMessageSync(int nodeID, OBNSimMsg::SMN2N &msg);
            
        private:
            /** The MQTT topic that this node will send to. */
            std::string m_topic;
            
            /** \brief MQTTClient object with which this node is associated, for sending messages to node. */
            MQTTClient *m_client;
            
            // The buffer for sending messages
            char* m_buffer = nullptr;
            
            // Allocated size of the buffer
            size_t m_buffer_allocsize = 0;
        };
    }
}

#endif // OBNSIM_COMM_MQTT_H
