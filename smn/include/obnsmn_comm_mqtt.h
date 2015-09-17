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

#include <obnsmn_node.h>
#include <obnsmn_report.h>
#include <obnsmn_gc.h>

#include "MQTTAsync.h"


namespace OBNsmn {
    namespace MQTT {

        /** Node implementation using MQTT for communication. */
//        class OBNNodeMQTT: public OBNNode {
//        public:
//            
//            /** \brief Construct an MQTT node with a given port.
//             
//             Construct a MQTT node with a given name, number of output groups, and a given MQTT port.
//             The node object will use the port as-is: it will not create / delete / open / close / connect the port.
//             Therefore, the port object must be initialized, and then deleted, properly somewhere else.
//             
//             The port is only used for sending messages out.
//             */
//            OBNNodeMQTT(const std::string& _name, int t_nUpdates, MQTTPort* _port): OBNNode(_name, t_nUpdates), port(_port) {
//                assert(_port);
//            }
//            
//            /** \brief Construct an MQTT node which owns a given port.
//             
//             This constructor is similar to the other constructor, however the MQTT node will own the given MQTT port object.
//             The node object will not create / open / connect the port, but it will close and delete the port when itself is deleted.
//             IOW, the node object owns the port object.
//             
//             The port is only used for sending messages out.
//             */
//            OBNNodeMQTT(const std::string& _name, int t_nUpdates, std::unique_ptr<MQTTPort> &&t_port): OBNNode(_name, t_nUpdates), port(t_port.get()) {
//                assert(t_port);
//                m_ownedport = std::move(t_port);
//            }
//            
//            // virtual ~OBNNodeMQTT() { //std::cout << "OBNNodeMQTT deleted." << std::endl; }
//            
//            /** \brief Asynchronously send a message to a node. */
//            virtual bool sendMessage(int nodeID, OBNSimMsg::SMN2N &msg) override;
//            
//            /* \brief Synchronously send a message to a node.  */
//            // virtual bool sendMessageSync(int nodeID, OBNSimMsg::SMN2N &msg);
//            
//        private:
//            /** \brief MQTT port object for sending messages to node. */
//            MQTTPort *port;
//            
//            /** Port object that is owned by this node object, may be empty (none is owned). */
//            std::unique_ptr<MQTTPort> m_ownedport;
//        };
        
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
            std::mutex m_notify_mutex;
            std::condition_variable m_notify_var;
            void notify_done() {
                std::lock_guard<std::mutex> mylock(m_notify_mutex);
                m_notify_done = true;
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
            
        public:
            /**
             Construct the MQTT client object, associated with a given GC.
             \param _gc Pointer to a valid GC thread, with which this thread is associated.
             */
            MQTTClient(GCThread* _gc): pGC(_gc) {
                // Set the function to send a message to the system port
                m_prev_gc_sendmsg_to_sys_port = pGC->getSendMsgToSysPortFunc();     // Save the current function to call it -> form a chain of function calls
                pGC->setSendMsgToSysPortFunc(std::bind(&MQTTClient::sendMessage, this, std::placeholders::_1));
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
            bool sendMessage(const OBNSimMsg::SMN2N &msg);
            
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
            
            bool start() {
                if (m_running) return false;  // Already running
                if (!pGC) return false;    // pGC must point to a valid GC object
                
                if (m_portName.empty() || m_client_id.empty() || m_server_address.empty()) {
                    OBNsmn::report_error(0, "MQTT error: the GC port name and the client ID and the MQTT server address must be set.");
                    return false;
                }
                
                int rc;
                
                // Start the client and immediately subscribe to the main GC topic
                if ((rc = MQTTAsync_create(&m_client, m_server_address.c_str(), m_client_id.c_str(), MQTTCLIENT_PERSISTENCE_DEFAULT, NULL)) != MQTTASYNC_SUCCESS) {
                    OBNsmn::report_error(0, "MQTT error: could not create MQTT client with error code = " + std::to_string(rc));
                    return false;
                }
                
                // Set the callback
                if ((rc = MQTTAsync_setCallbacks(m_client, this, &MQTTClient::on_connection_lost, &MQTTClient::on_message_arrived, NULL)) != MQTTASYNC_SUCCESS) {
                    OBNsmn::report_error(0, "MQTT error: could not set callbacks with error code = " + std::to_string(rc));
                    return false;
                }
                
                // Connect and subscribe
                MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
                conn_opts.keepAliveInterval = 20;
                conn_opts.cleansession = 1;
                conn_opts.onSuccess = &MQTTClient::onConnect;
                conn_opts.onFailure = &MQTTClient::onConnectFailure;
                conn_opts.context = this;
                if ((rc = MQTTAsync_connect(m_client, &conn_opts)) != MQTTASYNC_SUCCESS)
                {
                    OBNsmn::report_error(0, "MQTT error: could not start connect with error code = " + std::to_string(rc));
                    return false;
                }
                
                // Wait until subscribed successfully (or failed)
                std::unique_lock<std::mutex> mylock(m_notify_mutex);
                m_notify_done = false;
                m_notify_var.wait(mylock, [this](){ return m_notify_done; });
                
                m_running = true;
                
                return true;
            }
            
            /** Stop the MQTT client. */
            void stop() {
                if (!m_running) {
                    return;
                }
                
                // Disconnect from the server
                m_running = false;
                
                MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer;
                disc_opts.onSuccess = &MQTTClient::onDisconnect;
                disc_opts.context = this;
                int rc;
                if ((rc = MQTTAsync_disconnect(m_client, &disc_opts)) != MQTTASYNC_SUCCESS)
                {
                    OBNsmn::report_error(0, "MQTT error: Failed to start disconnect with error code = " + std::to_string(rc));
                    MQTTAsync_destroy(&m_client);
                    return;
                }
                
                // Wait until finished
                std::unique_lock<std::mutex> mylock(m_notify_mutex);
                m_notify_done = false;
                m_notify_var.wait(mylock, [this](){ return m_notify_done; });
                
                MQTTAsync_destroy(&m_client);
            }
            
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
    }
}


#endif // OBNSIM_COMM_MQTT_H
