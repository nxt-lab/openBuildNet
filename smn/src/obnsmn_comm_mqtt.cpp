/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implement the communication interface in MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnsmn_basic.h>
#include <obnsmn_comm_mqtt.h>

using namespace OBNsmn::MQTT;

// The regex expression to check for topics of node arrival announcements
std::regex OBNsmn::MQTT::MQTTClient::online_nodes_topic_regex("^(\\w+/)?_smn_/_nodes_/(\\w+)$", std::regex::ECMAScript|std::regex::optimize);

bool MQTTClient::start() {
    if (m_running) return false;  // Already running
    if (!pGC) return false;    // pGC must point to a valid GC object
    
    if (m_portName.empty() || m_client_id.empty() || m_server_address.empty()) {
        OBNsmn::report_error(0, "MQTT error: the GC port name and the client ID and the MQTT server address must be set.");
        return false;
    }
    
    int rc;
    
    // Start the client and immediately subscribe to the main GC topic
    if ((rc = MQTTAsync_create(&m_client, m_server_address.c_str(), m_client_id.c_str(), MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTASYNC_SUCCESS) {
        OBNsmn::report_error(0, "MQTT error: could not create MQTT client with error code = " + std::to_string(rc));
        return false;
    }
    
    // Set the callback
    if ((rc = MQTTAsync_setCallbacks(m_client, this, &MQTTClient::on_connection_lost, &MQTTClient::on_message_arrived, &MQTTClient::on_message_delivered)) != MQTTASYNC_SUCCESS)
    {
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
    
    m_running = (m_notify_result == 0);
    
    return m_running;
}


void MQTTClient::stop() {
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


int MQTTClient::sendMessage(char* msg, std::size_t msglen, const std::string& topic, int retained) {
    if (topic.empty()) {
        return false;
    }
    
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    
    // Set the callback opts.onSuccess if we want to make synchronous wait, also the next line
    // opts.context = m_client->m_client;
    
    pubmsg.payload = msg;
    pubmsg.payloadlen = msglen;
    pubmsg.qos = MQTTClient::QOS;
    pubmsg.retained = retained;
    
    ++m_msgout_count;   // Increase the message count (assuming the next function will be successful).
    
    // Request to send the message
    return MQTTAsync_sendMessage(m_client, topic.c_str(), &pubmsg, &opts);
}


int MQTTClient::sendMessage(const OBNSimMsg::SMN2N &msg, const std::string& topic, int retained) {
    if (topic.empty()) {
        return false;
    }
    
    // Allocate buffer to store the bytes of the message
    auto msgsize = msg.ByteSizeLong();
    char* buffer = new char[msgsize];
    
    if (!msg.SerializeToArray(buffer, msgsize)) {
        delete [] buffer;
        return false;
    }

    auto result = sendMessage(buffer, msgsize, topic, retained);
    delete [] buffer;
    return result;
}


bool MQTTClient::sendMessageToGC(const OBNSimMsg::SMN2N &msg) {
    bool result = sendMessage(msg, m_portName);
    bool resultNext = m_prev_gc_sendmsg_to_sys_port?m_prev_gc_sendmsg_to_sys_port(msg):true;
    
    return result && resultNext;
}


/* Start waiting for nodes to announce their arrivals. */
bool MQTTClient::startListeningForArrivals(const std::string& t_workspace) {
    if (!m_running) {
        // The client must already be running
        return false;
    }
    
    if (m_listening_for_arrivals) {
        // Already listening
        return true;
    }
    
    // subscribe to the topic
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    int rc;
    opts.onSuccess = &MQTTClient::onSubscribe;
    opts.onFailure = &MQTTClient::onSubscribeFailure;
    opts.context = this;
    
    auto topic_name = t_workspace + "_smn_/_nodes_/+";  // subscribe to all nodes' announcements
    
    if ((rc = MQTTAsync_subscribe(m_client, topic_name.c_str(), MQTTClient::QOS, &opts)) != MQTTASYNC_SUCCESS)
    {
        OBNsmn::report_error(0, "MQTT error: failed to start subscribe with error code = " + std::to_string(rc));
        return false;
    }
    
    // Wait until subscribed successfully (or failed)
    std::unique_lock<std::mutex> mylock(m_notify_mutex);
    m_notify_done = false;
    m_notify_var.wait(mylock, [this](){ return m_notify_done; });
    
    m_listening_for_arrivals = (m_notify_result == 0);
    m_online_nodes_topic = topic_name;
    
    return m_listening_for_arrivals;
}

/* Stop waiting for nodes to announce their arrivals. */
void MQTTClient::stopListeningForArrivals() {
    if (m_running && m_listening_for_arrivals) {
        // Unsubscribe from the topic
        MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
        int rc;
        // opts.context = this;
        
        if ((rc = MQTTAsync_unsubscribe(m_client, m_online_nodes_topic.c_str(), &opts)) != MQTTASYNC_SUCCESS)
        {
            OBNsmn::report_error(0, "MQTT error: failed to unsubscribe for nodes' arrival announcements with error code = " + std::to_string(rc));
        }
        
        m_listening_for_arrivals = false;
        
        //std::cout << "Stop listening for arrivals\n";
    }
}

/* Check if a given node has announced its arrival. */
bool MQTTClient::checkNodeOnline(const std::string& name) {
    std::lock_guard<std::mutex> mylock(m_online_nodes_mutex);
    return m_online_nodes.count(name) != 0;
}

void MQTTClient::clearListOfOnlineNodes() {
    std::lock_guard<std::mutex> mylock(m_online_nodes_mutex);
    m_online_nodes.clear();
}


void MQTTClient::on_connection_lost(void *context, char *cause)
{
    MQTTClient* client = static_cast<MQTTClient*>(context);
    
    // Retry to connect once
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    
    conn_opts.onSuccess = &MQTTClient::onReconnect;
    conn_opts.onFailure = &MQTTClient::onReconnectFailure;
    conn_opts.context = context;
    
    int rc;
    if ((rc = MQTTAsync_connect(client->m_client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
        // At this point, the client is not running
        client->m_running = false;
        OBNsmn::report_error(0, "MQTT error: could not start reconnect with error code = " + std::to_string(rc));
        client->onPermanentConnectionLost();
    }
}


int MQTTClient::on_message_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message *message)
{    
    // Ignore if the message is a duplicate
    if (!(message->dup)) {
        MQTTClient* client = static_cast<MQTTClient*>(context);
        
        // std::cout << "msgrcvd: " << topicName << "retained " << message->retained << "len " << message->payloadlen << "track " << client->m_listening_for_arrivals << std::endl;
        
        // Check main GC topic
        bool isGCTopic = topicLen>0?(client->m_portName.compare(0, std::string::npos, topicName, topicLen) == 0):(client->m_portName == topicName);
        if (isGCTopic) {
            // Get message and Push to queue
            if (client->m_n2smn_msg.ParseFromArray(message->payload, message->payloadlen)) {
                client->pGC->pushNodeEvent(client->m_n2smn_msg, 0);
            } else {
                OBNsmn::report_error(0, "Critical error: error while parsing input message to MQTT.");
            }
        } else if (message->payloadlen > 0 && client->m_listening_for_arrivals) {
            // If listening for nodes' arrivals, check if this is an arrival announcement
            
            // Copy the topic name to a std::string for easy access
            std::string theTopic;
            if (topicLen == 0) {
                theTopic.assign(topicName);
            } else {
                theTopic.assign(topicName, topicLen);
            }
            
            //std::cout << "Arrival: " << theTopic << "(" << std::string((const char*)message->payload, message->payloadlen) << ") [retained=" << message->retained << "]" << std::endl;
            
            std::smatch m;
            bool isArrivalTopic = std::regex_match(theTopic, m, MQTTClient::online_nodes_topic_regex);
            if (isArrivalTopic) {
                // The node names must match
                if (m.str(2).compare(0, std::string::npos, (const char*)(message->payload), message->payloadlen) == 0) {
                    {
                        // Add this node to the list
                        std::lock_guard<std::mutex> mylock(client->m_online_nodes_mutex);
                        client->m_online_nodes.emplace((const char*)(message->payload), message->payloadlen);
                    }
                    
                    // Send an empty retained message to the topic to delete the retained message on the broker
                    client->sendMessage(nullptr, 0, theTopic, 1);
                }
//                else {
//                    std::cout << "Arrival topic does not match name, got: " << m.str(2) << std::endl;
//                }
            }
//            else {
//                std::cout << "Arrival topic does not match regex\n";
//            }
        }
    }
    
    MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
}


void MQTTClient::on_message_delivered(void *context, MQTTAsync_token token) {
    // std::cout << "Message delivered: " << std::chrono::duration <double, std::nano> (std::chrono::steady_clock::now()-OBNsim::clockStart).count() << " ns\n";
    MQTTClient* client = static_cast<MQTTClient*>(context);
    if (--(client->m_msgout_count) < 0) {
        client->m_msgout_count = 0;     // Must be an internal error
    }
}


void MQTTClient::onConnect(void* context, MQTTAsync_successData* response)
{
    OBNsmn::report_info(0, "MQTT connected.");
    MQTTClient* client = static_cast<MQTTClient*>(context);
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    int rc;

    // Try to subscribe to the main GC topic
    opts.onSuccess = &MQTTClient::onSubscribe;
    opts.onFailure = &MQTTClient::onSubscribeFailure;
    opts.context = context;
    
    if ((rc = MQTTAsync_subscribe(client->m_client, client->m_portName.c_str(), MQTTClient::QOS, &opts)) != MQTTASYNC_SUCCESS)
    {
        OBNsmn::report_error(0, "MQTT error: failed to start subscribe with error code = " + std::to_string(rc));
        client->notify_done(1);
    }
}


void MQTTClient::onConnectFailure(void* context, MQTTAsync_failureData* response)
{
    MQTTClient* client = static_cast<MQTTClient*>(context);
    OBNsmn::report_error(0, "MQTT error: connect failed with error code = " + std::to_string(response ? response->code : 0));
    client->notify_done(1);
}


void MQTTClient::onSubscribe(void* context, MQTTAsync_successData* response) {
    OBNsmn::report_info(0, "MQTT subscribed.");
    MQTTClient* client = static_cast<MQTTClient*>(context);
    
    // At this point, the client is running
    client->m_running = true;
    client->notify_done();
}

void MQTTClient::onSubscribeFailure(void* context, MQTTAsync_failureData* response) {
    MQTTClient* client = static_cast<MQTTClient*>(context);
    OBNsmn::report_error(0, "MQTT error: subscribe failed with error code = " + std::to_string(response ? response->code : 0));
    client->notify_done(1);
}

void MQTTClient::onReconnect(void* context, MQTTAsync_successData* response)
{
    MQTTClient* client = static_cast<MQTTClient*>(context);
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    int rc;
    
    // Try to subscribe to the main GC topic
    opts.onSuccess = NULL;
    opts.onFailure = &MQTTClient::onReSubscribeFailure;
    opts.context = context;
    
    if ((rc = MQTTAsync_subscribe(client->m_client, client->m_portName.c_str(), MQTTClient::QOS, &opts)) != MQTTASYNC_SUCCESS)
    {
        // At this point, the client is not running
        client->m_running = false;
        OBNsmn::report_error(0, "MQTT error: failed to restart subscribe with error code = " + std::to_string(rc));
        client->onPermanentConnectionLost();
    }
}

void MQTTClient::onReconnectFailure(void* context, MQTTAsync_failureData* response)
{
    MQTTClient* client = static_cast<MQTTClient*>(context);
    
    // At this point, the client is not running
    client->m_running = false;
    OBNsmn::report_error(0, "MQTT error: reconnect failed with error code = " + std::to_string(response ? response->code : 0));
    client->onPermanentConnectionLost();
}

void MQTTClient::onReSubscribeFailure(void* context, MQTTAsync_failureData* response) {
    MQTTClient* client = static_cast<MQTTClient*>(context);
    // At this point, the client is not running
    client->m_running = false;
    OBNsmn::report_error(0, "MQTT error: resubscribe failed with error code = " + std::to_string(response ? response->code : 0));
    client->onPermanentConnectionLost();
}

void MQTTClient::onDisconnect(void* context, MQTTAsync_successData* response)
{
    OBNsmn::report_info(0, "MQTT disconnected.");
    MQTTClient* client = static_cast<MQTTClient*>(context);
    client->notify_done();
}


///////////////////////////////////////////////
// Implementation of OBNNodeMQTT
///////////////////////////////////////////////


void OBNNodeMQTT::allocateBuffer(size_t newsize) {
    if (m_buffer && m_buffer_allocsize < newsize) {
        // If _allocsize >= _size, we reuse the memory block
        delete [] m_buffer;
        m_buffer = nullptr;
        m_buffer_allocsize = 0;
    }
    
    if (m_buffer == nullptr && newsize > 0) {
        m_buffer_allocsize = (newsize & 0x0F)?(((newsize >> 4)+1) << 4):newsize;
        m_buffer = new char[m_buffer_allocsize];
    }
}

/** Sends an SMN2N message to a given node asynchronously, i.e. it does not wait for the message to be actually sent to the node.
 The message object (msg) contains the necessary data of the message (type, time, data) which are set by the caller (the GC).
 The ID field (which indicates the ID of the node) is not set by the caller (GC), but may be set by this method depending on the communication protocol. Therefore the message object may be modified upon returning from this method, but it should not cause any problem.
 \param nodeID The node's ID, which is its index in the list of all nodes, managed by the GC.
 \param msg The message object, of type OBNSimMsg::SMN2N, that contains the message data.
 \return True if successful.
 \see sendMessageSync()
 */
bool OBNNodeMQTT::sendMessage(int nodeID, OBNSimMsg::SMN2N &msg) {
    // OBNsim::clockStart = std::chrono::steady_clock::now();
    
    msg.set_id(nodeID);
    
    // Allocate buffer to store the bytes of the message
    auto msgsize = msg.ByteSizeLong();
    allocateBuffer(msgsize);

    if (!msg.SerializeToArray(m_buffer, m_buffer_allocsize)) {
        return false;
    }
    
    // Request to send the message
    int rc;
    if ((rc = m_client->sendMessage(m_buffer, msgsize, m_topic.c_str())) != MQTTASYNC_SUCCESS) {
        OBNsmn::report_error(0, "MQTT error: failed to start sending message to node " + std::to_string(nodeID) +
                             " with error code " + std::to_string(rc));
        return false;
    }
    
    // std::cout << "Message sent: " << std::chrono::duration <double, std::nano> (std::chrono::steady_clock::now()-OBNsim::clockStart).count() << " ns\n";

    return true;
}
