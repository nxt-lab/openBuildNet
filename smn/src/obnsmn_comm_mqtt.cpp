/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implement the communication interface in MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnsmn_comm_mqtt.h>

using namespace OBNsmn::MQTT;

bool MQTTClient::sendMessage(const OBNSimMsg::SMN2N &msg) {
    // Call the previous function if available
    if (m_prev_gc_sendmsg_to_sys_port) {
        return m_prev_gc_sendmsg_to_sys_port(msg);
    }
    
    return true;
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
    MQTTClient* client = static_cast<MQTTClient*>(context);
    
    // Check main GC topic
    bool isTopic = topicLen>0?(client->m_portName.compare(0, std::string::npos, topicName, topicLen) == 0):(client->m_portName == topicName);
    if (isTopic) {
        // Get message and Push to queue
        if (client->m_n2smn_msg.ParseFromArray(message->payload, message->payloadlen)) {
            client->pGC->pushNodeEvent(client->m_n2smn_msg, 0);
        } else {
            OBNsmn::report_error(0, "Critical error: error while parsing input message to MQTT.");
        }
    }
    MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
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
        client->notify_done();
    }
}


void MQTTClient::onConnectFailure(void* context, MQTTAsync_failureData* response)
{
    MQTTClient* client = static_cast<MQTTClient*>(context);
    OBNsmn::report_error(0, "MQTT error: connect failed with error code = " + std::to_string(response ? response->code : 0));
    client->notify_done();
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
    client->notify_done();
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
    msg.set_id(nodeID);
    
    // Allocate buffer to store the bytes of the message
    auto msgsize = msg.ByteSize();
    allocateBuffer(msgsize);

    if (!msg.SerializeToArray(m_buffer, m_buffer_allocsize)) {
        return false;
    }
    
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    
    // Set the callback opts.onSuccess if we want to make synchronous wait, also the next line
    // opts.context = m_client->m_client;
    
    pubmsg.payload = m_buffer;
    pubmsg.payloadlen = msgsize;
    pubmsg.qos = MQTTClient::QOS;
    pubmsg.retained = 0;
    
    // Request to send the message
    int rc;
    if ((rc = MQTTAsync_sendMessage(m_client->m_client, m_topic.c_str(), &pubmsg, &opts)) != MQTTASYNC_SUCCESS) {
        OBNsmn::report_error(0, "MQTT error: failed to start sending message to node " + std::to_string(nodeID) +
                             " with error code " + std::to_string(rc));
        return false;
    }
    
    return true;
}
