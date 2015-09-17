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
//    int i;
//    char* payloadptr;
//    
//    printf("Message arrived\n");
//    printf("     topic: %s\n", topicName);
//    printf("   message: ");
//    
//    payloadptr = message->payload;
//    for(i=0; i<message->payloadlen; i++)
//    {
//        putchar(*payloadptr++);
//    }
//    putchar('\n');
//    MQTTAsync_freeMessage(&message);
//    MQTTAsync_free(topicName);
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