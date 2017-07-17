/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implement the communication interface in MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnnode_mqttport.h>
#include <algorithm>        // std::find

//#define MQTT_PRINT_DEBUG

using namespace OBNnode;

const int MQTTClient::QOS = 2;

bool MQTTClient::initialize() {
    if (m_client_id.empty() || m_server_address.empty()) {
        return false;
    }
    
    int rc;
    
#ifdef MQTT_PRINT_DEBUG
    std::cout << "MQTT with address: " << m_server_address << " and Client ID: " << m_client_id << std::endl;
#endif
    
    if ((rc = MQTTAsync_create(&m_client, m_server_address.c_str(), m_client_id.c_str(), MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTASYNC_SUCCESS)
    {
        return false;
    }
    
    // Set the callback
    if ((rc = MQTTAsync_setCallbacks(m_client, this, &MQTTClient::on_connection_lost, &MQTTClient::on_message_arrived, NULL)) != MQTTASYNC_SUCCESS)  // &MQTTClient::on_message_delivered
    {
        return false;
    }
    
    return true;
}

bool MQTTClient::start() {
    if (isRunning()) return false;  // Already running
    
    int rc;
    
    // Connect
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.onSuccess = &MQTTClient::onConnect;
    conn_opts.onFailure = &MQTTClient::onConnectFailure;
    conn_opts.context = this;
    if ((rc = MQTTAsync_connect(m_client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
#ifdef MQTT_PRINT_DEBUG
        std::cout << "[MQTT] Request to connect failed: " << rc << std::endl;
#endif
        return false;
    }
    
    // Wait until connecting successfully (result = 0) or failed (result != 0)
    std::unique_lock<std::mutex> mylock(m_notify_mutex);
    m_notify_done = 1;
    m_notify_var.wait(mylock, [this](){ return (m_notify_done == 0); });
    
    // If the connection failed, we disconnect and destroy the client object
    if (m_notify_result != 0) {
        stop();
        return false;
    }
    
    return true;
}


void MQTTClient::stop() {
    if (!isRunning()) {
        return;
    }
    
    // Disconnect from the server
    MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer;
    disc_opts.onSuccess = &MQTTClient::onDisconnect;
    disc_opts.context = this;
    {
        std::unique_lock<std::mutex> mylock(m_notify_mutex);
        m_notify_done = 1;
    }
    if (MQTTAsync_disconnect(m_client, &disc_opts) == MQTTASYNC_SUCCESS)
    {
        // Wait until finished
        std::unique_lock<std::mutex> mylock(m_notify_mutex);
        if (m_notify_done != 1) {
            m_notify_var.wait(mylock, [this](){ return (m_notify_done == 0); });
        }
    }
    
    MQTTAsync_destroy(&m_client);
}


bool MQTTClient::sendData(void *data, int size, const std::string& topic, int retained) {
    if (topic.empty() || data == nullptr || size <= 0) {
        return false;
    }

    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    
    // Set the callback opts.onSuccess if we want to make synchronous wait, also the next line
    // opts.context = m_client->m_client;
    
    pubmsg.payload = data;
    pubmsg.payloadlen = size;
    pubmsg.qos = MQTTClient::QOS;
    pubmsg.retained = retained;
    
    // Request to send the message
    return (MQTTAsync_sendMessage(m_client, topic.c_str(), &pubmsg, &opts) == MQTTASYNC_SUCCESS);
}


int MQTTClient::addSubscription(IMQTTInputPort* port, const std::string& topic) {
    if (topic.empty() || port == nullptr) {
        return -3;
    }
    
    std::unique_lock<std::mutex> lock(m_topics_mutex);
    
    // Find or insert the topic in the map
    auto found = m_topics.find(topic);
    if (found != m_topics.end()) {
        // Found the topic: find if port is already in the list
        auto foundport = std::find(found->second.begin(), found->second.end(), port);
        if (foundport != found->second.end()) {
            return 1;
        } else {
            // Add the port to the list
            found->second.push_back(port);
            return 0;
        }
    } else {
        m_topics.emplace(topic, decltype(m_topics)::mapped_type(1, port));
        lock.unlock();      // Finish using it
        
        // Subscribe to the topic if the client is running
        if (isRunning()) {
            std::unique_lock<std::mutex> mylock(m_notify_mutex);
            m_notify_done = 1;
            subscribeTopic(topic);
            m_notify_var.wait(mylock, [this](){ return (m_notify_done == 0); });

            return (m_notify_result == 0)?0:-2;
        }
        return 0;
    }
}

void MQTTClient::removeSubscription(IMQTTInputPort* port) {
    if (port == nullptr) {
        return;
    }
    
    std::unique_lock<std::mutex> lock(m_topics_mutex);
    
    // Find the given port in all topics and remove it
    for (auto topic = m_topics.begin(); topic != m_topics.end(); ++topic) {
        auto found = std::find(topic->second.begin(), topic->second.end(), port);
        if (found != topic->second.end()) {
            // Found it
            if (topic->second.size() == 1) {
                // This is the only element (most of the cases) => unsubscribe and delete the topic
                unsubscribeTopic(topic->first);
                m_topics.erase(topic);
            } else {
                // Only remove the port from the topic
                topic->second.erase(found);
            }
        }
    }
}

void MQTTClient::unsubscribeTopic(const std::string& topic) {
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    // int rc;
    
    if ((MQTTAsync_unsubscribe(m_client, topic.c_str(), &opts)) != MQTTASYNC_SUCCESS) {
        // Currently we don't do anything if there is an error unsubscribing from the topic
    }
}

void MQTTClient::subscribeTopic(const std::string& topic) {
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    
    opts.onSuccess = &MQTTClient::onSubscribe;
    opts.onFailure = &MQTTClient::onSubscribeFailure;
    opts.context = this;
    
    if (MQTTAsync_subscribe(m_client, topic.c_str(), MQTTClient::QOS, &opts) != MQTTASYNC_SUCCESS) {
        // Signal an error
        m_notify_result = 1;
        notify_done();
    }
}

void MQTTClient::subscribeAllTopics(bool resubscribe) {
    /* Set m_notify_result = 0.
     We will use MQTTAsync_subscribeMany() to subscribe to all topics in one shot, and only one call to either of the success or failure callback is carried out.
     After m_notify_done = 0, the original caller (which started connection) will be notified, and it should check m_notify_result for the overall result.
     For resubscriptions, if any error happens, client->onPermanentConnectionLost() should be called to report permanent connection error.
     */
    
    m_notify_result = 0;
    
    // Construct the list of topics
    std::unique_lock<std::mutex> lock(m_topics_mutex);
    int count = m_topics.size();
    if (count == 0) {
        notify_done();
        return;
    }

    // Copy the keys out
    char** topics = new char*[count];
    {
        auto it = topics;
        std::size_t slen;
        for (const auto& t: m_topics) {
            slen = t.first.length() + 1;
            *it = new char[slen];
            std::strcpy(*(it++), t.first.c_str());
        }
    }
    lock.unlock();  // We don't need access to m_topics anymore
    
    // Construct array of QOS values
    std::vector<int> qos(count, MQTTClient::QOS);
    
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    if (resubscribe) {
        // For resubscribing we can use onSubscribe as success callback because it will just notify the success
        opts.onSuccess = &MQTTClient::onSubscribe;
        opts.onFailure = &MQTTClient::onReSubscribeFailure;
    } else {
        opts.onSuccess = &MQTTClient::onSubscribe;
        opts.onFailure = &MQTTClient::onSubscribeFailure;
    }
    opts.context = this;
    
    int rc = MQTTAsync_subscribeMany(m_client, count, topics, qos.data(), &opts);

    // Delete topics
    for (int k = 0; k < count; ++k) {
        delete [] topics[k];
    }
    delete [] topics;
    
    if (rc != MQTTASYNC_SUCCESS) {
#ifdef MQTT_PRINT_DEBUG
        std::cout << "[MQTT] Request to subscribe failed.\n";
#endif
        m_notify_result = 1;    // error
        // Notify waiting threads
        notify_done();
    }

    // We won't notify in case of success because the callbacks will do this
}

void MQTTClient::on_connection_lost(void *context, char *cause)
{
#ifdef MQTT_PRINT_DEBUG
    std::cout << "Connection lost\n";
#endif
    
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
        client->onPermanentConnectionLost();
    }
}


int MQTTClient::on_message_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message *message)
{
    // Ignore if the message is a duplicate
    if (!(message->dup)) {
        MQTTClient* client = static_cast<MQTTClient*>(context);
        
        // copy the topic to C++ string
        std::string topic;
        if (topicLen <= 0) {
            topic.assign(topicName);
        } else {
            topic.assign(topicName, topicLen);
        }
        
        // Find if the topic is in the subscription list
        std::lock_guard<std::mutex> lock(client->m_topics_mutex);
        auto found = client->m_topics.find(topic);
        if (found != client->m_topics.end()) {
            // Ask the subscribing ports to process the message
            for (auto& port: found->second) {
                port->parse_message(message->payload, message->payloadlen);
            }
        } else {
            // Not found
            if (client->m_node) {
                client->m_node->onOBNWarning("Unrecognized topic sent to MQTT: " + topic);
            }
        }
    }
    
    MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
}


void MQTTClient::on_message_delivered(void *context, MQTTAsync_token token) {
//    std::cout << "Message delivered: " << std::chrono::duration <double, std::nano> (std::chrono::steady_clock::now()-OBNsim::clockStart).count() << " ns\n";
}


void MQTTClient::onConnect(void* context, MQTTAsync_successData* response)
{
#ifdef MQTT_PRINT_DEBUG
    std::cout << "[MQTT] Connected\n";
#endif
    
    MQTTClient* client = static_cast<MQTTClient*>(context);

    // Try to subscribe to topics
    client->subscribeAllTopics();
    
    // We don't notify here as the callbacks will do it
}


void MQTTClient::onConnectFailure(void* context, MQTTAsync_failureData* response)
{
#ifdef MQTT_PRINT_DEBUG
    std::cout << "[MQTT] Connect failed";
    if (response != nullptr) {
        std::cout << " with code: " << response->code;
        if (response->message) {
            std::cout << " and message: " << response->message << std::endl;
        } else {
            std::cout << std::endl;
        }
    }  else {
        std::cout << std::endl;
    }
#endif
    
    MQTTClient* client = static_cast<MQTTClient*>(context);
    // Notify of error
    client->m_notify_result = 1;
    client->notify_done();
}


void MQTTClient::onSubscribe(void* context, MQTTAsync_successData* response) {
#ifdef MQTT_PRINT_DEBUG
    std::cout << "[MQTT] Subscribed.\n";
#endif
    
    MQTTClient* client = static_cast<MQTTClient*>(context);
    // Notify of success
    client->m_notify_result = 0;
    client->notify_done();
}

void MQTTClient::onSubscribeFailure(void* context, MQTTAsync_failureData* response) {
#ifdef MQTT_PRINT_DEBUG
    std::cout << "[MQTT] Subscribe failed";
    if (response != nullptr) {
        std::cout << " with code: " << response->code;
        if (response->message) {
            std::cout << " and message: " << response->message << std::endl;
        } else {
            std::cout << std::endl;
        }
    }  else {
        std::cout << std::endl;
    }
#endif
    
    MQTTClient* client = static_cast<MQTTClient*>(context);
    // Notify of failure
    client->m_notify_result = 2;
    client->notify_done();
}

void MQTTClient::onReconnect(void* context, MQTTAsync_successData* response)
{
    // std::cout << "Reconnected\n";
    
    MQTTClient* client = static_cast<MQTTClient*>(context);
    
    // Try to resubscribe to topics
    // We don't wait for the result here because this callback is currently running on the MQTT thread
    // If we wait, the thread will pause and further callbacks won't be called.
    client->subscribeAllTopics(true);
}

void MQTTClient::onReconnectFailure(void* context, MQTTAsync_failureData* response)
{
    // std::cout << "Reconnect failed.\n";
    
    MQTTClient* client = static_cast<MQTTClient*>(context);
    
    // At this point, the client is not running
    client->onPermanentConnectionLost();
}

void MQTTClient::onReSubscribeFailure(void* context, MQTTAsync_failureData* response) {
    MQTTClient* client = static_cast<MQTTClient*>(context);
    
    // At this point, the client is not running
    client->onPermanentConnectionLost();
}

void MQTTClient::onDisconnect(void* context, MQTTAsync_successData* response)
{
    MQTTClient* client = static_cast<MQTTClient*>(context);
    client->m_notify_result = 0;
    client->notify_done();
}


///////////////////////////////////////////////
// Implementation of MQTTGCPort
///////////////////////////////////////////////
void MQTTGCPort::parse_message(void *msg, int msglen) {
    // printf("MQTT GC Port callback\n");
    
    // Parse the ProtoBuf message
    if (msg != nullptr && msglen > 0) {
        if (m_smn_msg.ParseFromArray(msg, msglen)) {
            // OK -> push the event
            m_node->postEvent(m_smn_msg);
        } else {
            // Problem
            m_node->onOBNError("Error while parsing a system message from the SMN.");
        }
    }
}


///////////////////////////////////////////////
// Implementation of MQTT base port classes
///////////////////////////////////////////////

std::pair<int, std::string> MQTTInputPortBase::connect_from_port(const std::string& source) {
    assert(!source.empty());

    if (!m_mqtt_client) {
        return std::make_pair(-2, "Internal error of MQTT port: MQTTClient is null.");
    }
    
    // Add the subscription to the client
    return std::make_pair(m_mqtt_client->addSubscription(this, source), "");
}
