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

#include <chrono>
#include <thread>
#include <obnnode_mqttnode.h>
#include <obnnode_mqttport.h>
#include <obnnode_exceptions.h>

using namespace OBNnode;
using namespace OBNSimMsg;


bool MQTTNodeBase::startMQTT() {
    if (!m_mqtt_client_initialized) {
        m_mqtt_client_initialized = mqtt_client.initialize();
        if (!m_mqtt_client_initialized) {
            return false;
        }
    }
    
    if (mqtt_client.isRunning()) {
        return true;
    } else {
        return mqtt_client.start();
    }
}

bool MQTTNodeBase::addInput(InputPortBase* port, bool owned) {
    // actually add the port
    if (NodeBase::addInput(port, owned)) {
        MQTTInputPortBase* mqttport = dynamic_cast<MQTTInputPortBase*>(port);
        if (mqttport) {
            // Assign the MQTT client
            mqttport->m_mqtt_client = &mqtt_client;
        }
        return true;
    }
    return false;
}

bool MQTTNodeBase::addOutput(OutputPortBase* port, bool owned) {
    // actually add the port
    if (NodeBase::addOutput(port, owned)) {
        MQTTOutputPortBase* mqttport = dynamic_cast<MQTTOutputPortBase*>(port);
        if (mqttport) {
            // Assign the MQTT client
            mqttport->m_mqtt_client = &mqtt_client;
        }
        return true;
    }
    return false;
}

void MQTTNodeBase::removePort(InputPortBase* port) {
    // remove all associated subscriptions if this is an MQTT input port
    MQTTInputPortBase* mqttport = dynamic_cast<MQTTInputPortBase*>(port);
    if (mqttport) {
        mqtt_client.removeSubscription(mqttport);
    }
    
    NodeBase::removePort(port); // actually remove the port
}

void MQTTNodeBase::sendN2SMNMsg() {
    // OBNsim::clockStart = chrono::steady_clock::now();
    
    // Generate the binary content
    m_gcbuffer.allocateData(_n2smn_message.ByteSize());
    bool success = _n2smn_message.SerializeToArray(m_gcbuffer.data(), m_gcbuffer.size());
    success = success && mqtt_client.sendData(m_gcbuffer.data(), m_gcbuffer.size(), m_smn_topic);
    
    // std::cout << "Message sent: " << std::chrono::duration <double, std::nano> (std::chrono::steady_clock::now()-OBNsim::clockStart).count() << " ns\n";

    
    if (!success) {
        // Error while serializing the raw message
        onOBNError("Error while sending a system message to the SMN.");
    }
}


bool MQTTNodeBase::openSMNPort() {
    // Start the MQTT client if needs to
    if (!startMQTT()) {
        return false;
    }
    
    // Subscribe to the SMN topic
    return mqtt_client.addSubscription(&m_smn_port, fullPortName("_gc_")) >= 0;
}


bool MQTTNodeBase::initializeForSimulation() {
    //onReportInfo("MQTT initializeForSimulation.");
    
    // Call the parent's initialization
    if (!NodeBase::initializeForSimulation()) {
        return false;
    }
    
    if (!mqtt_client.isRunning()) {
        onReportInfo("[MQTT] MQTT Client is not running; check the communication network or the MQTT broker.");
        return false;
    }
    
    // Send a message which contains my name to the common MQTT topic to announce that I'm online
    // This needs to be done everytime this node starts a simulation run
    auto namelen = _nodeName.size();
    char myname[namelen];
    _nodeName.copy(myname, namelen);
    
    // Send message: last param is retained = 1
    if (!mqtt_client.sendData(myname, namelen, _workspace + "_smn_/_nodes_/" + _nodeName, 1)) {
        onReportInfo("[MQTT] Could not send the availability announcement; check the communication network or the MQTT broker.");
        return false;
    }
    return true;
}


void MQTTNodeBase::onPermanentCommunicationLost(CommProtocol comm) {
    auto error_message = std::string("Permanent connection lost for protocol ") + (comm==COMM_YARP?"YARP":"MQTT");
    std::cerr << "ERROR: " << error_message << " => Terminate." << std::endl;
    
    if (comm != COMM_MQTT) {
        // We can still try to stop the simulation as the GC's communication (in MQTT) is not affected
        stopSimulation();
        
        // Wait a bit
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Push an error event to the main thread
    postExceptionEvent(std::make_exception_ptr(std::runtime_error(error_message)));
}



/** This method requests a future update from the Global Clock by sending a request to the SMN.
 The request still needs to be approved by the SMN, by an acknowledgement (ACK) message from the SMN.
 The ACK will tell if the request is accepted or rejected (and the reason for the rejection).
 If waiting = true, the function will wait (block) until it receives the ACK; otherwise it will return immediately.
 If the method returns a valid pointer to an WaitForCondition object, the request has been sent; if it returns nullptr, the request is invalid (e.g. a past or present update is requested).
 In case waiting = false, the ACK can be waited for later on by calling wait() on the returned object.
 Once the ACK has been received (the request has been answered), the result of the request can be checked by the I field of the returned data record (accessed by calling getData() on the condition object, see the OBN document for details).
 Make sure that the condition has been cleared (either after waiting for it or by calling MQTTNodeBase::isCleared()) before accessing its data, otherwise the content of the data record is undefined or it may cause a data race issue.
 \param t The time of the requested update; must be in the future t > current simulation time
 \param m The update mask requested for the update.
 \param waiting Whether the method should wait (blocking/synchronously) for the ACK to receive [default: true]; if a timeout is desired, call this function with waiting=false and explicitly call wait() on the returned condition object.
 \return A pointer to the wait-for condition, which is used to wait for the ACK; or nullptr if the request is invalid.
 */
MQTTNodeBase::WaitForCondition* MQTTNodeBase::requestFutureUpdate(simtime_t t, updatemask_t m, bool waiting) {
    if (t <= _current_sim_time) {
        // Cannot request a present or past update time
        return nullptr;
    }
    
    // Send request to the SMN
    _n2smn_message.set_msgtype(OBNSimMsg::N2SMN_MSGTYPE_SIM_EVENT);
    _n2smn_message.set_id(_node_id);
    
    auto *data = new OBNSimMsg::MSGDATA;
    data->set_t(t);
    data->set_i(m);
    _n2smn_message.set_allocated_data(data);
    
    sendN2SMNMsg();
    
    // Register an wait-for condition (lock mutex at beginning and unlock it after we've done)
    MQTTNodeBase::WaitForCondition *pCond = nullptr;
    MQTTNodeBase::WaitForCondition::the_checker f = [t](const OBNSimMsg::SMN2N& msg) {
        return msg.msgtype() == OBNSimMsg::SMN2N_MSGTYPE_SIM_EVENT_ACK && (msg.has_data() && (msg.data().has_t() && msg.data().t() == t));
    };
    
    std::unique_lock<std::mutex> lock(_waitfor_conditions_mutex);
    
    // Look through the list of conditions to find an inactive one
    for (auto c = _waitfor_conditions.begin(); c != _waitfor_conditions.end(); ++c) {
        if (c->status == WaitForCondition::INACTIVE) {
            // Found one => reuse it
            c->_check_func = f;
            c->status = WaitForCondition::ACTIVE;
            pCond = &(*c);
            break;
        }
    }
    
    if (!pCond) {
        // No inactive condition can be reused => create new one
        _waitfor_conditions.emplace_front(f);
        pCond = &(_waitfor_conditions.front());
    }
    
    lock.unlock();     // We've done changing the list

    // If waiting = true, we will wait (blocking) until the wait-for condition is cleared; otherwise, just return
    if (waiting) {
        pCond->wait(-1.0);
    }
    
    return pCond;
}

/** This method returns the result of a pending request for a future update. If the request hasn't been acknowledged by the SMN, this method will wait (block) until it receives the ACK for this request. It returns the value of the I field of the ACK message's data (see the OBN design document for details). Basically if it returns 0, the request was successful; otherwise there was an error and the request failed.
  This method will reset the condition after it's cleared.
 \param pCond Pointer to the condition object, as returned by requestFutureUpdate().
 \param timeout An optional timeout value; if a timeout occurs and the waiting failed then the returned value will be -1.
 \return The result of the request: 0 if successful; -1 if the waiting failed (due to timeout).
 \sa requestFutureUpdate()
 */
int64_t MQTTNodeBase::resultFutureUpdate(MQTTNodeBase::WaitForCondition* pCond, double timeout) {
    return resultWaitForCondition(pCond, timeout);
}

/** This method waits until a wait-for condition is cleared and returns the value of the integer field I of the message data. This method does not check if the return message actually had the message data and the I field; if it did not, the default value (0) is returned.
 This method will reset the condition after it's cleared.
 \param pCond Pointer to the condition object.
 \param timeout An optional timeout value; if a timeout occurs and the waiting failed then the returned value will be -1.
 \return The integer field I of the message data if the waiting is successful (default to 0 if I does not exist); or -1 if the waiting failed (due to timeout).
 */
int64_t MQTTNodeBase::resultWaitForCondition(MQTTNodeBase::WaitForCondition* pCond, double timeout) {
    assert(pCond);
    
    std::unique_lock<std::mutex> lock(_waitfor_conditions_mutex);
    auto s = pCond->status;
    lock.unlock();
    
    assert(s != WaitForCondition::INACTIVE);
    if (s != MQTTNodeBase::WaitForCondition::ACTIVE || pCond->wait(timeout)) {
        // At this point, the status of the condition must be CLEARED => get the data and the I field
        int64_t i;
        {
            std::lock_guard<std::mutex> lockcond(pCond->_mutex);
            i = pCond->getData().i();
        }
        // The following function will need the _mutex of pCond, so must not lock it
        resetWaitFor(pCond);
        return i;
    } else {
        return -1;
    }
}

/** This method iterates the list of wait-for conditions and check if any of them can be cleared according to the given message. If there is one, its status will be changed to CLEARED, the MSGDATA will be saved. At most one condition can be cleared. */
void MQTTNodeBase::checkWaitForCondition(const OBNSimMsg::SMN2N& msg) {
    // Lock access to the list, because the SMN port thread may access it
    std::lock_guard<std::mutex> lock(_waitfor_conditions_mutex);
    
    // Look through the list of conditions
    for (auto c = _waitfor_conditions.begin(); c != _waitfor_conditions.end(); ++c) {
        if (c->status == WaitForCondition::ACTIVE && c->_check_func(msg)) {
            // This condition is cleared
            c->status = WaitForCondition::CLEARED;

            {
                std::lock_guard<std::mutex> lockcond(c->_mutex);
                if (msg.has_data()) {
                    c->_data.CopyFrom(msg.data());
                }
                c->_waitfor_done = true;
            }
            
            c->_event.notify_all();
            return;
        }
    }
}


