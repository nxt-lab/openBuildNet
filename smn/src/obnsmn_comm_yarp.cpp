/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implement the communication interface in YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnsmn_comm_yarp.h>
#include <obnsmn_report.h>

using namespace OBNsmn::YARP;

void YARPMsg::allocateData(size_t newsize) {
    assert(newsize > 0);
    
    _size = newsize;
    
    if (_data) {
        // If _allocsize >= _size, we reuse the memory block
        if (_allocsize < _size) {
            delete [] _data;
        }
    }
    else {
        _allocsize = 0;  // Make sure that _allocsize < _size
    }
    
    if (_allocsize < _size) {
        _allocsize = (_size & 0x0F)?(((_size >> 4)+1) << 4):_size;
        assert(_allocsize >= _size);
        _data = new char[_allocsize];
    }
}

bool YARPMsg::read(yarp::os::ConnectionReader& connection) {
    if (connection.isError()) {
        return false;
    }
    
    int newsize = connection.expectInt();
    if (newsize <= 0) {
        _size = 0;
        return newsize<0?false:true;
    }
    
    allocateData(newsize);
    
    return connection.expectBlock(_data, _size);
}


//void YARPPort::onRead(YARPMsg& b) {
//    // Get the N2SMN mnessage from b
//    if (!b.getMessage(msg)) {
//        // We should report an error here
//        std::cout << "Critical error: error while parsing input message from node " << nodeID << std::endl;
//    }
//    
//    pGC->pushNodeEvent(msg, nodeID, true);
//}


/** Sends an SMN2N message to a given node asynchronously, i.e. it does not wait for the message to be actually sent to the node.
The message object (msg) contains the necessary data of the message (type, time, data) which are set by the caller (the GC).
The ID field (which indicates the ID of the node) is not set by the caller (GC), but may be set by this method depending on the communication protocol. Therefore the message object may be modified upon returning from this method, but it should not cause any problem.
\param nodeID The node's ID, which is its index in the list of all nodes, managed by the GC.
\param msg The message object, of type OBNSimMsg::SMN2N, that contains the message data.
\return True if successful.
\see sendMessageSync()
*/
bool OBNNodeYARP::sendMessage(int nodeID, OBNSimMsg::SMN2N &msg) {
    msg.set_id(nodeID);
    
    // Prepare the data to send
    YARPMsg &m = port->prepare();
    if (!m.setMessage(msg)) {
        return false;
    }
    
    // Send and strictly make sure that the message will not be dropped
    port->writeStrict();
    
    return true;
}


// NOT USED
/* Sends an SMN2N message to a given node synchronously, i.e. it will wait until the message has been sent to the node.
The method may time out, depending on the configuration of the communication framework.
The message object (msg) contains the necessary data of the message (type, time, data) which are set by the caller (the GC).
The ID field (which indicates the ID of the node) is not set by the caller (GC), but may be set by this method depending on the communication protocol. Therefore the message object may be modified upon returning from this method, but it should not cause any problem.
\param nodeID The node's ID, which is its index in the list of all nodes, managed by the GC.
\param msg The message object, of type OBNSimMsg::SMN2N, that contains the message data.
\return True if successful.
\see sendMessage()
*/
//bool OBNNodeYARP::sendMessageSync(int nodeID, OBNSimMsg::SMN2N &msg) {
//    msg.set_id(nodeID);
//    
//    // Prepare the data to send
//    YARPMsg &m = port->prepare();
//    if (!m.setMessage(msg)) {
//        return false;
//    }
//    
//    // Send and strictly make sure that the message will not be dropped
//    port->writeStrict();
//    
//    // Wait until sent
//    port->waitForWrite();
//
//    return true;
//}


bool YARPPollingThread::sendMessage(const OBNSimMsg::SMN2N &msg) {
    if (port.isClosed()) {
        return false;
    }
    
    // Prepare the data to send
    YARPMsg &m = port.prepare();
    if (!m.setMessage(msg)) {
        return false;
    }
    
    // Send and strictly make sure that the message will not be dropped
    port.writeStrict();
    
    // Call the previous function if available
    if (m_prev_gc_sendmsg_to_sys_port) {
        return m_prev_gc_sendmsg_to_sys_port(msg);
    }
    
    return true;
}

void YARPPollingThread::ThreadMain() {
    done_execution = false;
    
    // This thread simply polls the main GC port
    OBNSimMsg::N2SMN msg;
    
    while (!pGC->simple_thread_terminate) {
        YARPMsg *b = port.read(false);
        if (b) {
            // data received in *b
            if (b->getMessage(msg)) {
                pGC->pushNodeEvent(msg, 0);
            } else {
                // We should report an error here
                OBNsmn::report_error(0, "Critical error: error while parsing input message to Yarp.");
            }
        }
    }
    
    // Close the port after finishing
    port.close();
    
    done_execution = true;
}