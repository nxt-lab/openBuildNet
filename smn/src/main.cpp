/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief The main SMN program.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>
#include <thread>
#include <obnsmn_report.h>
#include <obnsmn_gc.h>   // The GC thread


#ifdef OBNSIM_SMN_COMM_YARP
#include <obnsim_comm_yarp.h>
#endif

using namespace OBNsmn;

// Implement reporting functions for the SMN
void OBNsmn::report_error(int code, std::string msg) {
    std::cerr << "ERROR (" << code << "): " << msg << std::endl;
}

void OBNsmn::report_warning(int code, std::string msg) {
    std::cout << "WARNING (" << code << "): " << msg << std::endl;
}

void OBNsmn::report_info(int code, std::string msg) {
    std::cout << "INFO (" << code << "): " << msg << std::endl;
}



/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNSIM_SMN_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_SMN_COMM_MQTT: if MQTT is supported for communication.
 */

class NodeYARPMsg : public yarp::os::Portable {
public:
    virtual bool write(yarp::os::ConnectionWriter& connection) {
        connection.appendInt(_size);
        connection.appendExternalBlock(_data, _size);
        return true;
    }
    virtual bool read(yarp::os::ConnectionReader& connection);
    
    /** \brief Set the contents of the N2SMN message. */
    bool setMessage(const OBNSimMsg::N2SMN &msg) {
        allocateData(msg.ByteSize());
        return msg.SerializeToArray(_data, _size);
    }
    
    virtual ~NodeYARPMsg() {
        if (_data) {
            delete [] _data;
        }
    }
private:
    /** The binary data of the message */
    char* _data = nullptr;
    
    /** The actual size of the message, not exceeding the allocated size. */
    size_t _size;
    
    /** The size of the allocated memory buffer (_data). */
    size_t _allocsize = 0;
    
    /** Allocate the memory block _data given the new size. It will reuse memory if possible. It will change _size. */
    void allocateData(size_t newsize);
};

bool NodeYARPMsg::read(yarp::os::ConnectionReader& connection) {
    if (connection.isError()) {
        return false;
    }
    
    int newsize = connection.expectInt();
    if (newsize <= 0) {
        _size = 0;
        return false;
    }
    
    allocateData(newsize);
    
    return connection.expectBlock(_data, _size);
}


void NodeYARPMsg::allocateData(size_t newsize) {
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

int main() {
    
#ifdef OBNSIM_SMN_COMM_YARP
    std::cout << "SMN supports YARP." << std::endl;
#endif

#ifdef OBNSIM_SMN_COMM_MQTT
    std::cout << "SMN supports MQTT." << std::endl;
#endif
    yarp::os::Network yarp;
    
    YARP::YARPPort gcPort;
    yarp::os::BufferedPort<NodeYARPMsg> nodePort;
    
    bool ok = gcPort.open("/test1/_gc_/node1");
    ok = ok && nodePort.open("/test1/_node1_");
    
    if (!ok) {
        std::cout << "Failed to create ports." << std::endl;
        return 1;
    }

    // The Global clock thread
    GCThread gc;
    
    // In this test, we create a few nodes
    OBNsmn::YARP::OBNNodeYARP* pnode = new OBNsmn::YARP::OBNNodeYARP("Node1", 2, &gcPort);
    gc.insertNode(pnode);
    pnode->setOutputGroup(0, 10, 0x01);
    pnode->setOutputGroup(1, 15, 0x02);
    
    // Start running the GC thread
    if (!gc.startThread()) {
        std::cout << "Error: cannot start GC thread." << std::endl;
        return 1;
    }

    // The YARP communication thread for GC's incoming port
    YARP::YARPPollingThread yarpThread(&gc, "/test1/_gc_");
    if (!yarpThread.startThread()) {
        std::cout << "Error: cannot start GC thread." << std::endl;
        return 1;
    }
    
    yarp.connect("/test1/_gc_/node1", "/test1/_node1_");
    yarp.connect("/test1/_node1_", "/test1/_gc_");
    
    // The main thread will check for input from the user
    char userCmd;
    bool simRunning = true;
    OBNSimMsg::N2SMN mymsg;

    while (simRunning) {
        std::cout << "Enter command [(s)top, (t)erminate] or any character: ";
        std::cin >> userCmd;
        
        // STOP means nicely stopping the simulation, i.e. allow remaining events to be processed
        if (userCmd == 's' || userCmd == 'S') {
            gc.setSysRequest(GCThread::SYSREQ_STOP);
            simRunning = false;
        }
        
        // TERMINATE means abrubtly stopping the simulation, discarding remaining events
        if (userCmd == 't' || userCmd == 'T') {
            gc.setSysRequest(GCThread::SYSREQ_TERMINATE);
            simRunning = false;
        }
        
        if (simRunning) {
            mymsg.Clear();
            if (userCmd == 'a' || userCmd == 'A') {
                mymsg.set_id(1);
                mymsg.set_msgtype(OBNSimMsg::N2SMN::SIM_Y_ACK);
            } else {
                mymsg.set_id(1);
                mymsg.set_msgtype(OBNSimMsg::N2SMN::SIM_EVENT);
                OBNSimMsg::MSGDATA* mydata = new OBNSimMsg::MSGDATA();
                mydata->set_i(1980);
                mymsg.set_allocated_data(mydata);
            }
            
            NodeYARPMsg& mypacket = nodePort.prepare();
            mypacket.setMessage(mymsg);
            nodePort.writeStrict();
        }
    }
    
    std::cout << "Waiting for simulation to stop..." << std::endl;
    
    //Join the threads with the main thread
    gc.joinThread();
    yarpThread.joinThread();


    // Test the graph implementation
    std::unique_ptr<NodeDepGraph> _nodeGraph(new NodeDepGraph_BGL(3));
    
    // Add dependency between nodes
    _nodeGraph->addDependency(0, 1, 0x03);
    _nodeGraph->addDependency(0, 2, 0x01);
    _nodeGraph->addDependency(1, 2, 0x02);
    
 
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
