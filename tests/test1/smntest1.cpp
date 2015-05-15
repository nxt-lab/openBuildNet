/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A test SMN which implements nodes inside the SMN itself.
 *
 * Requires YARP.
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

#ifndef OBNSIM_COMM_YARP
#error This test requires YARP to run
#endif

#include <obnsmn_comm_yarp.h>

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
 - OBNSIM_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_SMN_COMM_MQTT: if MQTT is supported for communication.
 */


int main() {
    yarp::os::Network yarp;
    
    OBNsmn::YARP::YARPPort gcPort;
    
    bool ok = gcPort.open("/test1/_smn_/node1");
    if (!ok) {
        std::cout << "Failed to create ports." << std::endl;
        return 1;
    }

    // The Global clock thread
    OBNsmn::GCThread gc;

    // ======== Creating node =========
    
    // In this test, we create one node
    OBNsmn::YARP::OBNNodeYARP* pnode = new OBNsmn::YARP::OBNNodeYARP("node1", 2, &gcPort);
    pnode->setUpdateType(0, 10);  // bit mask 0
    pnode->setUpdateType(1, 0);  // bit mask 1 -> irregular update

    gc.insertNode(pnode);
    
    OBNsmn::NodeDepGraph* nodeGraph = new OBNsmn::NodeDepGraph_BGL(1);
    gc.setDependencyGraph(nodeGraph);
    
    // ========== End creating nodes ===========
    
    // Configure the GC
    gc.ack_timeout = 0;
    gc.setFinalSimulationTime(100);

    
    // The YARP communication thread for GC's incoming port
    OBNsmn::YARP::YARPPollingThread yarpThread(&gc, "/test1/_smn_/_gc_");
    if (!yarpThread.startThread()) {
        std::cout << "Error: cannot start GC thread." << std::endl;
        return 1;
    }

    // The GC has a dedicated output port for each node
    yarp.connect("/test1/_smn_/node1", "/test1/node1/_gc_");
    
    // However, each node has only one IO port to communicate with GC, and all nodes send to the same GC input port
    yarp.connect("/test1/node1/_gc_", "/test1/_smn_/_gc_");
    
    // Start running the GC thread
    if (!gc.startThread()) {
        std::cout << "Error: cannot start GC thread." << std::endl;
        return 1;
    }
    
    
    // The main thread will only wait until the GC thread stops
    std::cout << "In this test, we only wait until the simulation stops..." << std::endl;
    
    //Join the threads with the main thread
    gc.joinThread();
    yarpThread.joinThread();
 
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
