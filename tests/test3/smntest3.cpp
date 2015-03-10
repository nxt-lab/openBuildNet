/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A test SMN.
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
#include <array>
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
    
    const int N = 5;  // number of slaves
    
    // The individual ports to send to nodes
    OBNsmn::YARP::YARPPort admmPort;
    
    bool ok = admmPort.open("/test3/_gc_/admm");
    
    std::array<OBNsmn::YARP::YARPPort, N> slavePorts;
    for (int i = 0; i < N; ++i) {
        ok = ok && slavePorts[i].open("/test3/_gc_/x" + std::to_string(i));
    }
    
    if (!ok) {
        std::cout << "Failed to create ports." << std::endl;
        return 1;
    }

    // The Global clock thread
    OBNsmn::GCThread gc;

    // ======== Creating node =========
    
    // In this test, we create an ADMM master node
    OBNsmn::YARP::OBNNodeYARP* pnode = new OBNsmn::YARP::OBNNodeYARP("admm", 1, &admmPort);
    pnode->setOutputGroup(0, 1000);  // bit mask 0
    gc.insertNode(pnode);
    
    for (int i = 0; i < N; ++i) {
        pnode = new OBNsmn::YARP::OBNNodeYARP("x" + std::to_string(i), 1, &slavePorts[i]);
        pnode->setOutputGroup(0, 1000);
        gc.insertNode(pnode);
    }
    
    // NOTE the index: 0 - admm
    OBNsmn::NodeDepGraph* nodeGraph = new OBNsmn::NodeDepGraph_BGL(1 + N);
    
    gc.setDependencyGraph(nodeGraph);
    
    // ========== End creating nodes ===========
    
    // Configure the GC
    gc.ack_timeout = 0;
    gc.setFinalSimulationTime(4900);

    
    // The YARP communication thread for GC's incoming port
    OBNsmn::YARP::YARPPollingThread yarpThread(&gc, "/test3/_gc_");
    if (!yarpThread.startThread()) {
        std::cout << "Error: cannot start GC thread." << std::endl;
        return 1;
    }

    // The GC has a dedicated output port for each node
    yarp.connect("/test3/_gc_/admm", "/test3/_admm_");
    // However, each node has only one IO port to communicate with GC, and all nodes send to the same GC input port
    yarp.connect("/test3/_admm_", "/test3/_gc_");

    for (int i = 0; i < N; ++i) {
        std::string slaveName = "x" + std::to_string(i);
        yarp.connect("/test3/_gc_/" + slaveName, "/test3/_" + slaveName + "_");
        yarp.connect("/test3/_" + slaveName + "_", "/test3/_gc_");
        
        // Connect the master and slave
        yarp.connect("/test3/admm/" + slaveName, "/test3/" + slaveName + "/x");
        yarp.connect("/test3/" + slaveName + "/x", "/test3/admm/" + slaveName);
    }
    
    
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
