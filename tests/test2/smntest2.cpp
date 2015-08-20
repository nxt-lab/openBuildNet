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
    
    // The individual ports to send to nodes
    OBNsmn::YARP::YARPPort motorPort, ctrlPort, spPort;
    
    bool ok = motorPort.open("/test2/_smn_/motor") && ctrlPort.open("/test2/_smn_/ctrl") && spPort.open("/test2/_smn_/sp");
    if (!ok) {
        std::cout << "Failed to create ports." << std::endl;
        return 1;
    }

    // The Global clock thread
    OBNsmn::GCThread gc;

    // ======== Creating node =========
    
    // In this test, we create three nodes: motor, controller, and setpoint source
    OBNsmn::YARP::OBNNodeYARP* pnode = new OBNsmn::YARP::OBNNodeYARP("motor", 1, &motorPort);
    pnode->setUpdateType(0, 10);  // bit mask 0
    gc.insertNode(pnode);
    
    pnode = new OBNsmn::YARP::OBNNodeYARP("ctrl", 1, &ctrlPort);
    pnode->setUpdateType(0, 10);  // bit mask 0
    gc.insertNode(pnode);
    
    pnode = new OBNsmn::YARP::OBNNodeYARP("setpoint", 1, &spPort);
    pnode->setUpdateType(0, 200);   // bit mask 0
    pnode->needUPDATEX = false;     // this setpoint node doesn't need UPDATE_X
    gc.insertNode(pnode);
    
    // NOTE the index: 0 - motor, 1 - ctrl, 2 - setpoint
    OBNsmn::NodeDepGraph* nodeGraph = new OBNsmn::NodeDepGraph_BGL(3);
    
    // We can try different tests: no dependency ...
    // ... or (setpoint -> controller, motor)
    // nodeGraph->addDependency(2, 1, 0x01, 0x01);
    // ... or (setpoint -> controller, controller -> motor)
    nodeGraph->addDependency(2, 1, 0x01, 0x01); nodeGraph->addDependency(1, 0, 0x01, 0x01);
    
    gc.setDependencyGraph(nodeGraph);
    
    // ========== End creating nodes ===========
    
    // Configure the GC
    gc.ack_timeout = 0;
    gc.setFinalSimulationTime(1000);

    
    // The YARP communication thread for GC's incoming port
    OBNsmn::YARP::YARPPollingThread yarpThread(&gc, "/test2/_smn_/_gc_");
    if (!yarpThread.startThread()) {
        std::cout << "Error: cannot start GC thread." << std::endl;
        return 1;
    }

    // The GC has a dedicated output port for each node
    yarp.connect("/test2/_smn_/motor", "/test2/motor/_gc_");
    yarp.connect("/test2/_smn_/ctrl", "/test2/ctrl/_gc_");
    yarp.connect("/test2/_smn_/sp", "/test2/sp/_gc_");
    
    // However, each node has only one IO port to communicate with GC, and all nodes send to the same GC input port
    yarp.connect("/test2/motor/_gc_", "/test2/_smn_/_gc_");
    yarp.connect("/test2/ctrl/_gc_", "/test2/_smn_/_gc_");
    yarp.connect("/test2/sp/_gc_", "/test2/_smn_/_gc_");
    
    // Now, connect the inputs and outputs of the node
    yarp.connect("/test2/sp/sp", "/test2/ctrl/sp");  // sp.sp -> ctrl.sp
    yarp.connect("/test2/motor/v", "/test2/ctrl/v");  // motor.velo -> ctrl.velo
    yarp.connect("/test2/ctrl/u", "/test2/motor/vol");  // ctrl.u -> motor.voltage
    
    
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
