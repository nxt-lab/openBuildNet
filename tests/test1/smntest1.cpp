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

#if !defined(OBNSIM_COMM_YARP) && !defined(OBNSIM_COMM_MQTT)
#error This test requires YARP or MQTT to run
#endif


#ifdef OBNSIM_COMM_MQTT
#include <obnsmn_comm_mqtt.h>
#else
#include <obnsmn_comm_yarp.h>
#endif

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


int main() {
#ifndef OBNSIM_COMM_MQTT
    yarp::os::Network yarp;
    OBNsmn::YARP::YARPPort gcPort;
    bool ok = gcPort.open("/test1/_smn_/node1");
    if (!ok) {
        std::cerr << "Failed to create ports." << std::endl;
        return 1;
    }
#endif

    // The Global clock thread
    OBNsmn::GCThread gc;
    
#ifdef OBNSIM_COMM_MQTT
    OBNsmn::MQTT::MQTTClient mqttClient(&gc);
    mqttClient.setClientID("test1");
    mqttClient.setPortName("test1/_smn_/_gc_");
    mqttClient.setServerAddress("tcp://localhost:1883");
    if (!mqttClient.start()) {
        std::cerr << "ERROR: could not start MQTT communication thread." << std::endl;
        return 1;
    }
#endif

    // ======== Creating node =========
    
    // In this test, we create one node
#ifdef OBNSIM_COMM_MQTT
    OBNsmn::MQTT::OBNNodeMQTT* pnode = new OBNsmn::MQTT::OBNNodeMQTT("node1", 2, "test1/node1/_gc_", &mqttClient);
#else
    OBNsmn::YARP::OBNNodeYARP* pnode = new OBNsmn::YARP::OBNNodeYARP("node1", 2, &gcPort);
#endif
    
    pnode->setUpdateType(0, 10);  // bit mask 0
    pnode->setUpdateType(1, 0);  // bit mask 1 -> irregular update

    gc.insertNode(pnode);
    
    OBNsmn::NodeDepGraph* nodeGraph = new OBNsmn::NodeDepGraph_BGL(1);
    gc.setDependencyGraph(nodeGraph);
    
    // ========== End creating nodes ===========
    
    // Configure the GC
    gc.ack_timeout = 0;
    gc.setFinalSimulationTime(100);

#ifndef OBNSIM_COMM_MQTT
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
#endif

    
    // Start running the GC thread
    if (!gc.startThread()) {
        std::cout << "Error: cannot start GC thread." << std::endl;
        return 2;
    }
    
    
    // The main thread will only wait until the GC thread stops
    std::cout << "In this test, we only wait until the simulation stops..." << std::endl;
    
    //Join the threads with the main thread
    gc.joinThread();
    
#ifndef OBNSIM_COMM_MQTT
    yarpThread.joinThread();
#endif
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
