/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A receiver node.
 *
 * Requires MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnnode.h>

/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNNODE_COMM_YARP: if YARP is supported for communication.
 - OBNNODE_COMM_MQTT: if MQTT is supported for communication.
 */

#ifndef OBNNODE_COMM_MQTT
#error This test requires MQTT to run
#endif

using namespace OBNnode;

#define MAIN_UPDATE 0

class ReceiverNode: public MQTTNode {
    MQTTInput<OBN_PB, double, true> m_u1;
    MQTTInput<OBN_PB, obn_vector<double>, true> m_u2;

public:
    ReceiverNode(const std::string& name, const std::string& ws = ""): MQTTNode(name, ws), m_u1("u1", &mqtt_client), m_u2("u2", &mqtt_client)
    { }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize() {
        if (!this->openSMNPort()) {
            std::cerr << "Error while opening the GC port; check the network or server.\n";
            return false;
        }
        
        bool success;
        
        // Add the ports to the node
        if (!(success = addInput(&m_u1))) {
            std::cerr << "Error while adding input:" << m_u1.getPortName() << std::endl;
        }
        
        if (success && !(success = addInput(&m_u2))) {
            std::cerr << "Error while adding input:" << m_u2.getPortName() << std::endl;
        }
        
        // Add the updates
        success = success && (this->addUpdate(MAIN_UPDATE, NULL_UPDATE_CALLBACK, std::bind(&ReceiverNode::printInputs, this)) >= 0);
        
        return success;
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() {
        // Initial state and output
        std::cout << "At " << this->currentSimulationTime() << " INIT" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "At " << this->currentSimulationTime() << " TERMINATED" << std::endl;
    }
    
    void printInputs() {
        std::cout << "At " << this->currentSimulationTime() << std::endl;
        auto n = m_u1.size();
        
        if (n > 0) {
            std::cout << "u1: ";
            while (m_u1.isValuePending()) {
                std::cout << m_u1.pop() << " ";
            }
            std::cout << std::endl;
        }
        
        n = m_u2.size();
        
        if (n > 0) {
            std::cout << "u2:\n";
            while (m_u2.isValuePending()) {
                auto u2 = m_u2.pop();
                std::cout << "    " << u2->transpose() << std::endl;
            }
            std::cout << std::endl;
        }
    }
    
    /* There are other callbacks for reporting errors, etc. */
};

int main() {
    std::cout << "This is receiver node." << std::endl;
    
    ReceiverNode node("receiver", "test4");
    if (!node.initialize()) {
        std::cout << "There was/were error(s) while initializing the node.\n";
        return 1;
    }
    
    // Here we will not connect the node to the GC, let the SMN do it
    
    std::cout << "Starting simulation..." << std::endl;
    
    node.run();
    
    std::cout << "Simulation finished. Goodbye!" << std::endl;
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    node.delayBeforeShutdown();
    
    return node.hasError()?3:0;
}

