/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A source node.
 *
 * Requires MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <chrono>
#include <random>
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

class SourceNode: public MQTTNode {
    MQTTOutput< OBN_PB, double > m_y1;
    MQTTOutput< OBN_PB, obn_vector<double> > m_y2;
    
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution;
    
public:
    SourceNode(const std::string& name, const std::string& ws = ""): MQTTNode(name, ws), m_y1("y1", &mqtt_client), m_y2("y2", &mqtt_client),
    generator(std::chrono::system_clock::now().time_since_epoch().count()), distribution(-100, 0)
    { }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize() {
        if (!this->openSMNPort()) {
            std::cerr << "Error while opening the GC port; check the network or server.\n";
            return false;
        }
        
        bool success;
        
        // Add the ports to the node
        if (!(success = addOutput(&m_y1))) {
            std::cerr << "Error while adding output:" << m_y1.getPortName() << std::endl;
        }
        
        if (success && !(success = addOutput(&m_y2))) {
            std::cerr << "Error while adding output:" << m_y2.getPortName() << std::endl;
        }
        
        // Add the updates
        success = success && (this->addUpdate(MAIN_UPDATE, std::bind(&SourceNode::doMainUpdate, this)) >= 0);
        
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
    
    void doMainUpdate() {
        m_y1 = distribution(generator) * 1.0;
        
        auto& y2 = (*m_y2);
        y2.resize(2);
        y2 << distribution(generator) * 1.0, distribution(generator) * 1.0;
    }
    
    /* There are other callbacks for reporting errors, etc. */
};

int main() {
    std::cout << "This is source node 2." << std::endl;
    
    SourceNode node("source2", "test4");
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

