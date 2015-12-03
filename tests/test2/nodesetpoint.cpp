/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Node that implements a simple setpoint.
 *
 * Requires YARP for communication with SMN, but will use MQTT for ports if it's available.
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
- OBNNODE_SMN_COMM_MQTT: if MQTT is supported for communication.
*/

#ifndef OBNNODE_COMM_YARP
#error This test requires YARP to run
#endif

using namespace OBNnode;

#ifdef OBNNODE_COMM_MQTT
#define MQTT_SERVER_ADDRESS "tcp://localhost:1883"
typedef MQTTInput<OBN_PB, double> INPUT_PORT_CLASS;
typedef MQTTOutput<OBN_PB, double> OUTPUT_PORT_CLASS;
MQTTClient mqtt_client;     // The MQTT client of this node (used for all communications)
#else
typedef YarpInput<OBN_PB, double> INPUT_PORT_CLASS;
typedef YarpOutput<OBN_PB, double> OUTPUT_PORT_CLASS;
#endif

#define MAIN_UPDATE 0

/* The controller node class */
class SetPoint: public YarpNode {
    /* Output: setpoint */
#ifdef OBNNODE_COMM_MQTT
    OUTPUT_PORT_CLASS setpoint{"sp"};
#else
    OUTPUT_PORT_CLASS setpoint{"sp");
#endif
    
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution;
    
public:
    SetPoint(const std::string& name, const std::string& ws = ""): YarpNode(name, ws),
    generator(std::chrono::system_clock::now().time_since_epoch().count()), distribution(-100, 100)
    { }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize() {
        bool success;
        
        // Add the ports to the node
        if (!(success = addOutput(&setpoint))) {
            std::cerr << "Error while adding setpoint output." << std::endl;
        }
        
#ifdef OBNNODE_COMM_MQTT
        setpoint.set_mqtt_client(&mqtt_client);
#endif
        
        // Add the update
        success = success && (addUpdate(MAIN_UPDATE, std::bind(&SetPoint::doMainUpdate, this)) >= 0);
        
        // Open the SMN port
        success = success && openSMNPort();
        
        return success;
    }
    
    /* This node should not receive UPDATE_X. */
    virtual void onUpdateX(updatemask_t m) override {
        std::cout << "At " << _current_sim_time << " UPDATE_X" << std::endl;
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() override {
        // Initial state and output
        setpoint = 0.0;
        std::cout << "At " << _current_sim_time << " INIT" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() override {
        std::cout << "At " << _current_sim_time << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
    
    // Callback for the main update
    void doMainUpdate() {
        setpoint = distribution(generator) / 10.0;
        std::cout << "At " << _current_sim_time << " UPDATE_Y" << std::endl;
    }
};

int main() {
    std::cout << "This is setpoint node." << std::endl;
    
    // Initialize the MQTT client
#ifdef OBNNODE_COMM_MQTT
    mqtt_client.setServerAddress(MQTT_SERVER_ADDRESS);
    mqtt_client.setClientID("test2_setpoint");
    if (!mqtt_client.initialize()) {
        std::cerr << "Error while initializing MQTT" << std::endl;
        return 10;
    }
    if (!mqtt_client.start()) {
        std::cerr << "Error while connecting to MQTT" << std::endl;
        return 10;
    }
#endif
    
    SetPoint node("sp", "test2");      // Node "sp" inside workspace "test2"
    if (!node.initialize()) {
        return 1;
    }
    
    // Here we will not connect the node to the GC, let the SMN do it
    
    std::cout << "Starting simulation..." << std::endl;
    
    node.run();
    
    std::cout << "Simulation finished. Goodbye!" << std::endl;
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    if (mqtt_client.isRunning()) {
        mqtt_client.stop();
    }
    
    google::protobuf::ShutdownProtobufLibrary();
    
    return node.hasError()?3:0;
}

