/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Node that implements a simple motor model.
 *
 * Requires YARP for communication with SMN, but will use MQTT for ports if it's available.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnnode.h>

/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNNODE_COMM_YARP: if YARP is supported for communication.
 - OBNNODE_SMN_COMM_MQTT: if MQTT is supported for communication.
 */

#if !defined(OBNNODE_COMM_YARP) && !defined(OBNNODE_COMM_MQTT)
#error Either Yarp or MQTT is required to run
#endif

using namespace OBNnode;

#ifdef OBNNODE_COMM_MQTT
#define MQTT_SERVER_ADDRESS "tcp://localhost:1883"
typedef MQTTInput<OBN_PB, double> INPUT_PORT_CLASS;
typedef MQTTOutput<OBN_PB, double> OUTPUT_PORT_CLASS;
#define BASE_CLASS MQTTNode
#else
typedef YarpInput<OBN_PB, double> INPUT_PORT_CLASS;
typedef YarpOutput<OBN_PB, double> OUTPUT_PORT_CLASS;
#define BASE_CLASS YarpNode
#endif

#define MAIN_UPDATE 0

/* The controller node class */
class Motor: public BASE_CLASS {
    /* One input: voltage */
    /* Output: velocity */
    
    INPUT_PORT_CLASS voltage{"vol"};
    OUTPUT_PORT_CLASS velocity{"v"};

    /* The state variable */
    Eigen::Vector2d x;
    
    /* The state matrix */
    Eigen::Matrix2d A;
    
    /* The output matrix */
    Eigen::RowVector2d C;
    
public:
    Motor(const std::string& name, const std::string& ws = ""): BASE_CLASS(name, ws)
    { }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize() {
        bool success;
        
        // Add the ports to the node
        if (!(success = addInput(&voltage))) {
            std::cerr << "Error while adding voltage input." << std::endl;
        }
        
        if (success && !(success = addOutput(&velocity))) {
            std::cerr << "Error while adding velocity output." << std::endl;
        }
        
#ifdef OBNNODE_COMM_MQTT
        voltage.set_mqtt_client(&mqtt_client);
        velocity.set_mqtt_client(&mqtt_client);
#endif
        
        // Add the update
        success = success && (addUpdate(MAIN_UPDATE, std::bind(&Motor::doMainUpdate, this), std::bind(&Motor::doStateUpdate, this)) >= 0);
        
        // Open the SMN port
        success = success && openSMNPort();
        
        // Initialize the state matrix
        if (success) {
            A <<
            1.511, -0.5488,
            1.0 , 0.0;
            
            C << 0.03294, 0.02697;
        }
        
        return success;
    }
    
    /* Implement this callback for state update. */
    void doStateUpdate() {
        // Update the state
        x = A * x;
        x(0) += 0.0625 * voltage();
        
        // At this point, the inputs are all up-to-date, regardless of the order, so we can dump log data
        std::cout << "At " << _current_sim_time << " UPDATE_X" << std::endl;
    }
    
    void doMainUpdate() {
        velocity = C * x;
        std::cout << "At " << _current_sim_time << " UPDATE_Y" << std::endl;
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual int64_t onInitialization() override {
        // Initial state and output
        x.setZero();
        velocity = 0.0;
        std::cout << "At " << currentSimulationTime() << " INIT" << std::endl;
        return 0;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() override {
        std::cout << "At " << currentSimulationTime() << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
};


int main() {
    std::cout << "This is motor node." << std::endl;
    
    Motor motor("motor", "test2");      // Node "motor" inside workspace "test2"
    
    // Initialize the MQTT client
#ifdef OBNNODE_COMM_MQTT
    motor.setServerAddress(MQTT_SERVER_ADDRESS);
#endif
    
    if (!motor.initialize()) {
        return 1;
    }
    
    // Here we will not connect the node to the GC, let the SMN do it
    
    std::cout << "Starting simulation..." << std::endl;
    
    motor.run();
    
    std::cout << "Simulation finished. Goodbye!" << std::endl;
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    
    google::protobuf::ShutdownProtobufLibrary();
    
    return motor.hasError()?3:0;
}
