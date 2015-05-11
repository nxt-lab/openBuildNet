/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Node that implements a simple controller.
 *
 * Uses the node.cpp framework.
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <fstream>
#include <obnnode.h>

#ifndef OBNSIM_COMM_YARP
#error This test requires YARP to run
#endif

/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNSIM_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_SMN_COMM_MQTT: if MQTT is supported for communication.
 */

using namespace OBNnode;

#define MAIN_UPDATE 0

/* The controller node class */
class Controller: public YarpNode {
    /* Two inputs: velocity and setpoint */
    YarpInput<OBN_PB, double> velocity, setpoint;
    
    /* Output: the control value */
    YarpOutput<OBN_PB, double> command;
    
    /* The state variable */
    Eigen::Vector3d x;
    
    /* The state matrix */
    Eigen::Matrix3d A;
    
    /* The output matrix */
    Eigen::RowVector3d C;
    
    std::ofstream dump;
    const char tab = '\t';
    
public:
    Controller(const std::string& name, const std::string& ws = ""): YarpNode(name, ws),
    velocity("v"), setpoint("sp"), command("u"), dump("controller.txt")
    { }
    
    virtual ~Controller() {
        dump.close();
    }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize() {
        bool success;
        
        // Add the ports to the node
        if (!(success = addInput(&velocity))) {
            std::cerr << "Error while adding velocity input." << std::endl;
        }
        
        if (success && !(success = addInput(&setpoint))) {
            std::cerr << "Error while adding setpoint input." << std::endl;
        }
        
        if (success && !(success = addOutput(&command))) {
            std::cerr << "Error while adding control output." << std::endl;
        }
        
        // Open the SMN port
        success = success && openSMNPort();
        
        // Initialize the state matrix
        if (success) {
            A <<
            -0.82, 1.0, 0.82,
            1.0 , 0.0, 0.0,
            0.0 , 1.0, 0.0;
            
            C << 12.62, -19.75, 7.625;
        }
        
        return success;
    }
    
    /* Implement this callback to process UPDATE_X. */
    virtual void onUpdateX() {
        // Update the state
        x = A * x;
        x(0) += 32.0 * (setpoint() - velocity());
        
        // At this point, the inputs are all up-to-date, regardless of the order, so we can dump log data
        dump << _current_sim_time << tab << setpoint() << tab << velocity() << tab << command() << tab << x.transpose() << std::endl;
        std::cout << "At " << _current_sim_time << " UPDATE_X" << std::endl;
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() {
        // Initial state and output
        x.setZero();
        command = 0.0;
        std::cout << "At " << _current_sim_time << " INIT" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "At " << _current_sim_time << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
    
    /* Declare the update types of the node by listing their index constants in the macro OBN_DECLARE_UPDATES(...)
     Their listing order determines the order in which the corresponding update callbacks are called. */
    OBN_DECLARE_UPDATES(MAIN_UPDATE)
};

/* For each update type, define the update callback function OUTSIDE the class declaration.
 Each callback is defined by OBN_DEFINE_UPDATE(<Your node class name>, <Index of the update type>) { code here; } */

OBN_DEFINE_UPDATE(Controller, MAIN_UPDATE) {
    command = C * x;
    std::cout << "At " << _current_sim_time << " UPDATE_Y" << std::endl;
}

int main() {
    std::cout << "This is controller node." << std::endl;
    
    Controller ctrl("ctrl", "test2");       // Node "ctrl" inside workspace "test2"

    if (!ctrl.initialize()) {
        return 1;
    }
    
    // Here we will not connect the node to the GC, let the SMN do it
    
    std::cout << "Starting simulation..." << std::endl;

    ctrl.run();
    
    std::cout << "Simulation finished. Goodbye!" << std::endl;
    
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return ctrl.hasError()?3:0;
}
