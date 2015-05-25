/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A test node, which doesn't do anything except testing the messages with SMN.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>
#include <obnnode.h>

#ifndef OBNSIM_COMM_YARP
#error This test requires YARP to run
#endif


/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNSIM_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_SMN_COMM_MQTT: if MQTT is supported for communication.
 */

using namespace OBNnode;
using std::cout;
using std::endl;

#define MAIN_UPDATE 0
#define IRREG_UPDATE 1

/* The main node class */
class MyNode: public YarpNode {
    int counter;
    WaitForCondition* pWaitFor;
public:
    MyNode(const std::string& name, const std::string& ws = ""): YarpNode(name, ws), counter(0)
    { }
    
    virtual ~MyNode() {
        cout << "Node program exits." << endl;
    }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize() {
        bool success = true;
        
        // Open the SMN port
        success = success && openSMNPort();
        
        return success;
    }
    
    /* Implement this callback to process UPDATE_X. */
    virtual void onUpdateX() {
        std::cout << "At " << _current_sim_time << " UPDATE_X" << std::endl;
        if (pWaitFor) {
            // Check the result of future update request
            auto r = resultFutureUpdate(pWaitFor);
            if (r != 0) {
                cout << "At " << _current_sim_time << " Event request was rejected." << endl;
            } else {
                cout << "At " << _current_sim_time << " Event request was accepted." << endl;
            }
            pWaitFor = nullptr;
        }
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() {
        // Initial state and output
        std::cout << "At " << _current_sim_time << " INIT" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "At " << _current_sim_time << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
    
    /* Declare the update types of the node by listing their index constants in the macro OBN_DECLARE_UPDATES(...)
     Their listing order determines the order in which the corresponding update callbacks are called. */
    OBN_DECLARE_UPDATES(MAIN_UPDATE, IRREG_UPDATE)
};

/* For each update type, define the update callback function OUTSIDE the class declaration.
 Each callback is defined by OBN_DEFINE_UPDATE(<Your node class name>, <Index of the update type>) { code here; } */

OBN_DEFINE_UPDATE(MyNode, MAIN_UPDATE) {
    if (counter++ % 3 == 2) {
        // Request for future update (event)
        pWaitFor = requestFutureUpdate(_current_sim_time + 13, 1 << IRREG_UPDATE, false);
        if (!pWaitFor) {
            cout << "At " << _current_sim_time << " Problem: could not request event." << endl;
        }
    } else {
        pWaitFor = nullptr;
    }
    std::cout << "At " << _current_sim_time << " Main UPDATE_Y" << std::endl;
}

OBN_DEFINE_UPDATE(MyNode, IRREG_UPDATE) {
    std::cout << "At " << _current_sim_time << " Irregular UPDATE_Y" << std::endl;
}

int main() {
    std::cout << "This is nodetest1, a very simple node." << std::endl;
    
    MyNode mynode("node1", "test1");
    
    if (!mynode.initialize()) {
        return 1;
    }
    
    // Here we will not connect the node to the GC, let the SMN do it
    
    std::cout << "Starting simulation..." << std::endl;
    
    mynode.run();
    
    std::cout << "Simulation finished. Goodbye!" << std::endl;
    
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return mynode.hasError()?3:0;
}
