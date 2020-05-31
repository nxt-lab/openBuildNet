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

/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNNODE_COMM_YARP: if YARP is supported for communication.
 - OBNNODE_COMM_MQTT: if MQTT is supported for communication.
 */

using namespace OBNnode;
using std::cout;
using std::endl;

#define MAIN_UPDATE 0
#define IRREG_UPDATE 1

#ifdef OBNNODE_COMM_MQTT
#define NODECLASS MQTTNode
#else
#ifdef OBNNODE_COMM_YARP
#define NODECLASS YarpNode
#else
#error This program requires either Yarp or MQTT to run
#endif
#endif

/* The main node class */
class MyNode: public NODECLASS {
    int counter;
    WaitForCondition* pWaitFor;
public:
    MyNode(const std::string& name, const std::string& ws = ""): NODECLASS(name, ws), counter(0)
    { }
    
    virtual ~MyNode() {
        cout << "Node program exits." << endl;
    }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize();
    
    /* Implement this callback to process UPDATE_X. */
    virtual void onUpdateX(updatemask_t m) {
        auto tsim = currentSimulationTime();
        std::cout << "At " << tsim << " UPDATE_X (" << m << ")" << std::endl;
        if (pWaitFor) {
            // Check the result of future update request
            auto r = resultFutureUpdate(pWaitFor);
            if (r != 0) {
                cout << "At " << tsim << " Event request was rejected." << endl;
            } else {
                cout << "At " << tsim << " Event request was accepted." << endl;
            }
            pWaitFor = nullptr;
        }
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual int64_t onInitialization() {
        // Initial state and output
        std::cout << "At " << currentSimulationTime() << " INIT" << std::endl;
        return 0;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "At " << currentSimulationTime() << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
    
    /* Here we define the callback for updates using template function. */
    template <const int idx> void doUpdateY();
};

/* Implement the callback for each update by specializing the template function. */
template<>
void MyNode::doUpdateY<MAIN_UPDATE>() {
    auto tsim = currentSimulationTime();
    if (counter++ % 3 == 2) {
        // Request for future update (event)
        pWaitFor = requestFutureUpdate(tsim + 13, 1 << IRREG_UPDATE, false);
        if (!pWaitFor) {
            cout << "At " << tsim << " Problem: could not request event." << endl;
        }
    } else {
        pWaitFor = nullptr;
    }
    std::cout << "At " << tsim << " Main UPDATE_Y" << std::endl;
}

template<>
void MyNode::doUpdateY<IRREG_UPDATE>() {
    std::cout << "At " << currentSimulationTime() << " Irregular UPDATE_Y" << std::endl;
}

bool MyNode::initialize() {
    // Open the SMN port
    bool success = openSMNPort();
    if (!success) {
        std::cerr << "Error while opening the GC/SMN port.\n";
        return false;
    }
    
    // Add the updates
    // Here the updates are to be defined in the SMN, so we don't care about the details.
    success = success && (addUpdate(MAIN_UPDATE, std::bind(&MyNode::doUpdateY<MAIN_UPDATE>, this)) >= 0);
    success = success && (addUpdate(IRREG_UPDATE, std::bind(&MyNode::doUpdateY<IRREG_UPDATE>, this)) >= 0);
    
    return success;
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
