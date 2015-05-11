/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief The main function of the node.
 *
 * This file contains code for the actual node (callbacks for events, configure the ports, etc.)
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnnode.h>

using namespace OBNnode;

/* Define the constants of the indices of the update types.
 Each update type must have a unique index between 0 and MAX_UPDATE_INDEX (defined in obnnode_basic.h)
 The constants can be defined by macros, or constant integers, or enumerate values, etc. as long as they are constants at compile time.
 */
#define FIRST_UPDATE 0
#define SECOND_UPDATE 1

/* Define the node class here, derived from YarpNode.
 The class should contain all the ports, node's data, callbacks, and any methods useful for running this node.
 */
class MyNodeClass: public YarpNode {
    /* All ports should be defined as members of the node class.
     They can be defined as member objects which must be initialized in the node's constructor, or pointers to objects which can be dynamically initialized later. */
    YarpInput<OBN_PB, double, false> input1;
    YarpInput<OBN_PB, obn_vector_fixed<double, 3>, false> *input2 = nullptr;  // input2 is dynamically allocated
    
    YarpOutput<OBN_PB, obn_vector_fixed<double, 3> > output1;

public:
    /* ws is the workspace name, useful if you have multiple simulation networks running on the same Yarp network. */
    MyNodeClass(const std::string& name, const std::string& ws = ""): YarpNode(name, ws),
    input1("u1"), output1("y")
    { }
    
    virtual ~MyNodeClass() {
        if (input2) {
            delete input2;
        }
    }
    
    /* This method initializes the node process (ONCE when the node is first created, not every simulation run).
     Nodes can be added / registered with the node, hardware components may be started, etc.
     */
    bool initialize() {
        // Dynamically create input2
        input2 = new YarpInput<OBN_PB, obn_vector_fixed<double, 3>, false>("u2");
        
        bool success;
        
        // Add the ports to the node
        if (!(success = addInput(&input1))) {
            std::cerr << "Error while adding input " << input1.getPortName() << std::endl;
        }
        
        if (success && !(success = addInput(input2))) {
            std::cerr << "Error while adding input " << input2->getPortName() << std::endl;
        }
    
        if (success && !(success = addOutput(&output1))) {
            std::cerr << "Error while adding output " << output1.getPortName() << std::endl;
        }
        
        // Open the SMN port
        success = success && openSMNPort();
        
        return success;
    }
    
    /* Implement this callback to process UPDATE_X. */
    virtual void onUpdateX() {
        std::cout << "UPDATE_X" << std::endl;
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() {
        std::cout << "INITIALIZATION" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
    
    /* Declare the update types of the node by listing their index constants in the macro OBN_DECLARE_UPDATES(...)
     Their listing order determines the order in which the corresponding update callbacks are called. */
    OBN_DECLARE_UPDATES(FIRST_UPDATE, SECOND_UPDATE)
};

/* For each update type, define the update callback function OUTSIDE the class declaration.
 Each callback is defined by OBN_DEFINE_UPDATE(<Your node class name>, <Index of the update type>) { code here; } */

OBN_DEFINE_UPDATE(MyNodeClass,FIRST_UPDATE) {
    std::cout << "Update 0" << std::endl;
    // Output can be assigned new value
    output1 = (*input2)() * input1();
}

OBN_DEFINE_UPDATE(MyNodeClass,SECOND_UPDATE) {
    std::cout << "Update 1" << std::endl;
    // Output's variable can also be accessed directly using * operator
    *output1 << 1.0, 2.0, 3.0;
}


int main() {
    MyNodeClass mynode("mynode", "myexperiment");   // Node named "mynode" in the workspace "myexperiment"
    
    if (!mynode.initialize()) {
        return 1;
    }
    
    // Typically the SMN will connect all the ports in the network, including between the nodes and the SMN
    // However, we can also connect the node and the SMN in the node if desired
    if (!mynode.connectWithSMN()) {
        return 2;
    }

    // Run the node
    mynode.run();

    // It's a good practice to clean up the ProtoBuf library, though not required
    google::protobuf::ShutdownProtobufLibrary();
    
    // If the node has error (and terminated due to that) we can detect it here
    return mynode.hasError()?3:0;
}
