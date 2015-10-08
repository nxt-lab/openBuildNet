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

/* This file can be used as a template for creating openBuildNet nodes with the C++ framework.
 * For typed input and output ports (see YarpInput and YarpOutput classes), vectors and matrices are created and stored using the Eigen template library (http://eigen.tuxfamily.org).
 * A very important note to keep in mind when using Eigen is the problem with the alignment of fixed-size vectorizable types; see http://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html for details.
 * A fixed-size vectorizable Eigen type (http://eigen.tuxfamily.org/dox/group__TopicFixedSizeVectorizable.html) has its size fixed at compile-time (vs. run-time) and its byte size is a multiple of 16.
 * To avoid these issues, do one of the following:
 *  - Disable vectorization completely (see http://eigen.tuxfamily.org/index.php?title=FAQ#I_disabled_vectorization.2C_but_I.27m_still_getting_annoyed_about_alignment_issues.21)
 *  - Take care of the alignment of these objects by: always pass Eigen objects to functions by references not by values, take special consideration when using STL containers and Eigen objects (http://eigen.tuxfamily.org/dox/group__TopicStlContainers.html), and modify code of any structure/class that contains Eigen objects (http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html).
 */


/** The following macros are defined by CMake to indicate which libraries this build supports:
 - OBNNODE_COMM_YARP: if YARP is supported for communication.
 - OBNNODE_COMM_MQTT: if MQTT is supported for communication.
 */

#include <obnnode.h>

using namespace OBNnode;


/* Define the node class here, derived from YarpNode.
 The class should contain all the ports, node's data, callbacks, and any methods useful for running this node.
 */
class MyNodeClass: public YarpNode {
    /* All ports should be defined as members of the node class.
     They can be defined as member objects which must be initialized in the node's constructor, or pointers to objects which can be dynamically initialized later. */
    YarpInput<OBN_PB, double, false> input1;
    YarpInput<OBN_PB, obn_vector<double>, false> *input2 = nullptr;  // input2 is dynamically allocated
    YarpOutput<OBN_PB, obn_vector_fixed<double, 3> > output1;
    
#ifdef OBNNODE_COMM_MQTT
#define MQTT_SERVER_ADDRESS "tcp://localhost:1883"
    
    MQTTClient m_mqtt_client;   ///< The MQTT Client object for communication.
    
    MQTTInput<OBN_PB, double, false> input3;
    MQTTInput<OBN_PB, obn_vector<double>, false> input4;
#endif

public:
    /* ws is the workspace name, useful if you have multiple simulation networks running on the same Yarp network. */
    MyNodeClass(const std::string& name, const std::string& ws = ""): YarpNode(name, ws),
    input1("u1"), output1("y"), input3("u3", &m_mqtt_client), input4("u4", &m_mqtt_client)
    { }
    
    virtual ~MyNodeClass() {
        if (input2) {
            delete input2;
        }
        if (m_mqtt_client.isRunning()) {
            m_mqtt_client.stop();
        }
    }
    
    /* This method initializes the node process (ONCE when the node is first created, not every simulation run).
     Nodes can be added / registered with the node, hardware components may be started, etc.
     */
    bool initialize() {
        // Dynamically create input2
        input2 = new YarpInput<OBN_PB, obn_vector<double>, false>("u2");
        
        bool success;
        
#ifdef OBNNODE_COMM_MQTT
        /** Start the MQTT Client. */
        m_mqtt_client.setClientID(full_name());
        m_mqtt_client.setServerAddress(MQTT_SERVER_ADDRESS);
        if (!(success = m_mqtt_client.start())) {
            std::cerr << "Error while connecting to MQTT" << std::endl;
        }
#endif
        
        // Add the ports to the node
        if (success && !(success = addInput(&input1))) {
            std::cerr << "Error while adding input " << input1.getPortName() << std::endl;
        }
        
        if (success && !(success = addInput(input2))) {
            std::cerr << "Error while adding input " << input2->getPortName() << std::endl;
        }
    
        if (success && !(success = addOutput(&output1))) {
            std::cerr << "Error while adding output " << output1.getPortName() << std::endl;
        }
        
#ifdef OBNNODE_COMM_MQTT
        if (success && !(success = addInput(&input3))) {
            std::cerr << "Error while adding input " << input3.getPortName() << std::endl;
        }
        
        if (success && !(success = addInput(&input4))) {
            std::cerr << "Error while adding input " << input4.getPortName() << std::endl;
        }
#endif
        
        // Add the first update
        if (success) {
            // These details of the update are optional, but they are useful later on
            
            // List of inputs to this update, the first is the port name, the second specifies whether this input has direct feedthrough to this update.
            UpdateType::INPUT_LIST inputs{ {"u1", true}, {"u2", true}};
            
#ifdef OBNNODE_COMM_MQTT
            inputs.emplace_back("u3", true);
            inputs.emplace_back("u4", true);
#endif
            
            // List of outputs of this update
            UpdateType::OUTPUT_LIST outputs{"y"};
            
            int update_idx = addUpdate(std::bind(&MyNodeClass::onFirstUpdateY, this), std::bind(&MyNodeClass::onFirstUpdateX, this), 2*YarpNode::SECOND, inputs, outputs, "First update");
            success = update_idx >= 0;
            if (!success) {
                std::cerr << "Error while adding the first update with code: " << update_idx << std::endl;
            }
        }

        // Add the second update
        if (success) {
            // Here we don't specify the optional details, but it's recommended to do so
            // And this update doesn't have state update (UPDATE_X)
            int update_idx = addUpdate(std::bind(&MyNodeClass::onSecondUpdateY, this));
            success = update_idx >= 0;
            if (!success) {
                std::cerr << "Error while adding the second update with code: " << update_idx << std::endl;
            }
        }
        
        // Open the SMN port
        success = success && openSMNPort();
        
        return success;
    }
    
    /* UPDATE_Y of first update */
    void onFirstUpdateY() {
        std::cout << "UPDATE_Y(1)" << std::endl;
        output1 = (*input2)() * input1();
#ifdef OBNNODE_COMM_MQTT
        output1 = output1() + input4() * input3();
#endif
    }

    /* UPDATE_X of first update */
    void onFirstUpdateX() {
        std::cout << "UPDATE_X(1)" << std::endl;
    }
    
    /* UPDATE_Y of second update */
    void onSecondUpdateY() {
        std::cout << "UPDATE_Y(2)" << std::endl;
        *output1 << 1.0, 2.0, 3.0;
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() override {
        std::cout << "INITIALIZATION" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() override {
        std::cout << "TERMINATED" << std::endl;
    }
};


int main() {
    // Set verbosity level
    yarp::os::Network::setVerbosity(-1);
    
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
    
    // It's a good practice to delay the termination of the node by a small amount of time
    // to avoid overloading the nameserver
    mynode.delayBeforeShutdown();
    
    // If the node has error (and terminated due to that) we can detect it here
    return mynode.hasError()?3:0;
}
