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

#include <chrono>
#include <thread>

#include <obnnode.h>

using namespace OBNnode;

int main() {
    YarpNode mynode("mynode");
    
    // Create ports to test
    YarpInput<OBN_PB, double, false> myport1("input1");
    YarpInput<OBN_PB, obn_vector_fixed<double, 3>, false> myport2("input2");
    
    YarpOutput<OBN_PB, obn_vector_fixed<double, 3>> myport3("output1");
    
    std::cout << "Port 1:" << std::endl << myport1() << std::endl;
    std::cout << "Port 2:" << std::endl << myport2() << std::endl;
    std::cout << "Port 3:" << std::endl << myport3() << std::endl;
    
    if (!mynode.addInput(&myport1)) {
        std::cerr << "Error while adding input 1" << std::endl;
    }
    
    if (!mynode.addInput(&myport2)) {
        std::cerr << "Error while adding input 2" << std::endl;
    }
    
    if (!mynode.addOutput(&myport3)) {
        std::cerr << "Error while adding input 3" << std::endl;
    }
    
    // Connect output to input
    if (!yarp::os::Network::connect(myport3.fullPortName(), myport2.fullPortName())) {
        std::cerr << "Error while connecting ports." << std::endl;
        return 1;
    }
    
    // Send out value
    *myport3 << 1.7, 2.7, 3.7;  // Directly access
    myport3 = myport3() * 2.0;  // Assignment
    std::cout << "Output value is " << std::endl << myport3() << std::endl;
    
    myport3.sendSync();
    
    std::cout << "Done sending." << std::endl;
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Read the value
    std::cout << "Input:" << std::endl << myport2() << std::endl;
    
    return 0;
}