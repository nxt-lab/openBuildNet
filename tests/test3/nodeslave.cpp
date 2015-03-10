/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Node that implements an ADMM slave.
 *
 * This node simply waits for values sent from the master node. Once receiving a vector xbar, it updates its internal states as follows: y' = y + rho*(x - xbar) and x' = 1/(2+rho)*(rho*xbar + 2*ref - y'). Then it sends x back to the master.
 *
 * To make the problem more interesting, in each regular update, each slave perturbs its ref vector by random values.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <array>
#include <algorithm>
#include <chrono>
#include <random>

#include <obnnode_basic.h>

#ifndef OBNSIM_COMM_YARP
#error This test requires YARP to run
#endif

#include <obnnode_comm_yarp.h>
#include <obnsim_io.pb.h>

// Implement reporting functions for the SMN
void report_error(int code, std::string msg) {
    std::cerr << "ERROR (" << code << "): " << msg << std::endl;
}

void report_warning(int code, std::string msg) {
    std::cout << "WARNING (" << code << "): " << msg << std::endl;
}

void report_info(int code, std::string msg) {
    std::cout << "INFO (" << code << "): " << msg << std::endl;
}



/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNSIM_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_SMN_COMM_MQTT: if MQTT is supported for communication.
 */

void sendACK(int myid, OBNSimMsg::N2SMN::MSGTYPE type, OBNSimMsg::N2SMN& n2smn, OBNnode::YARP::smnPort &port) {
    OBNnode::YARP::smnMsg& msg = port.prepare();
    
    n2smn.set_msgtype(type);
    n2smn.set_id(myid);
    
    msg.setMessage(n2smn);
    port.writeStrict();
}

/* Read inputs as a vector of doubles */
template <class IT>
bool readInput(yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleVector> > &port, IT begin) {
    auto * input = port.read(false);
    if (input) {
        // There is a message, extract the value
        OBNSimIOMsg::DoubleVector msg;
        input->getMessage(msg);
        std::copy(msg.value().begin(), msg.value().end(), begin);
        return true;
    }
    return false;
}

/* Send output */
template <class ValIT>
void sendOutput(yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleVector> > &port, ValIT vbegin, ValIT vend, OBNnode::simtime_t t) {
    OBNSimIOMsg::DoubleVector msg;
    msg.set_time(t);
    
    for (int i=0; vbegin != vend; ++vbegin, ++i) {
        msg.add_value(*vbegin);
    }
    
    OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleVector> & output = port.prepare();
    output.setMessage(msg);
    port.writeStrict();
}

int main(int argc, char* argv[]) {
    std::cout << "This is the ADMM slave node." << std::endl;
    
    if (argc < 2) {
        std::cerr << "Not enough argument. Must specify the ID of the node in the command line." << std::endl;
        return 1;
    }
    
    int slaveID;
    
    try {
        slaveID = std::stoi(argv[1]);
    } catch (...) {
        std::cerr << "Error while parsing the slave node ID." << std::endl;
        return 2;
    }
    
    std::string myName = "x" + std::to_string(slaveID);
    
    yarp::os::Network yarp;
    
    OBNnode::YARP::smnPort smnPort;  // The SMN control port
    
    // Port to communicate with the master
    yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleVector> > admmPort;
    
    bool ok = smnPort.open("/test3/_" + myName + "_");
    ok = ok && admmPort.open("/test3/" + myName + "/x");
    
    if (!ok) {
        std::cout << "Failed to create ports." << std::endl;
        return 1;
    }
    
    // Make sure that no system messages from SMN are dropped
    smnPort.setStrict();
    
    std::cout << "SMN Port opened." << std::endl;
    
    enum NODESTATE {
        NODE_INIT,  // Node hasn't been initialized yet (or has been terminated)
        NODE_RUN    // Node is running
    } nodeState = NODE_INIT;
    
    OBNnode::YARP::smnMsg* msg;
    OBNSimMsg::SMN2N smn2n;
    OBNSimMsg::N2SMN n2smn;
    
    const int dim = 3;  // dimension of the x vector
    
    const double rho = 1.0;
    
    // The vectors
    std::array<double, dim> x, y, xbar, temp, ref;
    
    for (int i = 0; i < ref.size(); ++i) {
        ref[i] = i*0.5 + slaveID;
    }
    
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    
    // Generate random perturbations
    std::uniform_real_distribution<> distribution(-1.0, 1.0);
    
    bool simRunning = true;
    OBNnode::simtime_t curTime;
    int myID;   // This is the ID of the node in the simulation network, not to be confused with the slave ID in the ADMM network.
    
    //std::ofstream dump("motor.txt");
    //const char tab = '\t';
    
    std::cout << "Simulation started." << std::endl;
    
    while (simRunning) {
        // Check if there is next message from GC
        msg = smnPort.read(false);
        
        if (msg) {
            msg->getMessage(smn2n);
            
            curTime = smn2n.time();
            
            if (smn2n.has_id()) {
                myID = smn2n.id();
            }
            
            if (smn2n.msgtype() == OBNSimMsg::SMN2N_MSGTYPE_SIM_TERM) {
                // Special case, always terminate
                report_info(0, "At " + std::to_string(curTime) + " TERMINATE");
                nodeState = NODE_INIT;
                simRunning = false;
                break;
            }
            
            switch (nodeState) {
                case NODE_RUN:
                    switch (smn2n.msgtype()) {
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_Y:
                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK, n2smn, smnPort);
                            break;
                            
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_X:
                            // Perturb the ref vector
                            for (auto it = ref.begin(); it != ref.end(); ++it) {
                                *it += distribution(generator);
                            }
                            
                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_X_ACK, n2smn, smnPort);
                            report_info(0, "At " + std::to_string(curTime) + " UPDATE_X");
                            break;
                            
                        default:
                            report_warning(0, "Unexpected message type " + std::to_string(smn2n.msgtype()) + " during RUNNING.");
                            break;
                    }
                    break;
                    
                case NODE_INIT:
                    switch (smn2n.msgtype()) {
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_INIT:
                            // Initialization
                            x.fill(0.0);
                            y.fill(0.0);
                            
                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_INIT_ACK, n2smn, smnPort);
                            report_info(0, "At " + std::to_string(curTime) + " INIT");
                            nodeState = NODE_RUN;
                            break;
                            
                        default:
                            report_warning(0, "Unknown message type " + std::to_string(smn2n.msgtype()) + " during INIT.");
                            break;
                    }
                    break;
            }
        }
        
        if (!simRunning) {
            break;
        }
        
        // Check the ADMM port
        if (readInput(admmPort, xbar.begin())) {
            // Update state variables
            std::transform(x.begin(), x.end(), xbar.begin(), temp.begin(),
                           [rho](double x, double y) {return rho*(x-y);});  // temp = rho*(x-xbar)
            std::transform(y.begin(), y.end(), temp.begin(), y.begin(), std::plus<double>());  // y' = y + rho*(x - xbar)
            
            std::transform(xbar.begin(), xbar.end(), ref.begin(), temp.begin(),
                           [rho](double x, double r) {return rho*x + 2*r;});    // temp = rho*xbar + 2*ref
            std::transform(temp.begin(), temp.end(), y.begin(), x.begin(),
                           [rho](double x, double y) {return (x - y)/(2+rho);});    // x' = 1/(2+rho)*(rho*xbar + 2*ref - y').
            
            // Send back to master
            sendOutput(admmPort, x.begin(), x.end(), curTime);
        }
    }
    
    std::cout << "Simulation finished." << std::endl;
    
    
    std::cout << "Goodbye!" << std::endl;
    
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
