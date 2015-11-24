/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Node that implements an ADMM master node.
 *
 * Algorithm: in each iteration
 * - Master sends xbar to all slaves.
 * - Each slave updates: y' = y + rho*(x - xbar) and x' = 1/(2+rho)*(rho*xbar + 2*ref - y')
 * - Each slave sends its x' back to master.
 * - Master updates xbar.
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
#include <chrono>
#include <random>
#include <array>
#include <bitset>
#include <algorithm>

#include <obnnode_basic.h>

#ifndef OBNSIM_COMM_YARP
#error This test requires YARP to run
#endif

#include <obnnode_yarpnode.h>
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

void sendEventRequest(int myid, OBNnode::simtime_t t, int mask, OBNSimMsg::N2SMN& n2smn, OBNnode::YARP::smnPort &port) {
    OBNnode::YARP::smnMsg& msg = port.prepare();
    
    n2smn.set_msgtype(OBNSimMsg::N2SMN_MSGTYPE_SIM_EVENT);
    n2smn.set_id(myid);
    
    auto *data = new OBNSimMsg::MSGDATA;
    data->set_t(t);
    data->set_i(mask);
    n2smn.set_allocated_data(data);
    
    msg.setMessage(n2smn);
    port.writeStrict();
    
    report_info(0, "Sent EVENT request for time " + std::to_string(t));
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
template <class PortIT, class ValIT>
void sendXbar(PortIT pbegin, PortIT pend, ValIT vbegin, ValIT vend, OBNnode::simtime_t t) {
    OBNSimIOMsg::DoubleVector msg;
    msg.set_time(t);

    for (int i=0; vbegin != vend; ++vbegin, ++i) {
        msg.add_value(*vbegin);
    }

    for (; pbegin != pend; ++pbegin) {
        OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleVector> & output = (*pbegin).prepare();
        output.setMessage(msg);
        (*pbegin).writeStrict();
    }
}

int main() {
    std::cout << "This is the ADMM master node." << std::endl;
    
    const int N = 3;  // number of slaves
    const int dim = 3;  // dimension of the x vector
    const int nIter = 20;   // number of iterations
    
    yarp::os::Network yarp;
    
    OBNnode::YARP::smnPort smnPort;  // The SMN control port
    
    // Input ports for velocity and setpoint
    // yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> > veloPort, setpointPort;
    
    // Output port for control value
    // yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> > ctrlPort;
    
    bool ok = smnPort.open("/test3/_admm_");
    
    // Open ports to connect to slaves
    std::array<yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleVector> >, N> slavePorts;
    for (int i=0; i < N; ++i) {
        ok = ok && slavePorts[i].open("/test3/admm/x" + std::to_string(i));
    }

    if (!ok) {
        std::cout << "Failed to create ports." << std::endl;
        return 1;
    }

    // Make sure that no system messages from SMN are dropped
    smnPort.setStrict();
    
    std::cout << "SMN Port opened." << std::endl;
    
    enum NODESTATE {
        NODE_INIT,  // Node hasn't been initialized yet (or has been terminated)
        NODE_RUN,    // Node is running
        NODE_EVENTACK   // Node is running but waiting for EVENT_ACK from SMN
    } nodeState = NODE_INIT;

    OBNnode::YARP::smnMsg* msg;
    OBNSimMsg::SMN2N smn2n;
    OBNSimMsg::N2SMN n2smn;
    
    int admmIter;  // Number of iteration of the ADMM algorithm
    
    std::array<double, dim> xbar;  // the xbar vector
    
    std::bitset<N> slaveRes;       // bitset to track responses (results) from the slaves
    
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    
    // Generate random time duration between iterations of the ADMM algorithm
    std::uniform_int_distribution<int> distribution(5,40);
    
    
    bool simRunning = true;
    OBNnode::simtime_t curTime;
    int myID;
    
    OBNSimMsg::N2SMN::MSGTYPE nextACKType;  // The next ACK type to send to SMN (Y_ACK or YI_ACK)
    
    std::ofstream dump("admm.txt");
    const char tab = '\t';
    
    std::cout << "Simulation started." << std::endl;
    
    while (simRunning) {
        // Waiting for the next message from GC
        msg = smnPort.read();
        
        if (msg) {
            msg->getMessage(smn2n);
            
            curTime = smn2n.time();
            
            if (smn2n.has_id()) {
                myID = smn2n.id();
            }
            
            // std::cout << "Got a message: id=" << myID << " type=" << smn2n.msgtype() << std::endl;
            
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
                            assert(smn2n.has_data() && smn2n.data().has_i());
                            switch (smn2n.data().i()) {
                                case 0x01:  // The normal update
                                    // Restart the ADMM loop
                                    admmIter = 0;
                                    
                                    // Register an irregular event
                                    nextACKType = OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK;
                                    sendEventRequest(myID, curTime + distribution(generator), 0x02, n2smn, smnPort);
                                    
                                    report_info(0, "At " + std::to_string(curTime) + " UPDATE_Y: starts ADMM.");
                                    
                                    nodeState = NODE_EVENTACK;
                                    break;
                                case 0x02:  // The ADMM iteration
                                    // Continue the ADMM loop
                                    if (++admmIter >= nIter) {
                                        // ADMM completes, acknowledges and stops
                                        sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK, n2smn, smnPort);
                                        report_info(0, "At " + std::to_string(curTime) + " UPDATE_Y: completes ADMM.");
                                    }
                                    else {
                                        // Send xbar to every slave
                                        slaveRes.reset();
                                        sendXbar(slavePorts.begin(), slavePorts.end(), xbar.begin(), xbar.end(), curTime);
                                        
                                        // std::cout << "Sent Xbar to all." << std::endl;
                                        
                                        // Reset xbar to compute the new mean
                                        xbar.fill(0.0);
                                        
                                        // Wait for returns from all slaves
                                        std::array<double, N> xj;  // to store xj from slave
                                        while (!slaveRes.all()) {
                                            for (int i = 0; i < N; ++i) {
                                                if (!slaveRes[i] && readInput(slavePorts[i], xj.begin())) {
                                                    // Got the result from slave i -> add to xbar
                                                    std::transform(xbar.begin(), xbar.end(), xj.begin(), xbar.begin(), std::plus<double>());
                                                    slaveRes.set(i);
                                                }
                                            }
                                        }
                                        
                                        // std::cout << "Got X back from all." << std::endl;
                                        
                                        // All slaves responded, xbar contains the sum, now calculate the mean
                                        std::transform(xbar.begin(), xbar.end(), xbar.begin(), [N](double v) {return v/N;});
                                        
                                        // Register an irregular event
                                        nextACKType = OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK;
                                        sendEventRequest(myID, curTime + distribution(generator), 0x02, n2smn, smnPort);
                                        
                                        // report_info(0, "At " + std::to_string(curTime) + " UPDATE_Y: continue ADMM.");
                                        
                                        nodeState = NODE_EVENTACK;
                                    }
                                    break;
                                default:
                                    report_warning(1, "Unknown UPDATE_Y mask " + std::to_string(smn2n.data().i()));
                                    break;
                            }
                            break;
                            
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_X:
                            // readInput(veloPort, v);
                            // readInput(setpointPort, sp);
                            
                            // Update the state
                            
                            // At this point, the inputs are all up-to-date, regardless of the order, so we can dump log data
                            dump << curTime;
                            for (auto it: xbar) {
                                dump << tab << it;
                            }
                            dump << std::endl;
                            
                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_X_ACK, n2smn, smnPort);
                            report_info(0, "At " + std::to_string(curTime) + " UPDATE_X");
                            break;
                            
                        default:
                            report_warning(0, "Unknown message type " + std::to_string(smn2n.msgtype()) + " during RUNNING.");
                            break;
                    }
                    break;
                case NODE_EVENTACK:
                    if (smn2n.msgtype() == OBNSimMsg::SMN2N_MSGTYPE_SIM_EVENT_ACK) {
                        if (smn2n.has_data() && smn2n.data().has_i() && smn2n.data().i() == 0) {
                            // OK
                            report_info(0, "Irregular event request accepted.");
                        }
                        else {
                            // Error
                            report_error(0, "Irregular event request denied with error code: " + std::to_string(smn2n.data().i()));
                            
                            // In this case, the ADMM loop stops.
                        }
                        sendACK(myID, nextACKType, n2smn, smnPort);
                        nodeState = NODE_RUN;
                    }
                    else {
                        report_warning(0, "Unknown message type " + std::to_string(smn2n.msgtype()) + " during EVENTACK.");
                    }

                    break;
                    
                case NODE_INIT:
                    switch (smn2n.msgtype()) {
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_INIT:
                            // Initialization
                            xbar.fill(0.0);
                            
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
    }
    
    std::cout << "Simulation finished." << std::endl;
    
    
    std::cout << "Goodbye!" << std::endl;
    
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    dump.close();
    // system("gnuplot ../../plot.plg");
    
    return 0;
}
