/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Node that implements a simple setpoint.
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

/* Read inputs */
void readInput(yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> > &port, double &val) {
    OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> * input = port.read(false);
    if (input) {
        // There is a message, extract the value
        OBNSimIOMsg::DoubleScalar msg;
        input->getMessage(msg);
        val = msg.value();
    }
}

/* Send output */
void sendOutput(yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> > &port, double val,
                OBNnode::simtime_t t) {
    OBNSimIOMsg::DoubleScalar msg;
    msg.set_value(val);
    msg.set_time(t);
    
    OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> & output = port.prepare();
    output.setMessage(msg);
    port.writeStrict();
}

int main() {
    std::cout << "This is setpoint node." << std::endl;
    
    yarp::os::Network yarp;
    
    OBNnode::YARP::smnPort smnPort;  // The SMN control port
    
    // Output port for SP value
    yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> > spPort;
    
    bool ok = smnPort.open("/test2/_sp_");
    ok = ok && spPort.open("/test2/sp/sp");
    
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
    
    double sp;       // The output setpoint, to be sent out
    
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> distribution(-100,100);
    
    bool simRunning = true;
    OBNnode::simtime_t curTime;
    int myID;
    
    //std::ofstream dump("motor.txt");
    //const char tab = '\t';
    
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
            
            switch (nodeState) {
                case NODE_RUN:
                    switch (smn2n.msgtype()) {
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_Y: {
                            // Generate random setpoint and send out
                            sp = distribution(generator) / 10.0;
                            
                            sendOutput(spPort, sp, curTime);
                            
                            std::ostringstream message;
                            message << "At " << curTime << " UPDATE_Y mask = 0x" << std::hex << smn2n.data().i();
                            report_info(0, message.str());
                            
                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK, n2smn, smnPort);
                            
                            break;
                        }
                            
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_X:
                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_X_ACK, n2smn, smnPort);
                            report_info(0, "At " + std::to_string(curTime) + " UPDATE_X");
                            break;
                            
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_TERM:
                            report_info(0, "At " + std::to_string(curTime) + " TERMINATE");
                            nodeState = NODE_INIT;
                            simRunning = false;
                            break;
                            
                        default:
                            report_warning(0, "Unknown message type " + std::to_string(smn2n.msgtype()) + " during RUNNING.");
                            break;
                    }
                    break;
                case NODE_INIT:
                    switch (smn2n.msgtype()) {
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_INIT:
                            // Initialization

                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_INIT_ACK, n2smn, smnPort);
                            report_info(0, "At " + std::to_string(curTime) + " INIT");
                            nodeState = NODE_RUN;
                            break;
                            
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_TERM:
                            report_info(0, "At " + std::to_string(curTime) + " TERMINATE");
                            simRunning = false;
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
    
    return 0;
}
