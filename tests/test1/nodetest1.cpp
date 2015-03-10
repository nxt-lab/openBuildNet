/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A test SMN which implements nodes inside the SMN itself.
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

#include <obnnode_basic.h>

#ifndef OBNSIM_COMM_YARP
#error This test requires YARP to run
#endif

#include <obnnode_comm_yarp.h>

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


int main() {
    std::cout << "This is nodetest1, a very simple node." << std::endl;
    
    yarp::os::Network yarp;
    
    OBNnode::YARP::smnPort smnPort;
    
    bool ok = smnPort.open("/test1/_node1_");
    
    if (!ok) {
        std::cout << "Failed to create ports." << std::endl;
        return 1;
    }
    
    // Make sure that no system messages from SMN are dropped
    smnPort.setStrict();

    std::cout << "Port opened." << std::endl;
    
    enum NODESTATE {
        NODE_INIT,  // Node hasn't been initialized yet (or has been terminated)
        NODE_RUN    // Node is running
    } nodeState = NODE_INIT;

    OBNnode::YARP::smnMsg* msg;
    OBNSimMsg::SMN2N smn2n;
    OBNSimMsg::N2SMN n2smn;
    
    while (true) {
        bool simRunning = true;
        OBNnode::simtime_t curTime;
        int myID;

        std::cout << "Simulation started." << std::endl;
        std::cout.flush();
        
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
                                sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK, n2smn, smnPort);
                                
                                std::ostringstream message;
                                message << "At " << curTime << " UPDATE_Y mask = 0x" << std::hex << smn2n.data().i();
                                report_info(0, message.str());
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
        
        char cmd;
        std::cout << "Simulation finished." << std::endl;

        do {
            std::cout << "Do you want to continue (c) or quit (q)? ";
            std::cin >> cmd;
        } while (cmd != 'c' && cmd != 'C' && cmd != 'q' && cmd != 'Q');
        
        if (cmd == 'q' || cmd == 'Q') {
            break;
        }
    }
    
    std::cout << "Goodbye!" << std::endl;
    
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
