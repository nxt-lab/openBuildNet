/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Node that implements a simple controller.
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
    std::cout << "This is controller node." << std::endl;
    
    yarp::os::Network yarp;
    
    OBNnode::YARP::smnPort smnPort;  // The SMN control port
    
    // Input ports for velocity and setpoint
    yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> > veloPort, setpointPort;
    
    // Output port for control value
    yarp::os::BufferedPort< OBNnode::YARP::YARPMsg<OBNSimIOMsg::DoubleScalar> > ctrlPort;
    
    bool ok = smnPort.open("/test2/_ctrl_");
    ok = ok && veloPort.open("/test2/ctrl/v") && setpointPort.open("/test2/ctrl/sp") && ctrlPort.open("/test2/ctrl/u");
    
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
    
    double x1, x2, x3, x1_new, x2_new, x3_new;  // The state variables
    double v;  // This is the velocity input, saved to be used in calculation later
    double sp;  // This is the setpoint input, saved to be used in calculation later
    double u;       // The control value, to be sent out
    
    bool simRunning = true;
    OBNnode::simtime_t curTime;
    int myID;
    
    std::ofstream dump("controller.txt");
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
            
            switch (nodeState) {
                case NODE_RUN:
                    switch (smn2n.msgtype()) {
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_Y: {
                            readInput(veloPort, v);
                            readInput(setpointPort, sp);

                            // Calculate output and send out
                            u = 12.62 * x1 - 19.75 * x2 + 7.625 * x3;
                            
                            sendOutput(ctrlPort, u, curTime);

                            std::ostringstream message;
                            message << "At " << curTime << " UPDATE_Y mask = 0x" << std::hex << smn2n.data().i();
                            report_info(0, message.str());

                            sendACK(myID, OBNSimMsg::N2SMN_MSGTYPE_SIM_Y_ACK, n2smn, smnPort);
                            
                            break;
                        }
                            
                        case OBNSimMsg::SMN2N_MSGTYPE_SIM_X:
                            readInput(veloPort, v);
                            readInput(setpointPort, sp);
                            
                            // Update the state
                            x1_new = -0.82 * x1 + x2 + 0.82 * x3 + 32.0*(sp - v);
                            x2_new = x1;
                            x3_new = x2;
                            x1 = x1_new;
                            x2 = x2_new;
                            x3 = x3_new;
                            
                            // At this point, the inputs are all up-to-date, regardless of the order, so we can dump log data
                            dump << curTime << tab << sp << tab << v << std::endl;
                            
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
                            x1 = 0.0;
                            x2 = 0.0;
                            x3 = 0.0;
                            v = 0.0;
                            sp = 0.0;
                            u = 0.0;
                            
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
    
    dump.close();
    system("gnuplot ../../plot.plg");
    
    return 0;
}
