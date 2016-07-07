/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file smnchai.h
 *
 * Common definitions used in SMNChai program.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * Header file for the node class, which holds information about a node in the simulation network.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef smnchai_smnchai_h
#define smnchai_smnchai_h

#include <chaiscript/chaiscript.hpp>

#include <obnsmn_report.h>
#include <obnsmn_gc.h>   // The GC thread

/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNSIM_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_COMM_MQTT: if MQTT is supported for communication.
 */

// At least one communication framework must be supported
#if !defined(OBNSIM_COMM_YARP) && !defined(OBNSIM_COMM_MQTT)
#error "At least one communication framework must be supported."
#endif

#ifdef OBNSIM_COMM_YARP
#include <obnsmn_comm_yarp.h>
#endif

#ifdef OBNSIM_COMM_MQTT
#include <obnsmn_comm_mqtt.h>
#endif

// The usage of this program
void show_usage();

// Function to shut down the SMN, should be called before exiting
void shutdown_SMN();

// Function to shut down communication threads
void shutdown_communication_threads(OBNsmn::GCThread& gc);

namespace SMNChai {
    /** A structure that contains the supported communication objects. */
    struct SMNChaiComm {
#ifdef OBNSIM_COMM_YARP
        OBNsmn::YARP::YARPPollingThread* yarpThread = nullptr;
#endif
#ifdef OBNSIM_COMM_MQTT
        OBNsmn::MQTT::MQTTClient* mqttClient = nullptr;
#endif
        // Check whether all communication threads have finished their execution
        bool allFinished() const {
#ifdef OBNSIM_COMM_YARP
            if (yarpThread && !yarpThread->done_execution) {
                return false;
            }
#endif
#ifdef OBNSIM_COMM_MQTT
            if (mqttClient && mqttClient->isRunning()) {
                return false;
            }
#endif
            return true;
        }
        
        // Join all communication threads (to finish them)
        void joinThreads() {
#ifdef OBNSIM_COMM_YARP
            if (yarpThread) {
                yarpThread->joinThread();
            }
#endif
#ifdef OBNSIM_COMM_MQTT
#endif
        }
    };
    
    /** The function to load the Chaiscript simulation file.
     \param comm Reference to a structure containing the pointers to the communication client objects. This function will fill the structure with dynamic objects. The caller (the SMN program) must delete these object upon exiting.
     \return First value is true if the simulation will continue, false if the program should exit with the return code given in the second value.
     */
    std::pair<bool, int> smnchai_loadscript(const std::string& script_file, const std::map<std::string, chaiscript::Boxed_Value>& arguments_map, const std::string& default_workspace,
                                            OBNsmn::GCThread& gc, SMNChaiComm& comm);
}
#endif
