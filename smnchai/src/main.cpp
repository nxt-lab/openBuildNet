/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief The main SMNChai program.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cstdlib>      // getevn
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

#include <boost/filesystem.hpp>     // manipulate paths

#include <smnchai.h>


// Currently we require YARP for running remote nodes
#ifndef OBNSIM_COMM_YARP
#error YARP is required for SMNChai as YarpRun is used to start remote nodes
#endif

// Implement reporting functions for the SMN
void OBNsmn::report_error(int code, std::string msg) {
    std::cerr << "ERROR (" << code << "): " << msg << std::endl;
}

void OBNsmn::report_warning(int code, std::string msg) {
    std::cout << "WARNING (" << code << "): " << msg << std::endl;
}

void OBNsmn::report_info(int code, std::string msg) {
    std::cout << "INFO (" << code << "): " << msg << std::endl;
}


const char *copyright = R"txt(
+--------------------------------------------------------------------+
|         SMNChai : scriptable SMN server for openBuildNet.          |
+--------------------------------------------------------------------+

This program is part of the openBuildNet framework developed at EPFL.
)txt";

const char *SMNChai_program_name;

// This structure will contain the pointers to the communication client objects (or threads)
SMNChai::SMNChaiComm comm_objects;


// The usage of this program
void show_usage() {
    std::cout << "Usage:\n" <<
    "  " << SMNChai_program_name << " SCRIPT [ARGS]\n\n" <<
    "where\n" <<
    "  SCRIPT is the name of the main Chaiscript file\n" <<
    "  ARGS is an optional list of arguments to the script file.\n";
}

// Function to shut down the SMN, should be called before exiting
void shutdown_SMN() {
    // Delete the communication objects (which are created dynamically in loadscript())
#ifdef OBNSIM_COMM_YARP
    if (comm_objects.yarpThread) {
        delete comm_objects.yarpThread;
        comm_objects.yarpThread = nullptr;
    }
#endif
#ifdef OBNSIM_COMM_MQTT
    if (comm_objects.mqttClient) {
        delete comm_objects.mqttClient;
        comm_objects.mqttClient = nullptr;
    }
#endif
    
    // Shutdown ProtoBuf
    google::protobuf::ShutdownProtobufLibrary();
    
    // Wait a bit before shutting down so that we won't overload the nameserver (hopefully)
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

void shutdown_communication_threads(OBNsmn::GCThread& gc) {
    gc.simple_thread_terminate = true;
    std::this_thread::sleep_for(std::chrono::seconds(2));
}


int main(int argc, const char* argv[]) {
    // Save the program name
    SMNChai_program_name = argv[0];
    
#ifdef OBNSIM_COMM_YARP
    yarp::os::Network yarp;
    yarp.setVerbosity(-1);
#endif
    
    std::cout << copyright << std::endl;
    
    // Check input arguments
    if (argc < 2) {
        // Not enough input args
        std::cerr << "ERROR: Not enough input arguments\n\n";
        show_usage();
        return 1;
    }
    
    // Get and check the script file
    boost::filesystem::path script_file(argv[1]);
    if (!boost::filesystem::exists(script_file) || !boost::filesystem::is_regular_file(script_file)) {
        std::cerr << "ERROR: The script file " << script_file << " does not exist.\n\n";
        show_usage();
        return 1;
    }
    
    // Extract input arguments to the script
    std::vector<std::string> script_args;
    for (auto i = 2; i < argc; ++i) {
        script_args.emplace_back(argv[i]);
    }
    
    // The Global clock thread
    OBNsmn::GCThread gc;
    
    // Run Chaiscript to load the network
    auto load_script_result = SMNChai::smnchai_loadscript(script_file.string(), script_args, script_file.stem().string(), gc, comm_objects);

    // Done with running Chaiscript to load the network, now we only need to run the simulation
    if (!load_script_result.first) {
        return load_script_result.second;
    }
    
    std::cout << "Done constructing the network.\nStart simulation...\n";
    
    // Start running the GC thread
    if (!gc.startThread()) {
        std::cerr << "ERROR: could not start GC thread. Shutting down..." << std::endl;
        
        // As the communication thread(s) already started, we try to signal them to stop
        shutdown_communication_threads(gc);
        
        if (comm_objects.allFinished()) {
            // Good
            shutdown_SMN();
            return 1;
        } else {
            // Crap, we have to terminate
            shutdown_SMN();
            std::terminate();
        }
    }
    
    // The main thread will only wait until the GC thread stops
    
    //Join the threads with the main thread
    gc.joinThread();
    comm_objects.joinThreads();
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    std::cout << "SHUTTING DOWN THE SERVER..." << std::endl;
    shutdown_SMN();
    
    return 0;
}
