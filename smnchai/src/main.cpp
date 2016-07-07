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
#include <signal.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <map>

#include <boost/filesystem.hpp>     // manipulate paths

#include <chaiscript/chaiscript.hpp>
#include <smnchai.h>

// At least one of the communication protocols must be supported
#if !defined(OBNSIM_COMM_YARP) && !defined(OBNSIM_COMM_MQTT)
#error At least one communication protocol must be supported (YARP, MQTT)
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

// Function to shut down the SMN properly
// should be called before exiting NORMALLY, but not when being terminated unexpectedly
void shutdown_SMN() {
    // Wait a bit before shutting down so that we won't overload the nameserver (hopefully)
    //std::this_thread::sleep_for(std::chrono::seconds(2));
    
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
}

// The main GC thread object once it's created
// Only use this pointer in special occasions, e.g. in the handler when the program exits unexpectedly
OBNsmn::GCThread *main_gcthread = nullptr;

void shutdown_communication_threads(OBNsmn::GCThread& gc) {
    gc.simple_thread_terminate = true;
    
#ifdef OBNSIM_COMM_MQTT
    if (comm_objects.mqttClient) {
        // Loop until MQTT finishes or a max timeout
        int niters = 0;
        while (niters++ <= 9 && comm_objects.mqttClient->outMsgCount() > 0) {
            // Messages are still pending -> wait
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        comm_objects.mqttClient->stop();    // Force stop MQTT
    }
#endif
    
#ifdef OBNSIM_COMM_YARP
    if (comm_objects.yarpThread && !comm_objects.yarpThread->done_execution) {
        // Wait a fixed amount of time for Yarp
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
#endif
}

// The handler called when the program is interrupted unexpectedly (e.g. Ctrl-C)
static void interrupt_signal_handler(int signal_value) {
    std::cout << "Interrupted by user or system." << std::endl;
    
    // If the GC is available, try to stop it immediately
    if (main_gcthread) {
        std:: cout << "Try to terminate the simulation system cleanly..." << std::endl;
        main_gcthread->setSysRequest(OBNsmn::GCThread::SYSREQ_TERMINATE);
        std::this_thread::sleep_for(std::chrono::seconds(2));   // Wait a bit to let GC handle the request
        
        shutdown_communication_threads(*main_gcthread);
    }
    
    // We won't delete the objects, etc., just let the program exit abnormally
}

// Get the named arguments to the script
bool process_script_args(int argc, char** argv, std::map<std::string, chaiscript::Boxed_Value>& argmap)
{
    // We consider argument #2 onwards as named arguments to the node script; Argument #1 is the script file name.
    char* equalsign = nullptr;
    for (auto i=2; i < argc; ++i) {
        // Each named argument must have the form "keyword=value" and the keyword must be unique
        
        equalsign = std::strchr(argv[i], '=');  // Find the equal sign position
        if (!equalsign || equalsign == argv[i]) {
            // '=' not found or keyword is empty -> invalid
            std::cerr << "ERROR: Invalid named argument: " << argv[i] << '\n';
            return false;
        }
        
        // Extract the key string and trim it
        std::string keystr(OBNsim::Utils::trim(std::string(argv[i], equalsign-argv[i])));
        if (keystr.empty()) {
            std::cerr << "ERROR: Named argument has empty key: " << argv[i] << '\n';
            return false;
        }
        
        // Check if the key already exists
        if (argmap.count(keystr) > 0) {
            std::cerr << "ERROR: Key of named argument is not unique: " <<  keystr << '\n';
            return false;
        }
        
        // Now we can register the key-value pair to the map
        argmap.emplace(keystr, chaiscript::Boxed_Value(std::string(equalsign+1)));
    }
    
    return true;
}



int main(int argc, char* argv[]) {
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
    std::map<std::string, chaiscript::Boxed_Value> arguments_map;   // The map of arguments to the node script
    if (!process_script_args(argc, argv, arguments_map)) {
        show_usage();
        return 2;
    }
    
    {
        // Set the handler for unexpected termination
        struct sigaction action;
        action.sa_handler = interrupt_signal_handler;
        action.sa_flags = 0;
        sigemptyset (&action.sa_mask);
        sigaction (SIGINT, &action, NULL);
        sigaction (SIGTERM, &action, NULL);
    }
    
    {
        // The Global clock thread
        OBNsmn::GCThread gc;
        main_gcthread = &gc;
        
        // Run Chaiscript to load the network
        auto load_script_result = SMNChai::smnchai_loadscript(script_file.string(), arguments_map, script_file.stem().string(), gc, comm_objects);
        
        if (!load_script_result.first) {
            shutdown_SMN();
            return load_script_result.second;
        }
        
        // Done with running Chaiscript to load the network, now we only need to run the simulation
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
        
        // Shutdown communications
        shutdown_communication_threads(gc);
        
        comm_objects.joinThreads();
        
        main_gcthread = nullptr;    // No more access to the GC
    }
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    std::cout << "SHUTTING DOWN THE SERVER..." << std::endl;
    shutdown_SMN();
    
    return 0;
}
