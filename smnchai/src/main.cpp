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

#include <obnsmn_report.h>
#include <obnsmn_gc.h>   // The GC thread

#include <smnchai_api.h>    // Chaiscript API for SMN

// Load the static Chaiscript library if needed
#ifdef SMNCHAI_CHAISCRIPT_STATIC
#include <chaiscript/chaiscript_stdlib.hpp>
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



/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNSIM_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_SMN_COMM_MQTT: if MQTT is supported for communication.
 */


const char *copyright = R"txt(
+--------------------------------------------------------------------+
|         SMNChai : scriptable SMN server for openBuildNet.          |
+--------------------------------------------------------------------+

This program is part of the openBuildNet framework developed at EPFL.
)txt";

const char *SMNCHAI_ENV_VAR = "SMNCHAI_DIR";    // environment variable that contains the path to the main SMNChai directory
const char *SMNCHAI_STDLIB_NAME = "stdlib.chai";    // name of the standard library file

// The usage of this program
void show_usage(const char *prog) {
    std::cout << "Usage:\n" <<
    "  " << prog << " SCRIPT [ARGS]\n\n" <<
    "where\n" <<
    "  SCRIPT is the name of the main Chaiscript file\n" <<
    "  ARGS is an optional list of arguments to the script file.\n";
}

int main(int argc, const char* argv[]) {
    std::cout << copyright << std::endl;
    
    // Check input arguments
    if (argc < 2) {
        // Not enough input args
        std::cerr << "ERROR: Not enough input arguments\n\n";
        show_usage(argv[0]);
        return 1;
    }

    yarp::os::Network yarp;
    
    // The Global clock thread
    OBNsmn::GCThread gc;
    
    // The YARP communication thread for GC's incoming port
    OBNsmn::YARP::YARPPollingThread yarpThread(&gc, "");

    // *************
    // Running Chaiscript to load the network results in large objects, so we use block code to free them after we've done with them
    {
        // Get the main directory of SMNChai
        boost::filesystem::path smnchai_main_dir;
        {
            char *p = getenv(SMNCHAI_ENV_VAR);
            if (p) {
                smnchai_main_dir = p;
            } else {
                // Environment variable not available -> use current directory
                smnchai_main_dir = boost::filesystem::current_path();
            }
        }
        
        if (!boost::filesystem::exists(smnchai_main_dir) || !boost::filesystem::is_directory(smnchai_main_dir)) {
            std::cerr << "ERROR: SMNChai main directory is invalid. Please set the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
            show_usage(argv[0]);
            return 1;
        }
        
        // Construct the library path and check that it exists
        smnchai_main_dir /= "libraries";
        if (!boost::filesystem::exists(smnchai_main_dir) || !boost::filesystem::is_directory(smnchai_main_dir)) {
            std::cerr << "ERROR: SMNChai library directory " << smnchai_main_dir << " does not exist.\nPlease check the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
            show_usage(argv[0]);
            return 1;
        }
        
        // Check that the standard library exists
        {
            boost::filesystem::path stdlib = smnchai_main_dir;
            stdlib /= SMNCHAI_STDLIB_NAME;
            if (!boost::filesystem::exists(stdlib) || !boost::filesystem::is_regular_file(stdlib)) {
                std::cerr << "ERROR: SMNChai standard library " << stdlib << " does not exist.\nYou may want to check the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
                show_usage(argv[0]);
                return 1;
            }
        }
        
        // Get and check the script file
        boost::filesystem::path script_file(argv[1]);
        if (!boost::filesystem::exists(script_file) || !boost::filesystem::is_regular_file(script_file)) {
            std::cerr << "ERROR: The script file " << script_file << " does not exist.\n\n";
            show_usage(argv[0]);
            return 1;
        }
        
        // Extract input arguments to the script
        std::vector<const std::string> script_args;
        for (auto i = 2; i < argc; ++i) {
            script_args.emplace_back(argv[i]);
        }
        
        
        std::vector<std::string> chai_usepath;
        chai_usepath.emplace_back(smnchai_main_dir.string() + boost::filesystem::path::preferred_separator);
        //std::cout << chai_usepath[0] << std::endl;
        
#ifndef SMNCHAI_CHAISCRIPT_STATIC
        chaiscript::ChaiScript chai(std::vector<std::string>(), chai_usepath);  // Dynamic standard library + search path
#else
        chaiscript::ChaiScript chai(chaiscript::Std_Lib::library(), std::vector<std::string>(), chai_usepath);  // Static standard library + search path
#endif
        
        SMNChai::WorkSpace ws(script_file.stem().string());  // default workspace name is the name of the script file
        SMNChai::registerSMNAPI(chai, ws);
        
        std::cout << "Loading the Chaiscript file: " << script_file << std::endl;
        try {
            chai.use(SMNCHAI_STDLIB_NAME);
            chai.eval_file(script_file.string());
        } catch (const chaiscript::exception::eval_error &e) {
            std::cout << "Chaiscript error:\n" << e.pretty_print() << std::endl;
            return 1;
        } catch (const SMNChai::smnchai_exception &e) {
            std::cerr << "SMNChai error:\n" << e.what() << std::endl;
            return 2;
        } catch (chaiscript::Boxed_Value &e) {
            std::string s(chaiscript::boxed_cast<const std::string&>(e));
            std::cerr << "User code error:\n" << s << std::endl;
            return 3;
        } catch (const std::exception &e) {
            // Everything else ...
            std::cerr << "Runtime error:\n" << e.what() << std::endl;
            throw;
        }
        
        
        // If it's set not to run the simulation, we can exit now
        if (!ws.settings.run_simulation) {
            std::cout << "THE SIMULATION IS SET NOT TO BE RUN AUTOMATICALLY (DRY-RUN)." << std::endl;
            return 0;
        }
        
        // Set the GC port name on this SMN
        yarpThread.setPortName(ws.get_full_path("_smn_", "_gc_"));
        
        // Must open the GC port on this SMN because other nodes will be connected to it in generate_obn_system()
        if (!yarpThread.openPort()) {
            std::cerr << "ERROR: Could not open the main GC port on the SMN." << std::endl;
            return 1;
        }
        
        std::cout << "Done loading the script file. Now constructing the simulation network...\n";
        try {
            ws.generate_obn_system(gc);
        } catch (SMNChai::smnchai_exception const &e) {
            std::cerr << "ERROR: " << e.what() << std::endl;
            return 1;
        }
    }
    // Done with running Chaiscript to load the network -> memory is freed, now we only need to run the simulation
    // *************
    
    
    if (!yarpThread.startThread()) {
        std::cerr << "ERROR: could not start GC thread." << std::endl;
        return 1;
    }
    
    std::cout << "Done constructing the network.\nStart simulation...\n";
    
    // Start running the GC thread
    if (!gc.startThread()) {
        std::cerr << "ERROR: could not start GC thread." << std::endl;
        
        // As yarpThread is already running, we should try to stop it cleanly
        gc.simple_thread_terminate = true;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        if (yarpThread.done_execution) {
            // Good
            google::protobuf::ShutdownProtobufLibrary();
            return 1;
        } else {
            // Crap, we have to terminate
            std::terminate();
        }
    }
    
    // The main thread will only wait until the GC thread stops
    
    //Join the threads with the main thread
    gc.joinThread();
    yarpThread.joinThread();
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
