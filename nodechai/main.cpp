/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief The main function of the nodechai program.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

/** The following macros are defined by CMake to indicate which libraries this build supports:
 - OBNNODE_COMM_YARP: if YARP is supported for communication.
 - OBNNODE_COMM_MQTT: if MQTT is supported for communication.
 */

#ifndef OBNNODE_COMM_MQTT
#error Currently only MQTT is supported.
#endif

#include <exception>
#include "nodechai.h"
#include <chaiscript/chaiscript.hpp>
#include "chaiscript_stdlib.h"

#include <boost/filesystem.hpp>     // manipulate paths

using namespace NodeChai;

void show_usage(char *prog) {
    std::cout << "USAGE:" << std::endl <<
    prog << " <script_file> [key1=value1 [key2=value2 ...]]" << std::endl <<
    R"args(
where
  <script_file> is the path to the script file of the node;

followed by an optional list of named arguments to the node script.
Each named argument is a key-value pair of the form "key=value".
The keys must be unique.
)args";
}

void show_banner() {
    std::cout << R"banner(
+-------------------------------------------------------------------+
|                   openBuildNet Scriptable Node                    |
+-------------------------------------------------------------------+
        
This program is part of the openBuildNet framework developed at EPFL.

)banner";
}


// Clean up when there is an error and we need to quit
// DO: destroy the node object and the node factory, shut down ProtoBuf
void cleanup_if_error() {
    if (global_variables.node_created && global_variables.node_factory) {
        // Try to destroy the node object
        global_variables.node_factory->destroy_node();
        global_variables.node_created = false;
        
        // Try to destroy the node factory by resetting the smart pointer
        global_variables.node_factory.reset();
    }
    google::protobuf::ShutdownProtobufLibrary();
}

int main(int argc, char **argv) {
    show_banner();
    
    //////////////////////////
    // Extract the arguments
    //////////////////////////
    
    // The map of arguments to the node script
    std::map<std::string, chaiscript::Boxed_Value> arguments_map;
    
    // The script file name
    const char *script_file = NodeChai::process_commandline_args(argc, argv, arguments_map);
    
    if (!script_file) {
        // Not enough input args
        std::cout << "Not enough input arguments." << std::endl;
        show_usage(argv[0]);
        exit(1);
    }

    // Construct the module path and use path for Chaiscript
    std::vector<std::string> chai_usepath;
    std::vector<std::string> modulepaths;

    // Module path contains the current directory and the path in CHAI_MODULE_PATH, if defined
    modulepaths.push_back("");
    {
        const char *modulepath = getenv("CHAI_MODULE_PATH");
        if (modulepath)
        {
            std::string s_usepath(modulepath);
            if (s_usepath.back() == boost::filesystem::path::preferred_separator) {
                modulepaths.emplace_back(s_usepath);
            } else {
                modulepaths.emplace_back(s_usepath + boost::filesystem::path::preferred_separator);
            }
        }
    }
    
    // Add the use paths: the current path (as "") and the directory containing the script file, and CHAI_USE_PATH if defined
    chai_usepath.emplace_back(""); // boost::filesystem::current_path().string() + boost::filesystem::path::preferred_separator);
    
    {
        // Check that the script file exists
        boost::filesystem::path script_file_path = boost::filesystem::canonical(boost::filesystem::path(script_file));
        if (!boost::filesystem::exists(script_file_path) || !boost::filesystem::is_regular_file(script_file_path)) {
            std::cout << "ERROR: The given script file is invalid.\n";
            show_usage(argv[0]);
            exit(2);
        }
        
        // Extract the path of the script file and add it to the use path if not the same as current path
        boost::filesystem::path script_file_parent = script_file_path.parent_path();
        if (script_file_parent.compare(boost::filesystem::current_path()) != 0) {
            chai_usepath.emplace_back(script_file_parent.string() + boost::filesystem::path::preferred_separator);
        }

        const char *usepath = getenv("CHAI_USE_PATH");
        if (usepath) {
            std::string s_usepath(usepath);
            if (s_usepath.back() == boost::filesystem::path::preferred_separator) {
                chai_usepath.emplace_back(s_usepath);
            } else {
                chai_usepath.emplace_back(s_usepath + boost::filesystem::path::preferred_separator);
            }
        }
    }


    // Create the ChaiScript engine
    chaiscript::ChaiScript chai(NodeChai::create_chaiscript_stdlib(), modulepaths, chai_usepath);
    global_variables.chaiscript_engine = &chai;
    
    // Add the named arguments to the chai engine as a const map variable
    chai.add(chaiscript::const_var(&arguments_map), "args");
  
    // Bind the API
    chai.add(NodeChai::create_bindings_before_node());

    // Run the script
    try {
        chai.eval_file(script_file);
    } catch (...) {
        cleanup_if_error();
        std::rethrow_exception(std::current_exception());
    }
    
    // Check if node is created, then run it
    if (global_variables.node_created && global_variables.node_factory) {
        OBNnode::NodeBase* the_node = global_variables.node_factory->get_node_object();
        std::cout << "Starting simulation of node " << the_node->name() << " ...\n";
        
        // Run it
        try {
            the_node->run(global_variables.timeout);
        } catch (...) { // const std::exception& e
            //std::cout << "Error: " << e.what() << std::endl;
            
            // Try to stop the simulation
            the_node->stopSimulation();
            cleanup_if_error();
            the_node = nullptr;
            std::rethrow_exception(std::current_exception());
        }
        
        // If we reach here, everything is ok and the simulation has finished, so destroy the node
        the_node->delayBeforeShutdown();
        
        global_variables.node_factory->destroy_node();
        global_variables.node_created = false;
        the_node = nullptr;
        
        // The node factory will be automatically destroyed when the program exits thanks to the smart pointer
    }
    
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
