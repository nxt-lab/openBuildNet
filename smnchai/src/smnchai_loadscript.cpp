/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Load the Chaiscript file of the simulation system.
 *
 * Made a separate C++ file to improve build performance.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <fstream>
#include <boost/filesystem.hpp>     // manipulate paths

#include "smnchai.h"        // Common defs
#include "smnchai_api.h"    // Chaiscript API for SMN

#include <chaiscript/chaiscript.hpp>

// Load the static Chaiscript library if needed
#ifdef SMNCHAI_CHAISCRIPT_STATIC
#include "chaiscript_stdlib.h"
#endif


const char *SMNCHAI_ENV_VAR = "SMNCHAI_DIR";    // environment variable that contains the path to the main SMNChai directory
const char *SMNCHAI_STDLIB_NAME = "stdlib.chai";    // name of the standard library file

// Return true if simulation should continue, false if should exit with given return code in the second value
std::pair<bool, int> SMNChai::smnchai_loadscript(const std::string& script_file,
                                                 const std::map<std::string, chaiscript::Boxed_Value>& arguments_map,
                                                 const std::string& default_workspace,
                                                 OBNsmn::GCThread& gc,
                                                 SMNChai::SMNChaiComm& comm,
                                                 const SMNChai::SystemSettings& sys_settings)        // Whether to generate the node list for Docker (without running simulation)
{
    // Get the main directory of SMNChai
    boost::filesystem::path smnchai_main_dir;
    bool smnchai_main_dir_defined = false;
    {
        char *p = getenv(SMNCHAI_ENV_VAR);
        if (p) {
            smnchai_main_dir = p;
            
            // Check the path
            if (!boost::filesystem::exists(smnchai_main_dir) || !boost::filesystem::is_directory(smnchai_main_dir)) {
                std::cerr << "ERROR: SMNChai main directory is invalid. Please set the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
                return std::make_pair(false, 1);
            }

            // Construct the library path and check that it exists
            smnchai_main_dir /= "libraries";
            if (!boost::filesystem::exists(smnchai_main_dir) || !boost::filesystem::is_directory(smnchai_main_dir)) {
                std::cerr << "ERROR: SMNChai library directory " << smnchai_main_dir << " does not exist.\nPlease check the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
                return std::make_pair(false, 1);
            }

            smnchai_main_dir_defined = true;
        } else {
            // Environment variable not available -> use current directory
            smnchai_main_dir = boost::filesystem::current_path();
        }
    }

    // Check that the standard library exists
    {
        boost::filesystem::path stdlib = smnchai_main_dir;
        stdlib /= SMNCHAI_STDLIB_NAME;
        if (!boost::filesystem::exists(stdlib) || !boost::filesystem::is_regular_file(stdlib)) {
            std::cerr << "ERROR: SMNChai standard library " << stdlib << " does not exist.\nYou may want to check the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
            return std::make_pair(false, 1);
        }
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
    
    // Add the use paths: the current path (as "") and the directory containing the script file
    chai_usepath.emplace_back(""); // boost::filesystem::current_path().string() + boost::filesystem::path::preferred_separator);
    
    {
        // Check that the script file exists
        boost::filesystem::path script_file_path = boost::filesystem::canonical(boost::filesystem::path(script_file));
        if (!boost::filesystem::exists(script_file_path) || !boost::filesystem::is_regular_file(script_file_path)) {
            std::cerr << "ERROR: The given script file is invalid.\n";
            return std::make_pair(false, 2);
        }
        
        // Extract the path of the script file and add it to the use path if not the same as current path
        boost::filesystem::path script_file_parent = script_file_path.parent_path();
        if (script_file_parent.compare(boost::filesystem::current_path()) != 0) {
            chai_usepath.emplace_back(script_file_parent.string() + boost::filesystem::path::preferred_separator);
        }
    }
    
    // Add the standard library path if it was defined
    if (smnchai_main_dir_defined) {
        chai_usepath.emplace_back(smnchai_main_dir.string() + boost::filesystem::path::preferred_separator);
    }
    
    // Add CHAI_USE_PATH if defined
    {
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

    
#ifndef SMNCHAI_CHAISCRIPT_STATIC
    chaiscript::ChaiScript chai(modulepaths, chai_usepath);  // Dynamic standard library + search path
#else
    chaiscript::ChaiScript chai(SMNChai::create_chaiscript_stdlib(), modulepaths, chai_usepath);  // Static standard library + search path
#endif
    
    SMNChai::WorkSpace ws(default_workspace, comm, gc);  // default workspace name is the name of the script file
    
    // Transfer the system settings to WorkSpace settings
    if (sys_settings.dryrun || sys_settings.dockerlist) {
        ws.m_settings.m_sys_run_simulation = false;     // Force no-simulation
        ws.m_settings.m_dockerlist = sys_settings.dockerlist;
    }
    
    SMNChai::registerSMNAPI(chai, ws);
    
    // Add the named arguments to the chai engine as a const map variable
    chai.add(chaiscript::const_var(&arguments_map), "args");
    
    std::cout << "Loading the Chaiscript file: " << script_file << std::endl;
    try {
        chai.use(SMNCHAI_STDLIB_NAME);
        chai.eval_file(script_file);
    } catch (const chaiscript::exception::eval_error &e) {
        std::cout << "Chaiscript error:\n" << e.pretty_print() << std::endl;
        return std::make_pair(false, 2);
    } catch (const SMNChai::smnchai_exception &e) {
        std::cerr << "SMNChai error:\n" << e.what() << std::endl;
        return std::make_pair(false, 3);
    } catch (chaiscript::Boxed_Value &e) {
        std::string s(chaiscript::boxed_cast<const std::string&>(e));
        std::cerr << "User code error:\n" << s << std::endl;
        return std::make_pair(false, 4);
    } catch (const std::exception &e) {
        // Everything else ...
        std::cerr << "Runtime error:\n" << e.what() << std::endl;
        throw;
    }
    
    
    // If it's set not to run the simulation, we can exit now
    if (sys_settings.dryrun || !ws.m_settings.m_run_simulation) {
        std::cout << "\nTHE SIMULATION IS SET NOT TO BE RUN AUTOMATICALLY (DRY-RUN).\n";
        return std::make_pair(false, 0);
    }
    
    if (sys_settings.dockerlist) {
        std::cout << "\nGENERATE NODE LIST FOR DOCKER WITHOUT RUNNING THE SIMULATION.\n";
        try {
            std::ofstream dockerlistfile(sys_settings.dockerlistfile);
            ws.obndocker_dump(dockerlistfile);
            dockerlistfile.close();
        } catch (std::ofstream::failure e) {
            std::cerr << "Error while writing to Docker node list file.\n";
        }
        
        return std::make_pair(false, 0);
    }
    
    std::cout << "Done loading the script file. Now constructing the simulation network...\n";
    
    // Need to create and start the communication threads here to get certain messages from the nodes
#ifdef OBNSIM_COMM_YARP
    bool create_yarp = (comm.yarpThread == nullptr);
#endif
    
    if (ws.is_comm_protocol_used(SMNChai::COMM_YARP)) {
        // Create and Open Yarp port
#ifdef OBNSIM_COMM_YARP
        if (create_yarp) {
            comm.yarpThread = new OBNsmn::YARP::YARPPollingThread(&gc, "");
        }
        
        // Set the GC port name on this SMN
        comm.yarpThread->setPortName('/' + ws.get_full_path("_smn_", OBNsim::NODE_GC_PORT_NAME));  // YARP requires / at the beginning
        
        bool yarpSuccess = true;
        
        // Must open the GC port on this SMN because other nodes will be connected to it in generate_obn_system()
        if (!comm.yarpThread->openPort()) {
            std::cerr << "ERROR: Could not open the main GC port on the SMN." << std::endl;
            if (create_yarp) {
                // Delete the YARP thread here because we created it
                delete comm.yarpThread;
                comm.yarpThread = nullptr;
            }
            yarpSuccess = false;
        }
        
        // Start Yarp communication
        if (!comm.yarpThread->startThread()) {
            std::cerr << "ERROR: could not start Yarp communication thread." << std::endl;
            if (create_yarp) {
                // Delete the YARP thread here because we created it
                delete comm.yarpThread;
                comm.yarpThread = nullptr;
            }
            yarpSuccess = false;
        }
        
        if (!yarpSuccess) {
            // Delete the MQTT thread if it's already started
#ifdef OBNSIM_COMM_MQTT
            if (comm.mqttClient != nullptr) {
                comm.mqttClient->stop();
                delete comm.mqttClient;
                comm.mqttClient = nullptr;
            }
#endif
            return std::make_pair(false, 5);
        }
#else
        std::cerr << "ERROR: YARP communication is not supported in this SMN." << std::endl;
        return std::make_pair(false, 5);
#endif
    }
    
    if (ws.is_comm_protocol_used(SMNChai::COMM_MQTT)) {
        // Create and Open MQTT port
        // If fail, remember to also shut down Yarp if necessary (shutdown_communication_threads)
#ifdef OBNSIM_COMM_MQTT
        // Start MQTT communication
        if (!ws.start_mqtt_client()) {
            std::cerr << "ERROR: could not start MQTT communication thread." << std::endl;
#ifdef OBNSIM_COMM_YARP
            // Shut down Yarp
            if (comm.yarpThread) {
                shutdown_communication_threads(gc);
                if (create_yarp) {
                    delete comm.yarpThread;
                    comm.yarpThread = nullptr;
                }
            }
#endif
            // Delete the MQTT client
            if (comm.mqttClient != nullptr) {
                delete comm.mqttClient;
                comm.mqttClient = nullptr;
            }
            return std::make_pair(false, 5);
        }
#else
        std::cerr << "Error: MQTT communication is not supported in this SMN." << std::endl;
#ifdef OBNSIM_COMM_YARP
        if (comm.yarpThread) {
            shutdown_communication_threads(gc);
            if (create_yarp) {
                delete comm.yarpThread;
                comm.yarpThread = nullptr;
            }
        }
#endif
        return std::make_pair(false, 5);
#endif
    }
#ifdef OBNSIM_COMM_MQTT
    else if (comm.mqttClient) {
        comm.mqttClient->stop();
        delete comm.mqttClient;
    }
#endif

#ifdef OBNSIM_COMM_MQTT
    if (comm.mqttClient && ws.m_tracking_mqtt_online_nodes) {
        // The MQTT client is running and may be tracking nodes' availability -> stop tracking
        comm.mqttClient->stopListeningForArrivals();
        ws.m_tracking_mqtt_online_nodes = false;
    }
#endif
    
    try {
        ws.generate_obn_system(gc, comm);
    } catch (SMNChai::smnchai_exception const &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        
        // As the communication thread(s) already started, we try to signal them to stop
        shutdown_communication_threads(gc);
        
        // Delete them if we created them before
#ifdef OBNSIM_COMM_YARP
        if (comm.yarpThread && create_yarp) {
            delete comm.yarpThread;
            comm.yarpThread = nullptr;
        }
#endif
#ifdef OBNSIM_COMM_MQTT
        if (comm.mqttClient) {
            comm.mqttClient->stop();
            delete comm.mqttClient;
            comm.mqttClient = nullptr;
        }
#endif
        
        return std::make_pair(false, 6);
    }
    
    return std::make_pair(true, 0);
}