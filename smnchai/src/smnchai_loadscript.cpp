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

#include <boost/filesystem.hpp>     // manipulate paths

#include <smnchai.h>        // Common defs
#include <smnchai_api.h>    // Chaiscript API for SMN

// Load the static Chaiscript library if needed
#ifdef SMNCHAI_CHAISCRIPT_STATIC
#include <chaiscript/chaiscript_stdlib.hpp>
#endif


const char *SMNCHAI_ENV_VAR = "SMNCHAI_DIR";    // environment variable that contains the path to the main SMNChai directory
const char *SMNCHAI_STDLIB_NAME = "stdlib.chai";    // name of the standard library file


// Return true if simulation should continue, false if should exit with given return code in the second value
std::pair<bool, int> SMNChai::smnchai_loadscript(const std::string& script_file, const std::vector<std::string>& script_args, const std::string& default_workspace,
                        OBNsmn::GCThread& gc, SMNChai::SMNChaiComm& comm)
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
        show_usage();
        return std::make_pair(false, 1);
    }
    
    // Construct the library path and check that it exists
    smnchai_main_dir /= "libraries";
    if (!boost::filesystem::exists(smnchai_main_dir) || !boost::filesystem::is_directory(smnchai_main_dir)) {
        std::cerr << "ERROR: SMNChai library directory " << smnchai_main_dir << " does not exist.\nPlease check the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
        show_usage();
        return std::make_pair(false, 1);
    }
    
    // Check that the standard library exists
    {
        boost::filesystem::path stdlib = smnchai_main_dir;
        stdlib /= SMNCHAI_STDLIB_NAME;
        if (!boost::filesystem::exists(stdlib) || !boost::filesystem::is_regular_file(stdlib)) {
            std::cerr << "ERROR: SMNChai standard library " << stdlib << " does not exist.\nYou may want to check the environment variable " << SMNCHAI_ENV_VAR << "\n\n";
            show_usage();
            return std::make_pair(false, 1);
        }
    }
    
    std::vector<std::string> chai_usepath;
    chai_usepath.emplace_back(smnchai_main_dir.string() + boost::filesystem::path::preferred_separator);
    //std::cout << chai_usepath[0] << std::endl;
    
#ifndef SMNCHAI_CHAISCRIPT_STATIC
    chaiscript::ChaiScript chai(std::vector<std::string>(), chai_usepath);  // Dynamic standard library + search path
#else
    chaiscript::ChaiScript chai(chaiscript::Std_Lib::library(), std::vector<std::string>(), chai_usepath);  // Static standard library + search path
#endif
    
    SMNChai::WorkSpace ws(default_workspace);  // default workspace name is the name of the script file
    SMNChai::registerSMNAPI(chai, ws);
    
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
    if (!ws.m_settings.m_run_simulation) {
        std::cout << "THE SIMULATION IS SET NOT TO BE RUN AUTOMATICALLY (DRY-RUN)." << std::endl;
        return std::make_pair(false, 0);
    }
    
    std::cout << "Done loading the script file. Now constructing the simulation network...\n";
    
    // Need to create and start the communication threads here to get certain messages from the nodes
    bool create_yarp = (comm.yarpThread == nullptr);
    bool create_mqtt = (comm.mqttClient == nullptr);
    
    if (ws.is_comm_protocol_used(SMNChai::COMM_YARP)) {
        // Create and Open Yarp port
#ifdef OBNSIM_COMM_YARP
        if (create_yarp) {
            comm.yarpThread = new OBNsmn::YARP::YARPPollingThread(&gc, "");
        }
        
        // Set the GC port name on this SMN
        comm.yarpThread->setPortName('/' + ws.get_full_path("_smn_", "_gc_"));  // YARP requires / at the beginning
        
        // Must open the GC port on this SMN because other nodes will be connected to it in generate_obn_system()
        if (!comm.yarpThread->openPort()) {
            std::cerr << "ERROR: Could not open the main GC port on the SMN." << std::endl;
            if (create_yarp) {
                // Delete the YARP thread here because we created it
                delete comm.yarpThread;
                comm.yarpThread = nullptr;
            }
            return std::make_pair(false, 5);
        }
        
        // Start Yarp communication
        if (!comm.yarpThread->startThread()) {
            std::cerr << "ERROR: could not start Yarp communication thread." << std::endl;
            if (create_yarp) {
                // Delete the YARP thread here because we created it
                delete comm.yarpThread;
                comm.yarpThread = nullptr;
            }
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
        if (create_mqtt) {
            comm.mqttClient = new OBNsmn::MQTT::MQTTClient(&gc);
        }
        comm.mqttClient->setClientID(ws.get_name());
        comm.mqttClient->setPortName(ws.get_full_path("_smn_", "_gc_"));
        comm.mqttClient->setServerAddress(ws.m_settings.m_mqtt_server);
        
        // Start MQTT communication
        if (!comm.mqttClient->start()) {
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
            if (create_mqtt) {
                delete comm.mqttClient;
                comm.mqttClient = nullptr;
            }
            return std::make_pair(false, 5);
        }
#else
        std::cerr << "Error: MQTT communication is not supported in this SMN." << std::endl;
        if (comm.yarpThread) {
            shutdown_communication_threads(gc);
            if (create_yarp) {
                delete comm.yarpThread;
                comm.yarpThread = nullptr;
            }
        }
        return std::make_pair(false, 5);
#endif
    }

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
        if (comm.mqttClient && create_mqtt) {
            delete comm.mqttClient;
            comm.mqttClient = nullptr;
        }
#endif
        
        return std::make_pair(false, 6);
    }
    
    return std::make_pair(true, 0);
}