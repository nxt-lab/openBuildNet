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

#include <iostream>
#include <thread>
#include <chrono>

#include <obnsmn_report.h>
#include <obnsmn_gc.h>   // The GC thread

//#include <chaiscript/chaiscript.hpp>
#include <smnchai_api.h>    // Chaiscript API for SMN



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


int main() {
    yarp::os::Network yarp;
    
    chaiscript::ChaiScript chai;
    
    SMNChai::WorkSpace ws("test2");
    
    SMNChai::registerSMNAPI(chai, ws);
    
//    const char* code = R"chai(
//    var motor = Node("motor");
//    motor.add_input("vol");
//    motor.add_output("v");
//    motor.add_update(0, 10);
//    motor.output_from_update(0, "v");
//    motor.input_to_update(0, "vol", false);
//    
//    var ctrl = Node("ctrl");
//    ctrl.add_input("v");
//    ctrl.add_input("sp");
//    ctrl.add_output("u");
//    ctrl.add_update(0, 10);
//    ctrl.output_from_update(0, "u");
//    ctrl.input_to_update(0, "v", true);
//    ctrl.input_to_update(0, "sp", true);
//    
//    var sp = Node("sp");
//    sp.add_output("sp");
//    sp.add_update(0, 200);
//    sp.need_updateX(false);
//    sp.output_from_update(0, "sp");
//    
//    add_node(motor);
//    add_node(ctrl);
//    add_node(sp);
//    
//    connect(sp.port("sp"), ctrl.port("sp"));
//    connect(motor.port("v"), ctrl.port("v"));
//    connect(ctrl.port("u"), motor.port("vol"));
//    
//    final_time(1000);
//    )chai";
    
    try {
        chai.eval_file("code.txt");
        ws.print();
    } catch (const chaiscript::exception::eval_error &e) {
        std::cout << "Chaiscript error:\n" << e.pretty_print() << std::endl;
        return 1;
    } catch (const SMNChai::smnchai_exception &e) {
        std::cerr << "SMNChai error:\n" << e.what() << std::endl;
        return 2;
    } catch (const std::exception &e) {
        // Everything else ...
        std::cerr << "Runtime error:\n" << e.what() << std::endl;
        throw;
    }
    
    
//    SMNChai::Node motor("motor"), ctrl("ctrl"), sp("sp");
//
//    try {
//        // In this test, we create three nodes: motor, controller, and setpoint source
//        motor.add_input("vol");
//        motor.add_output("v");
//        motor.add_update(0, 10);
//        motor.output_from_update(0, "v");
//        motor.input_to_update(0, "vol", false);
//        
//        ctrl.add_input("v");
//        ctrl.add_input("sp");
//        ctrl.add_output("u");
//        ctrl.add_update(0, 10);
//        ctrl.output_from_update(0, "u");
//        ctrl.input_to_update(0, "v", true);
//        ctrl.input_to_update(0, "sp", true);
//        
//        sp.add_output("sp");
//        sp.add_update(0, 200);
//        sp.set_need_updateX(false);
//        sp.output_from_update(0, "sp");
//        
//        ws.add_node(&motor);
//        ws.add_node(&ctrl);
//        ws.add_node(&sp);
//        
//        ws.connect(sp.port("sp"), ctrl.port("sp"));
//        ws.connect(motor.port("v"), ctrl.port("v"));
//        ws.connect(ctrl.port("u"), motor.port("vol"));
//        
//        ws.print();
//    } catch (SMNChai::smnchai_exception const &e) {
//        std::cerr << "An error happened: " << e.what() << std::endl;
//        return 1;
//    }
    
    // The Global clock thread
    OBNsmn::GCThread gc;
    
    // The YARP communication thread for GC's incoming port
    OBNsmn::YARP::YARPPollingThread yarpThread(&gc, ws.get_full_path("_smn_", "_gc_"));
    if (!yarpThread.openPort()) {
        std::cerr << "Could not open the main GC port on the SMN." << std::endl;
        return 1;
    }
   
    try {
        ws.generate_obn_system(gc);
        
    } catch (SMNChai::smnchai_exception const &e) {
        std::cerr << "An error happened: " << e.what() << std::endl;
        return 1;
    }
    
    // Configure the GC
    //gc.ack_timeout = 0;
    //gc.setFinalSimulationTime(1000);

    if (!yarpThread.startThread()) {
        std::cout << "Error: could not start GC thread." << std::endl;
        return 1;
    }
    
    // Start running the GC thread
    if (!gc.startThread()) {
        std::cout << "Error: could not start GC thread." << std::endl;
        
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
    std::cout << "In this test, we only wait until the simulation stops..." << std::endl;
    
    //Join the threads with the main thread
    gc.joinThread();
    yarpThread.joinThread();
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return 0;
}
