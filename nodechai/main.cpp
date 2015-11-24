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

#include <nodechai.h>
#include <chaiscript/chaiscript.hpp>
#include "chaiscript_stdlib.h"

using namespace NodeChai;

typedef std::function<void (const std::string&)> FUNCTYPE;
FUNCTYPE the_func;
double the_num = 0.0;

void register_func(FUNCTYPE f) {
    the_func = f;
}

double someoperator(double a, double b) {
    return a + b;
}

int main() {
    chaiscript::ChaiScript chai(NodeChai::create_chaiscript_stdlib());
    global_variables.chaiscript_engine = &chai;

    /*
    chai.add(var(&the_num), "num");
    
    // Call the func
    //register_func(chai.eval<FUNCTYPE>("fun(s) {print(s);}"));
    chai.add(fun(&register_func), "register_func");
    chai.eval("register_func(fun(s) {print(s);});");
    
    the_func("Hello from Chaiscript");
    
    chai.eval("num = 1.5 + 2.5;");
    std::cout << the_num << std::endl;
    */
    
    // Bind the API
    chai.add(NodeChai::create_bindings_before_node());
    
    // Run the script
    chai.eval_file("test.chai");
    
    // Check if node is created, then run it
    if (global_variables.node_created && global_variables.node_factory) {
        auto the_node = global_variables.node_factory->get_node_object();
        std::cout << "The node " << the_node->name() << " was created.\n";
        
        // Run it
        the_node->run();
        
        global_variables.node_factory->destroy_node();
    }
    
    return 0;
}
