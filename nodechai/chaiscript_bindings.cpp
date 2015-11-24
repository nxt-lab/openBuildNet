/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief All bindings with Chaiscript should be put in this file to allow faster compilation.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <memory>
#include "chaiscript_bindings.h"
#include <chaiscript/chaiscript.hpp>
#include <chaiscript/dispatchkit/bootstrap.hpp>

#ifdef OBNNODE_COMM_MQTT
#include "nodechai_mqtt.h"
#endif

using namespace chaiscript;

namespace NodeChai {
    
    /* All the global variables. */
    GlobalVariables global_variables;
    
    /** Add the API bindings after a node has been created, e.g. to create inputs, outputs, etc. */
    std::shared_ptr<chaiscript::Module> create_bindings_after_node(std::shared_ptr<chaiscript::Module> m = std::make_shared<chaiscript::Module>()) {
        if (!global_variables.node_created || !global_variables.node_factory) {
            // Node not yet created
            throw nodechai_exception("The node has not been created.");
        }
        
        // Create the specific bindings
        global_variables.node_factory->create_bindings(m);
        
        // Create the common bindings
        NodeFactory* pNF = global_variables.node_factory.get();
        m->add(fun(&NodeFactory::callback_init, pNF), "callback_init");
        m->add(fun(&NodeFactory::callback_term, pNF), "callback_term");
        m->add(fun(&NodeFactory::callback_x, pNF), "callback_x");
        m->add(fun(&NodeFactory::callback_y, pNF), "callback_y");
        
        return m;
    }
    
    /** Actually create the node. */
    void create_node() {
        if (global_variables.node_created || global_variables.node_factory) {
            // Node is already created
            throw nodechai_exception("Node has already been created; only one node is allowed.");
        }
        if (global_variables.node_name.empty() || !OBNsim::Utils::isValidNodeName(global_variables.node_name)) {
            throw nodechai_exception("Node name must be set and valid.");
        }
        if (!global_variables.chaiscript_engine) {
            throw nodechai_exception("Internal error: Chaiscript engine has not been initialized.");
        }

        // Create the factory object based on the chosen comm protocol
        switch (global_variables.comm_protocol) {
            case OBNnode::COMM_MQTT:
#ifdef OBNNODE_COMM_MQTT
                global_variables.node_factory.reset(new NodeFactoryMQTT());
#else
                throw nodechai_exception("MQTT is not supported.");
#endif
                break;
                
            default:
                throw nodechai_exception("Unsupported communication protocol.");
        }
        if (!global_variables.node_factory) {
            throw nodechai_exception("Failed to create the factory for the chosen communication protocol.");
        }
        
        // Actually create the node using the factory
        global_variables.node_created = global_variables.node_factory->create_node(global_variables.node_name, global_variables.workspace);
        if (!global_variables.node_created) {
            throw nodechai_exception("Failed to create the node.");
        }
        
        // Bind the API for running the node
        global_variables.chaiscript_engine->add(create_bindings_after_node());
    }
    
    std::shared_ptr<chaiscript::Module> create_bindings_before_node(std::shared_ptr<chaiscript::Module> m) {
        ////// Functions to set properties before creating a node
        m->add(fun([](const std::string& s) { global_variables.node_name = s; }), "set_node_name");
        m->add(fun([](const std::string& s) { global_variables.workspace = s; }), "set_workspace");
        m->add(fun([]() { global_variables.comm_protocol = OBNnode::COMM_YARP; }), "set_comm_yarp");
        m->add(fun([](const std::string& s) { global_variables.comm_protocol = OBNnode::COMM_MQTT; global_variables.mqtt_server = s; }), "set_comm_mqtt");
        
        ////// Function to actually create the node
        m->add(fun(&create_node), "create_node");
        
        return m;
    }
    
    
    
#ifdef OBNNODE_COMM_MQTT
    std::shared_ptr<chaiscript::Module> NodeFactoryMQTT::create_bindings(std::shared_ptr<chaiscript::Module> m) {
        
        ////////////////////////////////////////////////////////////////
        // Add the input port types and their methods to access
        ////////////////////////////////////////////////////////////////
        
        m->add(user_type<NodeFactoryMQTT::InputScalarDouble>(), "InputScalarDouble");
        m->add(fun(&NodeFactoryMQTT::InputScalarDouble::get), "get");
        m->add(fun(&NodeFactoryMQTT::InputScalarDouble::isValuePending), "pending");
        
        m->add(user_type<NodeFactoryMQTT::InputScalarDoubleStrict>(), "InputScalarDoubleStrict");
        m->add(fun(&NodeFactoryMQTT::InputScalarDoubleStrict::pop), "pop");
        m->add(fun(&NodeFactoryMQTT::InputScalarDoubleStrict::size), "size");
        m->add(fun(&NodeFactoryMQTT::InputScalarDoubleStrict::isValuePending), "pending");
        
        m->add(user_type<NodeFactoryMQTT::OutputScalarDouble>(), "OutputScalarDouble");
        m->add(fun(&NodeFactoryMQTT::OutputScalarDouble::operator()), "get");
        m->add(fun([](NodeFactoryMQTT::OutputScalarDouble& p, const double v) { return (p = v); }), "=");
        m->add(fun([](NodeFactoryMQTT::OutputScalarDouble& p, const double v) { p = v; }), "set");
        
        /*
        m->add(user_type<NodeFactoryMQTT::InputVectorDouble>(), "InputVectorDouble");
        m->add(user_type<NodeFactoryMQTT::InputVectorDoubleStrict>(), "InputVectorDoubleStrict");
        m->add(user_type<NodeFactoryMQTT::OutputVectorDouble>(), "OutputVectorDouble");
        
        m->add(user_type<NodeFactoryMQTT::InputMatrixDouble>(), "InputMatrixDouble");
        m->add(user_type<NodeFactoryMQTT::InputMatrixDoubleStrict>(), "InputMatrixDoubleStrict");
        m->add(user_type<NodeFactoryMQTT::OutputMatrixDouble>(), "OutputMatrixDouble");
         */

        ////////////////////////////////////////////////////////////////
        // Methods to create ports
        ////////////////////////////////////////////////////////////////
        
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputScalarDouble>, this), "new_input_double");
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputScalarDoubleStrict>, this), "new_input_double_strict");
        m->add(fun(&NodeFactoryMQTT::chai_create_output<OutputScalarDouble>, this), "new_output_double");

        /*
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputVectorDouble>, this), "new_input_double_vector");
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputVectorDoubleStrict>, this), "new_input_double_vector_strict");
        m->add(fun(&NodeFactoryMQTT::chai_create_output<OutputVectorDouble>, this), "new_output_double_vector");

        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputMatrixDouble>, this), "new_input_double_matrix");
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputMatrixDoubleStrict>, this), "new_input_double_matrix_strict");
        m->add(fun(&NodeFactoryMQTT::chai_create_output<OutputMatrixDouble>, this), "new_output_double_matrix");
        */

        return m;
    }
#endif
}