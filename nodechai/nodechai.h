/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basics of nodechai.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef NODECHAI_NODECHAI_H
#define NODECHAI_NODECHAI_H

#include <memory>
#include <functional>
#include <unordered_map>
#include <map>
#include <obnnode.h>
#include "chaiscript_bindings.h"

namespace NodeChai {
    /** The exception class. */
    class nodechai_exception: public std::exception {
        std::string m_message;
    public:
        const char* what() const noexcept {
            return m_message.c_str();
        }
        
        nodechai_exception(const std::string &msg): m_message(msg) { }
        nodechai_exception(const char *msg): m_message(msg) { }
    };
    
    /** The abstract factory class for creating nodes. */
    struct NodeFactory {
        virtual ~NodeFactory() { }  // To make sure the factory object will be destroyed properly
        
        /** Create the node object.
         This method will only be called once.
         It should create a new node object and store it inside this factory object.
         \return true if successful.
         */
        virtual bool create_node(const std::string& t_name, const std::string& t_workspace) = 0;
        
        /** Destroy the node object.
         This method is called by the main program when the node should be destroyed, before the program exits.
         The node is destroyed explicitly because it may contain pointers to Chaiscript functions, therefore we want to make sure that the node is destroyed before the Chaiscript engine is destroyed.
         */
        virtual void destroy_node() = 0;
        
        /** Return the created node object as a NodeBase object. Could be nullptr. */
        virtual OBNnode::NodeBase* get_node_object() const = 0;
        
        /** Return the textual description of the last error. */
        virtual std::string get_last_error() const = 0;
        
        /** Create and add a computation block.
         \return true if successful.
         */
        virtual bool add_block(int t_id,
                               OBNnode::UpdateType::UPDATE_CALLBACK t_yfunc,
                               OBNnode::UpdateType::UPDATE_CALLBACK t_xfunc,
                               double t_T,
                               const OBNnode::UpdateType::INPUT_LIST& t_inputs,
                               const OBNnode::UpdateType::OUTPUT_LIST& t_outputs,
                               const std::string& t_name) = 0;
        
        
        /* These methods modify or create (if necessary) callback functions.
         For callbacks associated with computation blocks, if the given block does not exist, it will be created.
         */
        virtual bool callback_x(int t_id, const OBNnode::UpdateType::UPDATE_CALLBACK& t_f) = 0;
        virtual bool callback_y(int t_id, const OBNnode::UpdateType::UPDATE_CALLBACK& t_f) = 0;
        virtual bool callback_init(const std::function<void ()>& f) = 0;
        virtual bool callback_term(const std::function<void ()>& f) = 0;
        
        /** Create the bindings specific for this node factory. */
        virtual std::shared_ptr<chaiscript::Module> create_bindings(std::shared_ptr<chaiscript::Module> m = std::make_shared<chaiscript::Module>()) = 0;
    };
    
    /** A structure that stores all the global variables for the script. */
    struct GlobalVariables {
        // Common settings
        OBNnode::CommProtocol comm_protocol{OBNnode::COMM_MQTT};
        std::string mqtt_server{"tcp://localhost:1883"};
        std::string node_name{""};
        std::string workspace{""};
        double timeout{-1.0}; // The timeout value for the node
        
        // The Chaiscript object to run the script.
        // It's a pointer to the real object, which should be created and destroyed by the main program.
        chaiscript::ChaiScript* chaiscript_engine{nullptr};
        
        std::unique_ptr<NodeFactory> node_factory; // Pointer to the one and only node factory
        bool node_created{false};   // Whether a node has been created
    };
    
    extern GlobalVariables global_variables;
    
    
    /** Common implementation of a node factory. */
    template <typename T>
    class NodeFactoryBase: public NodeFactory {
    protected:
        std::string m_last_error;
        std::shared_ptr<T> m_node;  ///< The node object, once it's created
        
        /** Map of input and output ports in this node. */
        std::unordered_map< std::string, std::shared_ptr<OBNnode::InputPortBase> > m_input_ports;
        std::unordered_map< std::string, std::shared_ptr<OBNnode::OutputPortBase> > m_output_ports;
        
        /** Check if a node has been created and generate appropriate error message. */
        bool check_node_object() {
            if (!m_node) {
                m_last_error = "A node has not been created.";
                return false;
            }
            return true;
        }
    public:
        virtual bool create_node(const std::string& t_name, const std::string& t_workspace) override {
            if (m_node) {
                // Already created -> error
                m_last_error = "A node has already been created.";
                return false;
            }
            
            try {
                m_node = std::make_shared<T>(t_name, t_workspace);
            } catch (...) {
                m_last_error = "Error while creating the node object.";
                return false;
            }
            
            // Try to open the GC port
            if (m_node->openSMNPort()) {
                return true;
            } else {
                // Delete the node and return error
                m_node.reset();
                m_last_error = "Could not open the system port on the node; check the communication network and configuration.";
                return false;
            }
        }
        
        /** Return the created node object as a NodeBase object. */
        virtual OBNnode::NodeBase* get_node_object() const override {
            return m_node.get();
        }
        
        /** Destroy the node object. */
        virtual void destroy_node() override {
            if (m_node) {
                // Reset the shared pointer to (try to) destroy the object
                m_node.reset();
            }
        }
        
        /** Return the textual description of the last error. */
        virtual std::string get_last_error() const override {
            return m_last_error;
        }
        
        /** Create and add a computation block.
         \return true if successful.
         */
        virtual bool add_block(int t_id,
                               OBNnode::UpdateType::UPDATE_CALLBACK t_yfunc = OBNnode::UpdateType::UPDATE_CALLBACK(),
                               OBNnode::UpdateType::UPDATE_CALLBACK t_xfunc = OBNnode::UpdateType::UPDATE_CALLBACK(),
                               double t_T = -1.0,
                               const OBNnode::UpdateType::INPUT_LIST& t_inputs = OBNnode::UpdateType::INPUT_LIST(),
                               const OBNnode::UpdateType::OUTPUT_LIST& t_outputs = OBNnode::UpdateType::OUTPUT_LIST(),
                               const std::string& t_name = "") override
        
        {
            if (!check_node_object()) {
                return false;
            }
            
            int result;
            if (t_id < 0) {
                // Auto assign the ID
                result = m_node->addUpdate(t_yfunc, t_xfunc, t_T, t_inputs, t_outputs, t_name);
            } else {
                result = m_node->addUpdate(t_id, t_yfunc, t_xfunc, t_T, t_inputs, t_outputs, t_name);
            }
            if (result >= 0) {
                return true;
            }
            switch (result) {
                case -1:
                    m_last_error = "The ID of the computation block is out of range, or no more could be added.";
                    break;
                case -2:
                    m_last_error = "A computation block already exists with the given ID.";
                    break;
                case -3:
                    m_last_error = "A given input port does not exist.";
                    break;
                case -4:
                    m_last_error = "A given output port does not exist.";
                    break;
                default:
                    m_last_error = "Unknown error while creating a computation block.";
                    break;
            }
            return false;
        }
    };
    
    
    /** Command-line arguments can be given to the node script in the form of named arguments:
                keyword=value
     This function processes the command-line arguments to the program and constructs a map of key-value pairs.
     It will throw an exception (nodechai_exception) if there is any invalid argument.
     Note that the first command-line argument must be the script file name and it is returned by this function; if it's nullptr, the script file is not provided.
     In Chaiscript, to convert the string value to other types, use to_*, e.g., to_double(), to_int()...
     \exception nodechai_exception
    */
    const char* process_commandline_args(int argc, char **argv, std::map<std::string, chaiscript::Boxed_Value>& argmap);

}

#endif