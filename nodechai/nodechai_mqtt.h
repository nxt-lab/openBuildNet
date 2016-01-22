/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basics of nodechai for MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef NODECHAI_NODECHAI_MQTT_H
#define NODECHAI_NODECHAI_MQTT_H

#ifndef OBNNODE_COMM_MQTT
#error MQTT must be enabled to use this header file.
#endif

#include <obnnode_mqttnode.h>
#include "nodechai.h"

namespace NodeChai {
    class NodeFactoryMQTT;
    
    /** Similar to MQTTNode but with callbacks for init, termination, etc. */
    class MQTTNodeChai: public OBNnode::MQTTNode {
        /** The callbacks for initialization and termination. */
        std::function<void ()> m_onInitialization_callback, m_onTermination_callback;
        
        friend class NodeFactoryMQTT;
    public:
        MQTTNodeChai(const std::string& t_name, const std::string& t_workspace): OBNnode::MQTTNode(t_name, t_workspace) { }
        
        virtual void onInitialization() override {
            if (m_onInitialization_callback) {
                m_onInitialization_callback();
            }
        }
        
        virtual void onTermination() override {
            if (m_onTermination_callback) {
                m_onTermination_callback();
            }
        }
    };
    
    /** The abstract factory class for creating nodes. */
    class NodeFactoryMQTT: public NodeFactoryBase<MQTTNodeChai> {
        std::string m_mqtt_server;  ///< Address of the MQTT server
    public:
        typedef OBNnode::MQTTInput<OBNnode::OBN_PB, double> InputScalarDouble;
        typedef OBNnode::MQTTInput<OBNnode::OBN_PB, double, true> InputScalarDoubleStrict;
        typedef OBNnode::MQTTOutput<OBNnode::OBN_PB, double> OutputScalarDouble;

        typedef OBNnode::MQTTInput<OBNnode::OBN_PB, OBNnode::obn_vector<double> > InputVectorDouble;
        typedef OBNnode::MQTTInput<OBNnode::OBN_PB, OBNnode::obn_vector<double>, true> InputVectorDoubleStrict;
        typedef OBNnode::MQTTOutput<OBNnode::OBN_PB, OBNnode::obn_vector<double> > OutputVectorDouble;

        typedef OBNnode::MQTTInput<OBNnode::OBN_PB, OBNnode::obn_matrix<double> > InputMatrixDouble;
        typedef OBNnode::MQTTInput<OBNnode::OBN_PB, OBNnode::obn_matrix<double>, true> InputMatrixDoubleStrict;
        typedef OBNnode::MQTTOutput<OBNnode::OBN_PB, OBNnode::obn_matrix<double> > OutputMatrixDouble;
        
        /** Constructor of MQTTNode factory, with given MQTT server address. */
        NodeFactoryMQTT(const std::string& t_mqttserver): m_mqtt_server(t_mqttserver) { }
        
        /** Create an MQTTNode object. */
        virtual bool create_node(const std::string& t_name, const std::string& t_workspace) override {
            if (m_node) {
                // Already created -> error
                m_last_error = "A node has already been created.";
                return false;
            }
            
            try {
                m_node = std::make_shared<MQTTNodeChai>(t_name, t_workspace);
            } catch (...) {
                m_last_error = "Error while creating the node object.";
                return false;
            }
            
            // Set the server settings
            m_node->setServerAddress(m_mqtt_server);
            
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

        ///////////////////////////////////////////////////////////////////////////////
        /* These methods create inputs and outputs of various types.
         It's guaranteed that create_node() was called successfully before any of these methods is called, so the node object should have existed.
         The methods should return valid port objects if successful.
         */
        template <typename T>
        std::shared_ptr<T> create_input(const std::string& t_name) {
            if (!check_node_object()) {
                return nullptr;
            }
            
            if (m_input_ports.find(t_name) != m_input_ports.end()) {
                m_last_error = std::string("An input with name ") + t_name + " already existed.";
                return nullptr;
            }
            
            if (m_output_ports.find(t_name) != m_output_ports.end()) {
                m_last_error = std::string("An output with name ") + t_name + " already existed.";
                return nullptr;
            }
            
            auto port = std::make_shared<T>(t_name);
            if (!port) {
                m_last_error = std::string("Error while creating input ") + t_name + ".";
                return nullptr;
            }
            
            // Add the port to the node
            if (m_node->addInput(port.get())) {
                // Add the port to the list
                m_input_ports.emplace(t_name, port);
                return port;
            } else {
                m_last_error = std::string("Could not add input ") + t_name + " to the node.";
                return nullptr;
            }
        }
        
        template <typename T>
        std::shared_ptr<T> chai_create_input(const std::string& t_name) {
            auto p = create_input<T>(t_name);
            if (p) {
                return p;
            }
            throw nodechai_exception(m_last_error);
        }
        
        template <typename T>
        std::shared_ptr<T> create_output(const std::string& t_name) {
            if (!check_node_object()) {
                return nullptr;
            }
            
            if (m_input_ports.find(t_name) != m_input_ports.end()) {
                m_last_error = std::string("An input with name ") + t_name + " already existed.";
                return nullptr;
            }
            
            if (m_output_ports.find(t_name) != m_output_ports.end()) {
                m_last_error = std::string("An output with name ") + t_name + " already existed.";
                return nullptr;
            }
            
            auto port = std::make_shared<T>(t_name);
            if (!port) {
                m_last_error = std::string("Error while creating input ") + t_name + ".";
                return nullptr;
            }
            
            // Add the port to the node
            if (m_node->addOutput(port.get())) {
                // Add the port to the list
                m_output_ports.emplace(t_name, port);
                return port;
            } else {
                m_last_error = std::string("Could not add input ") + t_name + " to the node.";
                return nullptr;
            }
        }
        
        template <typename T>
        std::shared_ptr<T> chai_create_output(const std::string& t_name) {
            auto p = create_output<T>(t_name);
            if (p) {
                return p;
            }
            throw nodechai_exception(m_last_error);
        }
        
        /* These methods modify or create (if necessary) callback functions.
         For callbacks associated with computation blocks, if the given block does not exist, it will be created.
         */
        virtual bool callback_x(int t_id, const OBNnode::UpdateType::UPDATE_CALLBACK& t_f) override;
        virtual bool callback_y(int t_id, const OBNnode::UpdateType::UPDATE_CALLBACK& t_f) override;
        virtual bool callback_init(const std::function<void ()>& f) override;
        virtual bool callback_term(const std::function<void ()>& f) override;
        
        virtual std::shared_ptr<chaiscript::Module> create_bindings(std::shared_ptr<chaiscript::Module> m = std::make_shared<chaiscript::Module>()) override;
    };
}

#endif