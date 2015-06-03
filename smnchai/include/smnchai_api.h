/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Chaiscrip API for SMNChai.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef SMNCHAI_SMNCHAI_API_H
#define SMNCHAI_SMNCHAI_API_H

#include <exception>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <forward_list>
#include <utility>      // std::pair
#include <limits>       // limits of integers (time, etc.)
#include <iostream>

#include <obnsim_basic.h>
#include <obnsmn_gc.h>   // The GC thread

#ifndef OBNSIM_COMM_YARP
#error SMNChai currently requires YARP
#endif

#include <obnsmn_comm_yarp.h>

#include <chaiscript/chaiscript.hpp>


/** \file
 The SMNChai API are the basic types and functions used to define a complete OBN Simulation network using the Chaiscript language.
 The WorkSpace class represents such a simulation network, consisting of nodes and their connections.
 It also contains all settings for the simulation, e.g. the final time.
 All time values in the WorkSpace are real numbers in microseconds, which will be converted to the appropriate integer time values in the specified time unit when the workspace is converted into an actual GC object for simulation.
 */

namespace SMNChai {

    /** The main exception class of SMNChai, for propagating errors from ChaiScript to main program. */
    class smnchai_exception: public std::exception {
        std::string m_message;
    public:
        const char* what() const noexcept {
            return m_message.c_str();
        }
        
        smnchai_exception(const std::string &msg): m_message(msg) { }
        smnchai_exception(const char *msg): m_message(msg) { }
    };
    
    
    /** \brief Information about a port on a node, used for specifying connections. */
    struct PortInfo {
        const std::string node_name;
        const std::string port_name;
        const enum PortType { INPUT, OUTPUT, DATA } port_type;
        
        PortInfo(const std::string &t_node, const std::string t_port, PortType t_type):
        node_name(t_node), port_name(t_port), port_type(t_type)
        { }
    };
    
    class WorkSpace;
    
    /** \brief Class that represents a node, to be created in Chaiscript
     */
    class Node {
        // DATA MEMBERS
        std::string m_name;     ///< Name of the node
        
        bool m_updateX = true;         ///< Whether this node needs UPDATE_X messages

        /** Set of physical inputs and associated updates for which an input has direct feedthrough. */
        std::unordered_map<std::string, OBNsim::updatemask_t> m_inputs;
        
        /** Set of physical outputs and associated updates that change the output. */
        std::unordered_map<std::string, OBNsim::updatemask_t> m_outputs;
        
        /** Set of data ports. */
        std::unordered_set<std::string> m_dataports;
        
        /** Set of update types: map from unique int ID to sampling period (in microseconds). */
        std::unordered_map<unsigned int, double> m_updates;
        
    private:
        
        /** Check if a given port's name exists. */
        bool port_exists(const std::string &t_name) const;
        
    public:
        /** Constructor of a node object given its name. The node is not yet added to the network.
         \exception smnchai_exception an error happens, e.g. invalid name.
         */
        Node(const std::string &t_name): m_name(t_name) {
            if (!OBNsim::Utils::isValidIdentifier(t_name)) {
                throw smnchai_exception("Node name '" + t_name + "' is an invalid identifier.");
            }
        }
        
        /** Return the name of this node. */
        std::string get_name() const { return m_name; }
        
        /** Set whether this node needs the UPDATE_X messages. */
        void set_need_updateX(bool b) { m_updateX = b; }
        
        /** \brief Add a new physical input port to the node.
         \param t_name The name of the port.
         \exception smnchai_exception an error happens, e.g. invalid name, port already exists...
         */
        void add_input(const std::string &t_name);
        
        /** \brief Add a new physical output port to the node.
         \param t_name The name of the port.
         \exception smnchai_exception an error happens, e.g. invalid name, port already exists...
         */
        void add_output(const std::string &t_name);
        
        /** \brief Add a new data output port to the node.
         \param t_name The name of the port.
         \exception smnchai_exception an error happens, e.g. invalid name, port already exists...
         */
        void add_dataport(const std::string &t_name);
        
        /** \brief Add a new update type to the node.
         \param t_id The unique ID of the update type, which must not exist already.
         \param t_period The sampling period of the update type in microseconds, can be 0 if it's not periodic.
         \exception smnchai_exception an error happens, e.g. ID is invalid (out of range), ID already used.
         */
        void add_update(unsigned int t_id, double t_period);
        
        /** \brief Specify an input port to an update.
         This method specifies that a given physical input port is an input to a given update.
         \param t_id The ID of the update, must be valid and already exist (already added to the node).
         \param t_port Name of the input port, must be an already-added physical input port of the node.
         \param t_direct Whether this input port has direct feedthrough for this update.
         \exception smnchai_exception an error happens, e.g. ID is invalid (out of range), ID not exists, port not exists, etc.
         */
        void input_to_update(unsigned int t_id, const std::string &t_port, bool t_direct);
        
        /** \brief Specify an output port from an update.
         This method specifies that a given physical output port is an output of a given update.
         \param t_id The ID of the update, must be valid and already exist (already added to the node).
         \param t_port Name of the output port, must be an already-added physical output port of the node.
         \exception smnchai_exception an error happens, e.g. ID is invalid (out of range), ID not exists, port not exists, etc.
         */
        void output_from_update(unsigned int t_id, const std::string &t_port);

        /** \brief Return a PortInfo object for connecting ports.
         \param t_port Port name
         \return The PortInfo object
         \exception smnchai_exception If the port name is invalid or does not exist.
         */
        PortInfo port(const std::string &t_port) const;
        
        /** \brief Create an Yarp node object for this node associated with the given system port.
         Note that the new Yarp node object will own the port object, i.e. when the node is destructed, it will also delete the port.
         \param sys_port Pointer to the system port on the SMN associated with this node; must be dynamically allocated as it will be deleted.
         \param ws The WorkSpace object to whom this node belongs (to access system settings).
         \return Pointer to the new node object; nullptr if there is any error.
         */
        OBNsmn::YARP::OBNNodeYARP* create_yarp_node(OBNsmn::YARP::YARPPort *sys_port, const WorkSpace &ws) const;
        
        /** Returns the update mask of a given input port, exception if port does not exist. */
        OBNsim::updatemask_t input_updatemask(const std::string &port_name) const {
            return m_inputs.at(port_name);
        }
        
        /** Returns the update mask of a given output port, exception if port does not exist. */
        OBNsim::updatemask_t output_updatemask(const std::string &port_name) const {
            return m_outputs.at(port_name);
        }
    };
    

    /** Class that represents a workspace, which contains nodes and their connections. */
    class WorkSpace {
        std::string m_name;     ///< Name of the workspace: all nodes will be under this name
        
        /** Mapping nodes' names to Node objects and their IDs (to be used later on). */
        std::unordered_map<std::string, std::pair<Node,std::size_t> > m_nodes;
        
        /** List of all connections between ports in this workspace. */
        std::forward_list< std::pair<PortInfo, PortInfo> > m_connections;
        
    public:
        /** Settings for the GC / simulation. */
        struct {
            int ack_timeout = 0;        ///< Timeout for ACK, in milliseconds.
            double final_time = std::numeric_limits<OBNsim::simtime_t>::max();      ///< The final time of simulation, real number in microseconds.
            double time_unit = 1.0;       ///< The atomic time unit, real number in microseconds [default = 1 microseconds]
        } settings;

    public:
        /** Construct a workspace object with a given name. */
        WorkSpace(const std::string &t_name = "") {
            set_name(t_name);
        }
        
        /** Set the workspace's name (throw an exception if invalid name). Empty is a valid name. */
        void set_name(const std::string &t_name) {
            if (!t_name.empty() && !OBNsim::Utils::isValidIdentifier(t_name)) {
                throw smnchai_exception("Workspace name '" + t_name + "' is invalid.");
            }
            m_name = t_name;
        }
        
        /** \brief Returns the full path to an object.
         \param t_obj1 Name of the first object.
         \param t_obj2 Optional name of the second object.
         \return Generally /workspace/obj1/obj2, where workspace and obj2 can be empty.
         */
        std::string get_full_path(const std::string &t_obj1, const std::string &t_obj2 = "") const;
        
        /** \brief Returns the full path to a port.
         \param t_port PortInfo object specifying a port.
         \return Generally /workspace/node/port, where workspace can be empty.
         */
        std::string get_full_path(const PortInfo &t_port) const;
        
        
        /** Add a node to the current workspace.
         \param p_node Reference to a node object, to be copied to this workspace.
         \exception smnchai_exception an error happenned, e.g. node's name already exists.
         */
        void add_node(const Node &p_node);

        /** Connect 2 ports by PortInfo objects, unless they are already connected.
         \param t_from The source port.
         \param t_to The target port.
         \sa Node::port()
         \exception smnchai_exception an error happenned, e.g. an input port can't be a source port.
         */
        void connect(PortInfo &&t_from, PortInfo &&t_to);
        
        /** Connect 2 ports by their names, unless they are already connected.
         \param from_node,from_port The source node and port.
         \param to_node,to_port The target node and port.
         \exception smnchai_exception an error happenned, e.g. node or port doesn't exist, invalid port type.
         */
        void connect(const std::string &from_node, const std::string &from_port, const std::string &to_node, const std::string &to_port);
        
        /** Utility function to print workspace's details to std::cout. */
        void print() const;
        
        /** \brief Generate the OBN system of this workspace into the given GC object.
         
         The following are created:
         - The system port on the SMN to talk with the remote node.
         - The connections declared in this workspace.
         - The connections between the SMN and the nodes.
         - The dependency graph.
         
         \exception smnchai_exception An error occurred; check its what() for details.
         */
        void generate_obn_system(OBNsmn::GCThread &gc);
        
        /** Returns the integer time value in terms of the time unit from the real time value t in microseconds.
         Returns 0 if the integer value is negative.
         */
        OBNsim::simtime_t get_time_value(double t) const;
    };
    
    /********** Utility and interface functions ************/
    
    /** \brief Register all API functions and types with a Chaiscript object. */
    void registerSMNAPI(chaiscript::ChaiScript &chai, WorkSpace &ws);

}


#endif  // SMNCHAI_SMNCHAI_API_H
