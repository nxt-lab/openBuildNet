/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implementation of the Chaiscript API of SMNChai.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cassert>
#include <algorithm>
#include <smnchai_api.h>

using namespace chaiscript;
using namespace SMNChai;

/** This function register all the functions, types, etc. provided by the SMNChai API to a given ChaiScript object.
 \param chai The ChaiScript object to which the API is registered.
 \param ws The WorkSpace object to which nodes and connections are added.
 */
void SMNChai::registerSMNAPI(ChaiScript &chai, WorkSpace &ws) {
    // Register the Node class and its constructor
    chai.add(constructor<Node (const std::string&)>(), "Node");
    chai.add(user_type<Node>(), "Node");
    
    // Methods to manipulate a node object: ports, updates, etc.
    chai.add(fun(&Node::add_input), "add_input");
    chai.add(fun(&Node::add_output), "add_output");
    chai.add(fun(&Node::add_dataport), "add_dataport");
    chai.add(fun(&Node::add_update), "add_update");
    chai.add(fun(&Node::set_need_updateX), "need_updateX");
    
    chai.add(fun(&Node::input_to_update), "input_to_update");
    chai.add(fun(&Node::output_from_update), "output_from_update");

    // Get PortInfo from a node, used for connecting ports
    chai.add(user_type<SMNChai::PortInfo>(), "PortInfo");
    chai.add(fun(&Node::port), "port");
    
    // Methods to work with the WorkSpace object: add nodes, connect ports...
    // These are bound with the given WorkSpace object
    chai.add(fun(&WorkSpace::add_node, &ws), "add_node");
    chai.add(fun<void (PortInfo, PortInfo)>([&ws](PortInfo s, PortInfo t) { ws.connect(std::move(s), std::move(t)); }), "connect");
    chai.add(fun<void (WorkSpace::*)(const std::string &, const std::string &, const std::string &, const std::string &)>(&WorkSpace::connect, &ws), "connect");
    
    // Function to change the name of the workspace
    chai.add(fun(&WorkSpace::set_name, &ws), "workspace");
    
    // Functions to configure the GC/simulation
    chai.add(fun<void (int)>([&ws](int t) { ws.settings.ack_timeout = t; }), "ack_timeout");
    chai.add(fun<void (OBNsim::simtime_t)>([&ws](OBNsim::simtime_t t) { ws.settings.final_time = t; }), "final_time");
}


bool SMNChai::Node::port_exists(const std::string &t_name) const {
    return (m_inputs.count(t_name) > 0) || (m_outputs.count(t_name) > 0) || (m_dataports.count(t_name) > 0);
}


OBNsmn::YARP::OBNNodeYARP* SMNChai::Node::create_yarp_node(OBNsmn::YARP::YARPPort *sys_port) const {
    assert(sys_port);
    
    OBNsmn::YARP::OBNNodeYARP *p_node = new OBNsmn::YARP::OBNNodeYARP(m_name, m_updates.size(), std::unique_ptr<OBNsmn::YARP::YARPPort>(sys_port));
    
    p_node->needUPDATEX = m_updateX;
    
    // Configure all update types in this node
    for (auto myupdate = m_updates.begin(); myupdate != m_updates.end(); ++myupdate) {
        p_node->setUpdateType(myupdate->first, myupdate->second);
    }
    
    return p_node;
}


void SMNChai::Node::add_input(const std::string &t_name) {
    if (!OBNsim::Utils::isValidIdentifier(t_name)) {
        throw smnchai_exception("Input port name '" + t_name + "' in node '" + m_name + "' is invalid.");
    }
    
    // Check if port already exists
    if (port_exists(t_name)) {
        throw smnchai_exception("Input port name '" + t_name + "' already exists in node '" + m_name + "'.");
    }
    
    // It's ok to add the port now
    m_inputs.emplace(t_name, 0);
}

void SMNChai::Node::add_output(const std::string &t_name) {
    if (!OBNsim::Utils::isValidIdentifier(t_name)) {
        throw smnchai_exception("Output port name '" + t_name + "' in node '" + m_name + "' is invalid.");
    }
    
    // Check if port already exists
    if (port_exists(t_name)) {
        throw smnchai_exception("Output port name '" + t_name + "' already exists in node '" + m_name + "'.");
    }
    
    // It's ok to add the port now
    m_outputs.emplace(t_name, 0);
}


void SMNChai::Node::add_dataport(const std::string &t_name) {
    if (!OBNsim::Utils::isValidIdentifier(t_name)) {
        throw smnchai_exception("Data port name '" + t_name + "' in node '" + m_name + "' is invalid.");
    }
    
    // Check if port already exists
    if (port_exists(t_name)) {
        throw smnchai_exception("Data port name '" + t_name + "' already exists in node '" + m_name + "'.");
    }
    
    // It's ok to add the port now
    m_dataports.emplace(t_name);
}


void SMNChai::Node::add_update(unsigned int t_id, OBNsim::simtime_t t_period) {
    if (t_id > OBNsim::MAX_UPDATE_INDEX) {
        throw smnchai_exception("Update ID=" + std::to_string(t_id) + " on node '" + m_name + "' is out of range.");
    }
    
    if (m_updates.count(t_id) > 0) {
        throw smnchai_exception("Update ID=" + std::to_string(t_id) + " on node '" + m_name + "' already exists.");
    }
    
    // Now it's ok to add the update
    m_updates.emplace(t_id, t_period);
}


void SMNChai::Node::input_to_update(unsigned int t_id, const std::string &t_port, bool t_direct) {
    // Check that ID is valid and already exists
    if (t_id > OBNsim::MAX_UPDATE_INDEX || m_updates.count(t_id) == 0) {
        throw smnchai_exception("In input_to_update, update ID=" + std::to_string(t_id) + " on node '" + m_name + "' is invalid or not existing.");
    }
    
    // Check that port is valid and exists
    if (!OBNsim::Utils::isValidIdentifier(t_port)) {
        throw smnchai_exception("In input_to_update, port name '" + t_port + "' on node '" + m_name + "' is invalid.");
    }
    
    auto it = m_inputs.find(t_port);
    if (it == m_inputs.end()) {
        throw smnchai_exception("In input_to_update, port name '" + t_port + "' on node '" + m_name + "' does not exist.");
    }
    
    // If this input has direct feedthrough, set the according bit
    if (t_direct) {
        it->second |= (1 << t_id);
    }
}


void SMNChai::Node::output_from_update(unsigned int t_id, const std::string &t_port) {
    // Check that ID is valid and already exists
    if (t_id > OBNsim::MAX_UPDATE_INDEX || m_updates.count(t_id) == 0) {
        throw smnchai_exception("In output_from_update, update ID=" + std::to_string(t_id) + " on node '" + m_name + "' is invalid or not existing.");
    }
    
    // Check that port is valid and exists
    if (!OBNsim::Utils::isValidIdentifier(t_port)) {
        throw smnchai_exception("In output_from_update, port name '" + t_port + "' on node '" + m_name + "' is invalid.");
    }
    
    auto it = m_outputs.find(t_port);
    if (it == m_outputs.end()) {
        throw smnchai_exception("In output_from_update, port name '" + t_port + "' on node '" + m_name + "' does not exist.");
    }
    
    it->second |= (1 << t_id);
}


SMNChai::PortInfo SMNChai::Node::port(const std::string &t_port) const {
    if (!OBNsim::Utils::isValidIdentifier(t_port)) {
        throw smnchai_exception("Port name '" + t_port + "' in node '" + m_name + "' is invalid.");
    }
    
    // Find the port in the lists of ports
    {
        auto it = m_inputs.find(t_port);
        if (it != m_inputs.end()) {
            return SMNChai::PortInfo(m_name, t_port, SMNChai::PortInfo::INPUT);
        }
    }
    
    {
        auto it = m_outputs.find(t_port);
        if (it != m_outputs.end()) {
            return SMNChai::PortInfo(m_name, t_port, SMNChai::PortInfo::OUTPUT);
        }
    }
    
    {
        auto it = m_dataports.find(t_port);
        if (it != m_dataports.end()) {
            return SMNChai::PortInfo(m_name, t_port, SMNChai::PortInfo::DATA);
        }
    }
    
    // Not found -> exception
    throw smnchai_exception("Port name '" + t_port + "' not found in node '" + m_name + "'.");
}


void SMNChai::WorkSpace::add_node(SMNChai::Node *p_node) {
    assert(p_node);
    
    auto nodeName = p_node->get_name();
    if (m_nodes.count(nodeName) != 0) {
        // Node already exists
        throw smnchai_exception("Node '" + nodeName + "' already exists in workspace '" + m_name + "'.");
    }
    
    // Add the node to the list of nodes
    m_nodes.emplace(nodeName, std::make_pair(p_node, 0));
}

void SMNChai::WorkSpace::connect(SMNChai::PortInfo &&t_from, SMNChai::PortInfo &&t_to) {
    // Check that these ports can be connected
    if (t_from.port_type == SMNChai::PortInfo::INPUT) {
        throw smnchai_exception("Port " + t_from.node_name + '/' + t_from.port_name + " is an input and can't be the source of a connection.");
    }
    
    if (t_to.port_type == SMNChai::PortInfo::OUTPUT) {
        throw smnchai_exception("Port " + t_to.node_name + '/' + t_to.port_name + " is an output and can't be the target of a connection.");
    }
    
    // Check if this connection has already existed
    bool bNotExist = std::none_of(m_connections.begin(), m_connections.end(),
                                  [t_from,t_to](const std::pair<PortInfo, PortInfo>& p) {
                                      return (p.first.node_name == t_from.node_name) && (p.first.port_name == t_from.port_name) &&
                                      (p.second.node_name == t_to.node_name) && (p.second.port_name == t_to.port_name);
                                  });
    if (bNotExist) {
        m_connections.emplace_front(t_from, t_to);
    }
}

void SMNChai::WorkSpace::connect(const std::string &from_node, const std::string &from_port, const std::string &to_node, const std::string &to_port) {
    // Look up the source and target nodes
    auto itsrc = m_nodes.find(from_node);
    if (itsrc == m_nodes.end()) {
        throw smnchai_exception("Source node '" + from_node + "' of a connection does not exist.");
    }
    
    auto ittgt = m_nodes.find(to_node);
    if (ittgt == m_nodes.end()) {
        throw smnchai_exception("Target node '" + to_node + "' of a connection does not exist.");
    }
    
    // Connect them
    connect(itsrc->second.first->port(from_port), ittgt->second.first->port(to_port));
}


void SMNChai::WorkSpace::print() const {
    std::cout << "Workspace '" << m_name << "'" << std::endl;
    std::cout << "List of nodes:" << std::endl;
    for (auto n: m_nodes) {
        std::cout << n.second.first->get_name() << ' ';
    }
    std::cout << "\nList of connections:" << std::endl;
    for (auto c: m_connections) {
        std::cout << c.first.node_name << '/' << c.first.port_name << " -> " << c.second.node_name << '/' << c.second.port_name << std::endl;
    }
}


std::string SMNChai::WorkSpace::get_full_path(const std::string &t_obj1, const std::string &t_obj2) const {
    assert(!t_obj1.empty());
    return (m_name.empty()?"/":('/'+m_name+'/')) + t_obj1 + (t_obj2.empty()?"":('/'+t_obj2));
}


std::string SMNChai::WorkSpace::get_full_path(const SMNChai::PortInfo &t_port) const {
    return get_full_path(t_port.node_name, t_port.port_name);
}


void SMNChai::WorkSpace::generate_obn_system(OBNsmn::GCThread &gc) {
    // Sanity check
    if (m_nodes.size() == 0) {
        // Error if there is no node
        throw smnchai_exception("There is no node in workspace '" + m_name + "' to generate its OBN system.");
    }
    
    // Create GC ports and node objects, add them to the GC object and save their IDs
    for (auto mynode = m_nodes.begin(); mynode != m_nodes.end(); ++mynode) {
        // Create a GC port for it and try to open it
        OBNsmn::YARP::YARPPort *p_port = new OBNsmn::YARP::YARPPort;
        std::string sys_port_name = get_full_path("_smn_", mynode->first);
        if (!p_port->open(sys_port_name)) {
            delete p_port;
            throw smnchai_exception("Could not open system port " + sys_port_name);
        }
        
        // Create the node and associate the port with it
        // Note that the node object will own the port object, while the GC object will own the node object.
        auto *p_node = mynode->second.first->create_yarp_node(p_port);
        auto result = gc.insertNode(p_node);
        if (result.first) {
            // Record the ID of this node in GC
            mynode->second.second = result.second;
        } else {
            // Delete the node object
            delete p_node;
            throw smnchai_exception("Could not insert node '" + mynode->first + "' into the system.");
        }
        
        // Connect the SMN and the ports (via their system ports)
        // Remote ports must already exist, i.e. remote nodes have already started and are waiting
        std::string remote_gc_port = get_full_path(mynode->first, "_gc_");
        
        // Check that the remote port exists
        if (!yarp::os::Network::exists(remote_gc_port)) {
            // Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
            throw smnchai_exception("System port on remote node " + mynode->first + " is unavailable.");
        }
        
        // The GC has a dedicated output port for each node
        if (!p_port->addOutput(remote_gc_port)) {
            // Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
            throw smnchai_exception("Could not connect to remote node " + mynode->first);
        }
        
        // However, all nodes send to the same GC input port
        if (!yarp::os::Network::connect(remote_gc_port, get_full_path("_smn_", "_gc_"))) {
            // Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
            throw smnchai_exception("Could not connect from remote node " + mynode->first + " to the SMN.");
        }
    }
    
    
    
    // Now connect the ports and create the dependency graph.
    // ASSUME that all ports have already been created, i.e. nodes are already started.
    OBNsmn::NodeDepGraph* nodeGraph = new OBNsmn::NodeDepGraph_BGL(m_nodes.size());
    
    for (auto myconn = m_connections.begin(); myconn != m_connections.end(); ++myconn) {
        std::string from_port = get_full_path(myconn->first), to_port = get_full_path(myconn->second);
        
        if (!yarp::os::Network::connect(from_port, to_port)) {
            // Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
            throw smnchai_exception("Could not connect " + from_port + " to " + to_port);
        }
        
        // Add a dependency link for this connection iff:
        // (a) source is an output port and target is an input port; and
        // (b) the update masks for both of them are non-zero
        auto src_node = m_nodes.at(myconn->first.node_name), tgt_node = m_nodes.at(myconn->second.node_name);
        OBNsim::updatemask_t src_mask = src_node.first->output_updatemask(myconn->first.port_name);
        OBNsim::updatemask_t tgt_mask = tgt_node.first->input_updatemask(myconn->second.port_name);
        
        if (myconn->first.port_type == PortInfo::OUTPUT && myconn->second.port_type == PortInfo::INPUT &&
            src_mask != 0 && tgt_mask != 0)
        {
            //std::cout << "Dependency from " << src_node.second << " with mask " << src_mask << " to " << tgt_node.second << " with mask " << tgt_mask << std::endl;
            nodeGraph->addDependency(src_node.second, tgt_node.second, src_mask, tgt_mask);
        }
    }

    gc.setDependencyGraph(nodeGraph);   // Set the dependency graph for the GC
    
    // Copy the settings to GC
    gc.ack_timeout = settings.ack_timeout;
    if (!gc.setFinalSimulationTime(settings.final_time)) {
        throw smnchai_exception("Error while setting final simulation time.");
    }
}

