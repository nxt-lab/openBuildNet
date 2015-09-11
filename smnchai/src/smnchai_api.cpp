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
#include <cmath>
#include <chrono>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <obnsmn_report.h>
#include <smnchai_api.h>
#include <chaiscript/dispatchkit/bootstrap.hpp>

#include <smnchai_utils.h>      // Utility API for SMNChai

#include <yarp/os/Run.h>        // YarpRun support

using namespace chaiscript;
using namespace SMNChai;

const char *NODE_GC_PORT_NAME = "_gc_";
std::string Node::m_global_prefix = "";

/** This function register all the functions, types, etc. provided by the SMNChai API to a given ChaiScript object.
 \param chai The ChaiScript object to which the API is registered.
 \param ws The WorkSpace object to which nodes and connections are added.
 */
void SMNChai::registerSMNAPI(ChaiScript &chai, WorkSpace &ws) {
    // *********************************************
    // Register the Node class and its constructor
    // *********************************************
    
    chai.add(user_type<Node>(), "Node");
    chai.add(constructor<Node (const std::string&)>(), "new_node");
    chai.add(bootstrap::copy_constructor<Node>("Node"));
    
    
    // *********************************************
    // Methods to manipulate a node object: ports, updates, etc.
    // *********************************************
    
    chai.add(fun(&Node::get_name), "get_name");
    
    chai.add(fun(&Node::add_input), "add_input");
    chai.add(fun(&Node::add_output), "add_output");
    chai.add(fun(&Node::add_dataport), "add_dataport");
    chai.add(fun(&Node::add_update), "add_update");
    chai.add(fun(&Node::set_need_updateX), "need_updateX");
    
    chai.add(fun(&Node::input_to_update), "input_to_update");
    chai.add(fun(&Node::output_from_update), "output_from_update");

    // PortInfo from a node: used for connecting ports, and methods to access it
    chai.add(user_type<SMNChai::PortInfo>(), "PortInfo");
    chai.add(bootstrap::copy_constructor<SMNChai::PortInfo>("PortInfo"));
    chai.add(fun(&Node::port), "port");
    // Returns the type of a PortInfo: 0 = input, 1 = output, 2 = data
    //chai.add(fun<int (const SMNChai::PortInfo&)>([](const SMNChai::PortInfo& p) { return p.port_type; }), "port_type");
    chai.add(fun([](const SMNChai::PortInfo& p) { return static_cast<int>(p.port_type); }), "port_type"); // need to cast from enum value to int for automatic deduction of type signature
    
    
    // *********************************************
    // Methods to work with subsystems
    // *********************************************
    
    chai.add(user_type<SubSystem>(), "SubSystem");
    chai.add(constructor<SubSystem (const std::string&)>(), "new_subsystem");
    chai.add(constructor<SubSystem (const SubSystem&, const std::string&)>(), "new_subsystem");
    chai.add(bootstrap::copy_constructor<SubSystem>("SubSystem"));
    
    chai.add(fun(&SubSystem::new_node), "new_node");
    chai.add(fun(&SubSystem::new_subsystem), "new_subsystem");
    
    //chai.add(fun<void (const SubSystem &)>([](const SubSystem &sub){
    chai.add(fun([](const SubSystem &sub){
        Node::m_global_prefix = sub.get_name() + '/';
    }), "set_current_subsystem");
    
    //chai.add(fun<void ()>([](){ Node::m_global_prefix.clear(); }), "clear_current_subsystem");
    chai.add(fun([](){ Node::m_global_prefix.clear(); }), "clear_current_subsystem");
    
    chai.add(fun(&SubSystem::current_subsystem), "get_current_subsystem");
             
    // *********************************************
    // Methods to work with the WorkSpace object: add nodes, connect ports...
    // These are bound with the given WorkSpace object
    // *********************************************
    
    // Add a node object to the workspace:
    // The object is COPIED to the workspace, so any later change to the Chaiscript's node object won't reflect in the workspace.
    chai.add(fun(&WorkSpace::add_node, &ws), "add_node");
    
    //chai.add(fun<void (PortInfo, PortInfo)>([&ws](PortInfo s, PortInfo t) { ws.connect(std::move(s), std::move(t)); }), "connect");
    chai.add(fun([&ws](PortInfo s, PortInfo t) { ws.connect(std::move(s), std::move(t)); }), "connect");
    chai.add(fun<void (WorkSpace::*)(const std::string &, const std::string &, const std::string &, const std::string &)>(&WorkSpace::connect, &ws), "connect");
    
    // Function to change the name of the workspace: workspace(new_name)
    chai.add(fun(&WorkSpace::set_name, &ws), "workspace");
    
    // Function to get the current workspace's name: workspace() without argument
    chai.add(fun(&WorkSpace::get_name, &ws), "workspace");
    
    // Function to print the workspace to screen, useful for checking it
    chai.add(fun(&WorkSpace::print, &ws), "print_system");
    
    
    // *********************************************
    // Functions to start remote nodes, wait for nodes to go online, etc.
    // *********************************************
    
    chai.add(fun<bool (WorkSpace::*) (const Node &) const>(&WorkSpace::is_node_online, &ws), "is_node_online");
    chai.add(fun<bool (WorkSpace::*) (const std::string &) const>(&WorkSpace::is_node_online, &ws), "is_node_online");

    chai.add(fun<void (WorkSpace::*) (const SMNChai::Node &, const std::string &, const std::string &, const std::string &, const std::string &) const>(&WorkSpace::start_remote_node, &ws), "start_remote_node");
    chai.add(fun<void (WorkSpace::*) (const std::string &, const std::string &, const std::string &, const std::string &, const std::string &) const>(&WorkSpace::start_remote_node, &ws), "start_remote_node");
    
    chai.add(fun([&ws](const SMNChai::Node &n, const std::string &c, const std::string &p, const std::string &a) {
        ws.start_remote_node(n, c, p, a);
    }), "start_remote_node");
    chai.add(fun([&ws](const std::string &n, const std::string &c, const std::string &p, const std::string &a) {
        ws.start_remote_node(n, c, p, a);
    }), "start_remote_node");
    
    chai.add(fun(&run_remote_command), "run_remote_command");
    
    chai.add(fun<void (WorkSpace::*)(const Node &, double) const>(&WorkSpace::waitfor_node_online, &ws), "waitfor_node");
    chai.add(fun<void (WorkSpace::*)(const std::string &, double) const>(&WorkSpace::waitfor_node_online, &ws), "waitfor_node");
    
    chai.add(fun(&WorkSpace::waitfor_all_nodes_online, &ws), "waitfor_all_nodes");
    
    
    // *********************************************
    // Functions to configure the GC/simulation
    // *********************************************
    
    /** Set timeout for ACK, in milliseconds. */
    //chai.add(fun<void (int)>([&ws](int t) { ws.settings.ack_timeout = t; }), "ack_timeout");
    chai.add(fun([&ws](int t) { ws.settings.ack_timeout = t; }), "ack_timeout");
    
    /** Set final simulation time, in microseconds. */
    //chai.add(fun<void (double)>([&ws](double t) {
    chai.add(fun([&ws](double t) {
        if (t <= 0.0) { throw smnchai_exception("Final simulation time must be positive, but " + std::to_string(t) + " is given."); }
        ws.settings.final_time = t;
    }), "final_time");
    
    /** Set the atomic time unit, in microseconds. */
    // chai.add(fun<void (unsigned int)>([&ws](unsigned int t) {
    chai.add(fun([&ws](unsigned int t) {
        if (t < 1) { throw smnchai_exception("Time unit must be positive, but " + std::to_string(t) + " is given."); }
        ws.settings.time_unit = t;
    }), "time_unit");
    
    /** Set the initial wallclock time by a string "YYYY-MM-DD HH:MM:SS". */
    chai.add(fun([&ws](const std::string &t) {
        std::tm tm = {0};
        std::stringstream ss(t);
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
        tm.tm_isdst = -1;
        ws.settings.wallclock = std::mktime(&tm);
    }), "wallclock");
    
    /** Choose not to run the simulation.
     By default, the simulation will automatically run; however if we set this to false, the simulation will not run.
     This can be useful to just load, print, and check the system configuration without actually running it.
     Note that certain commands are affected by where this command is placed, e.g. commands to start remote nodes will still run if at the time it is called the setting is still "true", however if this setting was switched off before a command to start a remote node, the latter will be disabled.  Similarly for commands to wait for nodes to come online.
     */
    //chai.add(fun<void (bool)>([&ws](bool b) { ws.settings.run_simulation = b; }), "run_simulation");
    chai.add(fun([&ws](bool b) { ws.settings.run_simulation = b; }), "run_simulation");
    
    /** Defines constants for typical time values (in microseconds). */
    chai.add(const_var(double(1.0)), "microsecond");
    chai.add(const_var(double(1000.0)), "millisecond");
    chai.add(const_var(double(1e6)), "second");
    chai.add(const_var(double(60e6)), "minute");
    chai.add(const_var(double(3.6e9)), "hour");
    chai.add(const_var(double(24*3.6e9)), "day");
    
    
    // *********************************************
    // Functions to export a node or network
    // *********************************************
    
    /** Export full description of a node to DOT. 
     \param n A node object
     \param fn A file name
     */
    chai.add(fun([](const Node &n, const std::string &fn) {
        std::stringstream ss(std::ios_base::out);
        n.export2dot_full(ss);
        std::ofstream fs(fn);
        if (fs.is_open()) {
            fs << ss.str();
            fs.close();
        } else {
            throw smnchai_exception("Could not open file " + fn + " to write.");
        }
    }), "export2dot");
    
    /** Export the system to a DOT file.
     \param fn File name
     \param cluster true if each node is a cluster; false if each node is a simple node.
     \param props Additional properties of the DOT graph.
     */
    chai.add(fun([&ws](const std::string &fn, bool cluster, const std::string &props) { ws.export2dotfile(fn, cluster, props); }), "export2dot");
    chai.add(fun([&ws](const std::string &fn, bool cluster) { ws.export2dotfile(fn, cluster); }), "export2dot");
    chai.add(fun([&ws](const std::string &fn) { ws.export2dotfile(fn, false); }), "export2dot");    ///< Default version with cluster = false
    
    /** Export the system to a GraphML file.
     \param fn File name
     */
    chai.add(fun([&ws](const std::string &fn) { ws.export2graphmlfile(fn); }), "export2graphml");
    
    
    // Add utility API
    chaiscript::ModulePtr util_module(SMNChai::APIUtils::smnchai_api_utils_io());
    util_module = SMNChai::APIUtils::smnchai_api_utils_math(util_module);
    util_module = SMNChai::APIUtils::smnchai_api_utils_fixes(util_module);
    util_module = SMNChai::APIUtils::smnchai_api_utils_misc(util_module);
    chai.add(util_module);
}


void SMNChai::run_remote_command(const std::string &t_node, const std::string &t_tag, const std::string &t_prog, const std::string &t_args) {
    // Run a command on a remote node/computer using YarpRun
    if (t_node.empty() || t_tag.empty() || t_prog.empty()) {
        throw smnchai_exception("run_remote_command error: node/computer name and tag and command must be non-empty.");
    }
    
    // Build the Property list for the command to run
    // Current implementation of Yarprun has a bug that doesn't take the arguments into account, so we work around this by concatenating the command and the arguments.
    yarp::os::Property prop;
    prop.put("name", t_prog + ' ' + t_args);
    //if (!t_args.empty()) {
    //    prop.put("parameters", t_args);
    //}
    
    auto tagName = yarp::os::ConstString(t_tag);
    if (yarp::os::Run::start(t_node, prop, tagName) != 0) {
        // Failed
        throw smnchai_exception("start_remote_node error: could not run the remote command.");
    }
}


void SMNChai::WorkSpace::start_remote_node(const std::string &t_node, const std::string &t_computer, const std::string &t_prog, const std::string &t_args, const std::string &t_tag) const {
    // Only start if node is not online
    if (settings.run_simulation && !is_node_online(t_node)) {
        if (t_tag.empty()) {
            SMNChai::run_remote_command(t_computer, t_node, t_prog, t_args);
        } else {
            SMNChai::run_remote_command(t_computer, t_tag, t_prog, t_args);
        }
    }
}

void SMNChai::WorkSpace::start_remote_node(const SMNChai::Node &t_node, const std::string &t_computer, const std::string &t_prog, const std::string &t_args, const std::string &t_tag) const {
    start_remote_node(t_node.get_name(), t_computer, t_prog, t_args, t_tag);
}

bool SMNChai::WorkSpace::is_node_online(const SMNChai::Node &t_node) const {
    return yarp::os::Network::exists(get_full_path(t_node.get_name(), NODE_GC_PORT_NAME));
}

bool SMNChai::WorkSpace::is_node_online(const std::string &t_node) const {
    return yarp::os::Network::exists(get_full_path(t_node, NODE_GC_PORT_NAME));
}

void SMNChai::WorkSpace::waitfor_node_online(const std::string &t_node, double timeout) const {
    if (!settings.run_simulation) {
        // If not going to run simulation then we should not wait
        return;
    }
    if (timeout <= 0.0) {
        while (!is_node_online(t_node)) {
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
        }
    } else {
        std::chrono::time_point<std::chrono::steady_clock> start;
        std::chrono::duration<double> dur;
        
        start = std::chrono::steady_clock::now();
        
        while (!is_node_online(t_node)) {
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
            
            dur = std::chrono::steady_clock::now() - start;
            if (dur.count() > timeout) {
                // Timeout occurs
                throw smnchai_exception("Waiting for node '" + t_node + "' to go online but timeout occurred.");
            }
        }
    }
}

void SMNChai::WorkSpace::waitfor_node_online(const SMNChai::Node &t_node, double timeout) const {
    waitfor_node_online(t_node.get_name(), timeout);
}

void SMNChai::WorkSpace::waitfor_all_nodes_online(double timeout) const {
    if (!settings.run_simulation) {
        // If not going to run simulation then we should not wait
        return;
    }

    if (timeout <= 0.0) {
        while (!are_all_nodes_online()) {
            std::this_thread::sleep_for (std::chrono::milliseconds(500));
        }
    } else {
        std::chrono::time_point<std::chrono::steady_clock> start;
        std::chrono::duration<double> dur;
        
        start = std::chrono::steady_clock::now();
        
        while (!are_all_nodes_online()) {
            std::this_thread::sleep_for (std::chrono::milliseconds(500));
            
            dur = std::chrono::steady_clock::now() - start;
            if (dur.count() > timeout) {
                // Timeout occurs
                throw smnchai_exception("Waiting for all nodes to go online but timeout occurred.");
            }
        }
    }
}

bool SMNChai::WorkSpace::are_all_nodes_online() const {
    for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it) {
        if (!yarp::os::Network::exists(get_full_path(it->second.first.get_name(), NODE_GC_PORT_NAME))) {
            return false;
        }
    }
    return true;
}

bool SMNChai::Node::port_exists(const std::string &t_name) const {
    return (m_inputs.count(t_name) > 0) || (m_outputs.count(t_name) > 0) || (m_dataports.count(t_name) > 0);
}


OBNsmn::YARP::OBNNodeYARP* SMNChai::Node::create_yarp_node(OBNsmn::YARP::YARPPort *sys_port, const WorkSpace &ws) const {
    assert(sys_port);
    
    OBNsmn::YARP::OBNNodeYARP *p_node = new OBNsmn::YARP::OBNNodeYARP(m_name, m_updates.size(), std::unique_ptr<OBNsmn::YARP::YARPPort>(sys_port));
    
    p_node->needUPDATEX = m_updateX;
    
    // Configure all update types in this node
    // Note that time values are stored as real numbers of microseconds, which must be converted to integer values in time unit
    for (auto myupdate = m_updates.begin(); myupdate != m_updates.end(); ++myupdate) {
        p_node->setUpdateType(myupdate->first, ws.get_time_value(myupdate->second));
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


void SMNChai::Node::add_update(unsigned int t_id, double t_period) {
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


void SMNChai::WorkSpace::add_node(const SMNChai::Node &p_node) {
    auto nodeName = p_node.get_name();
    if (m_nodes.count(nodeName) != 0) {
        // Node already exists
        throw smnchai_exception("Node '" + nodeName + "' already exists in workspace '" + m_name + "'.");
    }
    
    // Add the node to the list of nodes
    m_nodes.emplace(nodeName, std::make_pair(p_node, 0));
}

void SMNChai::WorkSpace::connect(const SMNChai::PortInfo &t_from, const SMNChai::PortInfo &t_to) {
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
    connect(itsrc->second.first.port(from_port), ittgt->second.first.port(to_port));
}


void SMNChai::WorkSpace::print() const {
    std::cout << "Workspace '" << m_name << "'" << std::endl;
    std::cout << "List of nodes:" << std::endl;
    for (auto n: m_nodes) {
        std::cout << n.second.first.get_name() << ' ';
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


OBNsim::simtime_t SMNChai::WorkSpace::get_time_value(double t) const {
    assert(settings.time_unit > 0);
    auto ti = std::llround(t / double(settings.time_unit));
    OBNsim::simtime_t result = (ti < 0)?0:ti;   // Convert to simtime_t, saturated below at 0
    
    // Check if a sampling time is nonzero but it's converted to 0
    if (t > 0.0 && result == 0) {
        OBNsmn::report_warning(0, "A non-zero sampling time is converted to 0; consider reducing the time unit.");
    }
    return result;
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
        auto *p_node = mynode->second.first.create_yarp_node(p_port, *this);
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
        std::string remote_gc_port = get_full_path(mynode->first, NODE_GC_PORT_NAME);
        
        // Check that the remote port exists
        if (!yarp::os::Network::exists(remote_gc_port)) {
            // Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
            throw smnchai_exception("System port on remote node " + mynode->first + " is unavailable.");
        }
        
        // The GC has a dedicated output port for each node
        if (!p_port->addOutput(remote_gc_port)) {
            // Try a second time
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            if (!p_port->addOutput(remote_gc_port)) {
                // Still Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
                throw smnchai_exception("Could not connect to remote node " + mynode->first);
            }
        }
        
        // However, all nodes send to the same GC input port
        if (!yarp::os::Network::connect(remote_gc_port, get_full_path("_smn_", NODE_GC_PORT_NAME), "", false)) {
            // Try a second time
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            if (!yarp::os::Network::connect(remote_gc_port, get_full_path("_smn_", NODE_GC_PORT_NAME), "", false)) {
                // Still Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
                throw smnchai_exception("Could not connect from remote node " + mynode->first + " to the SMN.");
            }
        }
    }
    
    
    
    // Now connect the ports and create the dependency graph.
    // ASSUME that all ports have already been created, i.e. nodes are already started.
    OBNsmn::NodeDepGraph* nodeGraph = new OBNsmn::NodeDepGraph_BGL(m_nodes.size());
    
    for (auto myconn = m_connections.begin(); myconn != m_connections.end(); ++myconn) {
        std::string from_port = get_full_path(myconn->first), to_port = get_full_path(myconn->second);
        
        if (!yarp::os::Network::connect(from_port, to_port, "", false)) {
            // Try a second time
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            if (!yarp::os::Network::connect(from_port, to_port, "", false)) {
                // Failed -> error; note that the GC is now managing all node objects, so do not delete node objects
                throw smnchai_exception("Could not connect " + from_port + " to " + to_port);
            }
        }
        
        // Add a dependency link for this connection iff:
        // (a) source is an output port and target is an input port; and
        // (b) the update masks for both of them are non-zero
        auto src_node = m_nodes.at(myconn->first.node_name), tgt_node = m_nodes.at(myconn->second.node_name);
        OBNsim::updatemask_t src_mask = src_node.first.output_updatemask(myconn->first.port_name);
        OBNsim::updatemask_t tgt_mask = tgt_node.first.input_updatemask(myconn->second.port_name);
        
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
    
    if (!gc.setSimulationTimeUnit(settings.time_unit)) {
        throw smnchai_exception("Error while setting simulation time unit.");
    }
    
    if (!gc.setInitialWallclock(settings.wallclock)) {
        throw smnchai_exception("Error while setting the initial wallclock time.");
    }
    
    // Note that time values are mostly real numbers in microseconds, so we need to convert them to integer numbers in the time unit.
    if (!gc.setFinalSimulationTime(get_time_value(settings.final_time))) {
        throw smnchai_exception("Error while setting final simulation time.");
    }
}

