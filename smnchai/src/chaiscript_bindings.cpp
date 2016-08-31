#include "smnchai_api.h"
#include "smnchai_utils.h"
#include <chaiscript/dispatchkit/bootstrap.hpp>

using namespace chaiscript;
using namespace SMNChai;

/** This function register all the functions, types, etc. provided by the SMNChai API to a given ChaiScript object.
 \param chai The ChaiScript object to which the API is registered.
 \param ws The WorkSpace object to which nodes and connections are added.
 */
void SMNChai::registerSMNAPI(ChaiScript &chai, SMNChai::WorkSpace &ws) {
    ModulePtr module = std::make_shared<Module>();  // A module to add stuff to chai engine (because Chaiscript's API has changed)
    
    // *********************************************
    // Register the Node class and its constructor
    // *********************************************
    
    chai.add(user_type<Node>(), "Node");
    chai.add(constructor<Node (const std::string&)>(), "new_node");
    bootstrap::copy_constructor<Node>("Node", *module);
    
    
    // *********************************************
    // Methods to manipulate a node object: ports, updates, etc.
    // *********************************************
    
    chai.add(fun(&Node::get_name), "get_name");
    
    chai.add(fun(&Node::add_input), "add_input");
    chai.add(fun([](Node& node, const std::string &name){ return node.add_input(name); }), "add_input");
    
    chai.add(fun(&Node::add_output), "add_output");
    chai.add(fun([](Node& node, const std::string &name){ return node.add_output(name); }), "add_output");
    
    chai.add(fun(&Node::add_dataport), "add_dataport");
    chai.add(fun([](Node& node, const std::string &name){ return node.add_dataport(name); }), "add_dataport");
    
    chai.add(fun(&Node::add_update), "add_block");
    chai.add(fun(&Node::set_need_updateX), "need_updateX");
    chai.add(fun(&Node::set_comm_protocol), "set_comm");   // a string of the name of the communication protocol: default, mqtt, yarp
    chai.add(fun(&Node::get_comm_protocol), "get_comm");   // returns a string of the name of the communication protocol: default, mqtt, yarp
    
    chai.add(fun(&Node::input_to_update), "input_to_block");
    chai.add(fun(&Node::output_from_update), "output_from_block");
    chai.add(fun(&Node::add_internal_dependency), "add_internal_dependency");

    // PortInfo from a node: used for connecting ports, and methods to access it
    chai.add(user_type<SMNChai::PortInfo>(), "PortInfo");
    bootstrap::copy_constructor<SMNChai::PortInfo>("PortInfo", *module);
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
    bootstrap::copy_constructor<SubSystem>("SubSystem", *module);
    
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
    
    chai.add(fun<bool (WorkSpace::*) (const Node &)>(&WorkSpace::is_node_online, &ws), "is_node_online");
    //chai.add(fun<bool (WorkSpace::*) (const std::string &) const>(&WorkSpace::is_node_online, &ws), "is_node_online");
    
    chai.add(fun([&ws](const SMNChai::Node &t_node, const std::string &t_computer, const std::string &t_prog, const std::string &t_args, const std::string &t_tag){ws.start_remote_node(t_node,t_computer,t_prog,t_args,t_tag);}), "start_remote_node");
    
    chai.add(fun([&ws](const std::string &t_node, const std::string &t_computer, const std::string &t_prog, const std::string &t_args, const std::string &t_tag){ws.start_remote_node(t_node,t_computer,t_prog,t_args,t_tag);}), "start_remote_node");
    
    chai.add(fun<void (WorkSpace::*) (const SMNChai::Node &, const std::string &, const std::string &, const std::string &, const std::string &, const std::string &)>(&WorkSpace::start_remote_node, &ws), "start_remote_node");
    chai.add(fun<void (WorkSpace::*) (const std::string &, const std::string &, const std::string &, const std::string &, const std::string &, const std::string &)>(&WorkSpace::start_remote_node, &ws), "start_remote_node");
    
    chai.add(fun([&ws](const SMNChai::Node &n, const std::string &c, const std::string &p, const std::string &a) {
        ws.start_remote_node(n, c, p, a);
    }), "start_remote_node");
    chai.add(fun([&ws](const std::string &n, const std::string &c, const std::string &p, const std::string &a) {
        ws.start_remote_node(n, c, p, a);
    }), "start_remote_node");
    
    chai.add(fun(&run_remote_command), "run_remote_command");
    
    chai.add(fun<void (WorkSpace::*)(const Node &, double)>(&WorkSpace::waitfor_node_online, &ws), "waitfor_node");
    //chai.add(fun<void (WorkSpace::*)(const std::string &, double) const>(&WorkSpace::waitfor_node_online, &ws), "waitfor_node");
    
    chai.add(fun(&WorkSpace::waitfor_all_nodes_online, &ws), "waitfor_all_nodes");
    
    // *********************************************
    // Functions to generate node list for Docker
    // *********************************************
    if (ws.m_settings.m_dockerlist) {
        chai.add(fun(&WorkSpace::obndocker_node, &ws), "obndocker_node");
        chai.add(fun(
                     [&ws](const SMNChai::Node& a1, const std::string& a2, const std::string& a3, const std::string& a4, const std::string& a5){
                         ws.obndocker_node(a1, a2, a3, a4, a5);
                     }), "obndocker_node");  // with default "extra" argument = ""
    } else {
        // empty function
        chai.add(fun([](const SMNChai::Node&, const std::string&, const std::string&, const std::string&, const std::string&){}), "obndocker_node");
        chai.add(fun([](const SMNChai::Node&, const std::string&, const std::string&, const std::string&, const std::string&, const std::string&){}), "obndocker_node");
    }
    
    // *********************************************
    // Functions to configure the GC/simulation
    // *********************************************
    ws.register_settings_with_Chaiscript(chai);

    
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
    
    
    // Add the module which contains bootstrap's stuff
    chai.add(module);
    
    // Add utility API
    chaiscript::ModulePtr util_module(SMNChai::APIUtils::smnchai_api_utils_io());
    util_module = SMNChai::APIUtils::smnchai_api_utils_math(util_module);
    util_module = SMNChai::APIUtils::smnchai_api_utils_fixes(util_module);
    util_module = SMNChai::APIUtils::smnchai_api_utils_misc(util_module);
    chai.add(util_module);
}

void SMNChai::WorkSpace::register_settings_with_Chaiscript(ChaiScript &chai) {
    // Register the Settings class and the settings object
    chai.add(user_type<SMNChai::WorkSpace::Settings>(), "WorkspaceSettings");
    chai.add_global(var(&m_settings), "settings");
    
    /* Set/get timeout for ACK, in milliseconds. */
    chai.add(fun(static_cast<void (SMNChai::WorkSpace::Settings::*)(int)>(&SMNChai::WorkSpace::Settings::ack_timeout)), "ack_timeout");
    chai.add(fun(static_cast<int (SMNChai::WorkSpace::Settings::*)() const>(&SMNChai::WorkSpace::Settings::ack_timeout)), "ack_timeout");
    
    /* Set/get final simulation time. */
    chai.add(fun(static_cast<void (SMNChai::WorkSpace::Settings::*)(double)>(&SMNChai::WorkSpace::Settings::final_time)), "final_time");
    chai.add(fun(static_cast<double (SMNChai::WorkSpace::Settings::*)() const>(&SMNChai::WorkSpace::Settings::final_time)), "final_time");

    /* Atomic time unit, in microseconds. */
    chai.add(fun(static_cast<void (SMNChai::WorkSpace::Settings::*)(unsigned int)>(&SMNChai::WorkSpace::Settings::time_unit)), "time_unit");
    chai.add(fun(static_cast<unsigned int (SMNChai::WorkSpace::Settings::*)() const>(&SMNChai::WorkSpace::Settings::time_unit)), "time_unit");
    

    /* Set the initial wallclock time by a string "YYYY-MM-DD HH:MM:SS". */
    chai.add(fun(&SMNChai::WorkSpace::Settings::wallclock), "wallclock");
    
    /** Choose not to run the simulation.
     By default, the simulation will automatically run; however if we set this to false, the simulation will not run.
     This can be useful to just load, print, and check the system configuration without actually running it.
     Note that certain commands are affected by where this command is placed, e.g. commands to start remote nodes will still run if at the time it is called the setting is still "true", however if this setting was switched off before a command to start a remote node, the latter will be disabled.  Similarly for commands to wait for nodes to come online.
     */
    chai.add(fun([this](bool b) { this->m_settings.m_run_simulation = b; }), "run_simulation");
    
    /* Get and Set the default communication. */
    chai.add(fun(static_cast<std::string (SMNChai::WorkSpace::Settings::*) () const>(&SMNChai::WorkSpace::Settings::default_comm)), "default_comm");
    chai.add(fun(static_cast<void (SMNChai::WorkSpace::Settings::*) (const std::string&)>(&SMNChai::WorkSpace::Settings::default_comm)), "default_comm");
    
    /* Set/get MQTT server. */
    chai.add(fun(static_cast<void (SMNChai::WorkSpace::Settings::*)(const std::string&)>(&SMNChai::WorkSpace::Settings::MQTT_server)), "MQTT_server");
    chai.add(fun(static_cast<std::string (SMNChai::WorkSpace::Settings::*)() const>(&SMNChai::WorkSpace::Settings::MQTT_server)), "MQTT_server");
}
