/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Export a node or network from SMNChai to graphic files.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cassert>
#include <cstdio>
#include <cmath>

#include <obnsmn_report.h>
#include <smnchai_api.h>

using namespace SMNChai;

const char QUOTE = '"';
const char TAB = '\t';

// Returns the string representation of an UPDATE name (UPDATEnn)
std::string gen_update_name(unsigned int id) {
    assert(0 <= id && id <= OBNsim::MAX_UPDATE_INDEX);
    
    char buf[32];
    int cx = snprintf(buf, 32, "UPDATE%02u", id);
    
    assert(cx >= 0 && cx < 32);

    if (cx >= 0 && cx < 32) {
        return std::string(buf);
    }
    else {
        return std::string();
    }
}

// Returns the string representation of a time value in microseconds (for UPDATEs)
std::string gen_time_rep(double t_ms) {
    assert(t_ms >= 0);
    
    if (t_ms <= 0.0) {
        return "nonperiodic";
    }
    
    const double DAY_US = 24*3.6e9;
    const double HOUR_US = 3.6e9;
    const double MIN_US = 60e6;
    const double SEC_US = 1e6;
    const double MS_US = 1e3;
    
    std::string result;
    int intpart; // integral part
    
    // Days
    if (t_ms >= DAY_US) {
        intpart = int(floor(t_ms / DAY_US));
        t_ms -= intpart*DAY_US;
        result += (std::to_string(intpart) + 'd');
    }
    
    if (t_ms <= 0.0) {
        return result;
    }
    
    // Hours
    if (t_ms >= HOUR_US) {
        intpart = int(floor(t_ms / HOUR_US));
        t_ms -= intpart*HOUR_US;
        result += (std::to_string(intpart) + 'h');
    }
    
    if (t_ms <= 0.0) {
        return result;
    }
    
    // Minutes
    if (t_ms >= MIN_US) {
        intpart = int(floor(t_ms / MIN_US));
        t_ms -= intpart*MIN_US;
        result += (std::to_string(intpart) + 'm');
    }
    
    if (t_ms <= 0.0) {
        return result;
    }
    
    // Seconds
    if (t_ms >= SEC_US) {
        intpart = int(floor(t_ms / SEC_US));
        t_ms -= intpart*SEC_US;
        result += (std::to_string(intpart) + 's');
    }
    
    if (t_ms <= 0.0) {
        return result;
    }
    
    // MilliSeconds
    if (t_ms >= MS_US) {
        intpart = int(floor(t_ms / MS_US));
        t_ms -= intpart*MS_US;
        result += (std::to_string(intpart) + "ms");
    }
    
    if (t_ms <= 0.0) {
        return result;
    }
    
    // Remaining microseconds
    result += (std::to_string(t_ms) + "us");

    return result;
}


/**
 \param tos An output stream to write the result to.
 \param tprops Optional extra properties of the graph can be specified in this string.
 */
void Node::export2dot_full(std::ostream &tos, const std::string &tprops) const {
    // Graph header: node's name, settings,...
    tos << "digraph " << QUOTE << m_name << QUOTE << " {" << std::endl
    << TAB << "// DOT description of node " << m_name << " exported from SMNChai" << std::endl
    << TAB << "label = " << QUOTE << m_name << QUOTE << ";\n"
    << TAB << "labelloc = t;\n"
    << TAB << "rankdir = LR;\n\n";
    
    // Extra properties
    tos << TAB << tprops << std::endl;
    
    bool has_updates = !m_updates.empty();
    bool has_inputs = !m_inputs.empty();
    bool has_outputs = !m_outputs.empty();
    
    // Used for invisible connections
    std::string default_update_name;
    if (has_updates) {
        default_update_name = gen_update_name(m_updates.cbegin()->first);
    }
    
    std::string default_input_name;
    if (has_inputs) {
        default_input_name = std::string("\"") + 'I' + m_inputs.cbegin()->first + QUOTE;
    }
    
    std::string default_output_name;
    if (has_outputs) {
        default_output_name = std::string("\"") + 'O' + m_outputs.cbegin()->first + QUOTE;
    }
    
    // List of updates
    for (auto it = m_updates.cbegin(); it != m_updates.cend(); ++it) {
        tos << TAB << gen_update_name(it->first) << " [shape=box,label="
        << QUOTE << "UPDATE " << it->first << "\\n" << (it->second<=0?"nonperiodic":"T = "+gen_time_rep(it->second)) << QUOTE << "];" << std::endl;
    }
    
    std::string portname;
    
    // List of inputs
    for (auto it = m_inputs.cbegin(); it != m_inputs.cend(); ++it) {
        portname = std::string("\"") + 'I' + it->first + QUOTE;
        tos << TAB << portname
        << " [color=blue,label=" << QUOTE << it->first << QUOTE << "];" << std::endl;
        
        bool has_connection = false;    // If an input has no connection, a virtual, invisible one is created to enforce the order
        for (OBNsim::updatemask_t id = 0; id <= OBNsim::MAX_UPDATE_INDEX; ++id) {
            if (it->second & (OBNsim::updatemask_t(1) << id)) {
                // This UDPATE-id has this port as direct input
                tos << TAB << portname << "->" << gen_update_name(id) << ";\n";
                has_connection = true;
            }
        }
        
        if (!has_connection) {
            if (has_updates) {
                tos << TAB << portname << "->" << default_update_name << " [style=\"invis\"];\n";
            } else if (has_outputs) {
                tos << TAB << portname << "->" << default_output_name << " [style=\"invis\"];\n";
            }
        }
        
        tos << std::endl;
    }
    
    // List of outputs
    for (auto it = m_outputs.cbegin(); it != m_outputs.cend(); ++it) {
        portname = std::string("\"") + 'O' + it->first + QUOTE;
        tos << TAB << portname
        << " [color=red,label=" << QUOTE << it->first << QUOTE << "];" << std::endl;
        
        bool has_connection = false;    // If an output has no connection, a virtual, invisible one is created to enforce the order
        for (OBNsim::updatemask_t id = 0; id <= OBNsim::MAX_UPDATE_INDEX; ++id) {
            if (it->second & (OBNsim::updatemask_t(1) << id)) {
                // This UDPATE-id has this port as output
                tos << TAB << gen_update_name(id) << "->" << portname << ";\n";
                has_connection = true;
            }
        }
        
        if (!has_connection) {
            if (has_updates) {
                tos << TAB << default_update_name << "->" << portname << " [style=\"invis\"];\n";
            } else if (has_inputs) {
                tos << TAB << default_output_name << "->" << portname << " [style=\"invis\"];\n";
            }
        }
        
        tos << std::endl;
    }

    
    // Closing
    tos << '}' << std::endl;
}


/**
 \param tos An output stream to write the result to.
 \param tprops Optional extra properties of the node can be specified in this string.
 */
void Node::export2dot_compact(std::ostream &tos, const std::string &tprops) const {
    // Node header: node's name, settings,...
    tos << TAB << "// Node: " << m_name << std::endl
    << TAB << QUOTE << m_name << QUOTE << " [shape=none,width=0,height=0,margin=0,label=<" << std::endl
    // The main table of the node
    << TAB << "<TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">\n"
    // Name of the node, at the top in blue color
    << TAB << "<TR><TD CELLPADDING=\"2\"><FONT COLOR=\"blue\"><B>" << m_name << "</B></FONT></TD></TR>\n";
    
    // The table of the input and output ports
    if (!m_inputs.empty() || !m_outputs.empty()) {
        std::string portname;
        
        tos << TAB << "<TR><TD><TABLE BORDER=\"1\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\"><TR>\n";
        
        if (!m_inputs.empty()) {
            // Table of input ports in the left cell
            tos << TAB << "<TD ALIGN=\"LEFT\"><TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"4\">\n";
        
            // Create each line for each input port
            for (auto it = m_inputs.cbegin(); it != m_inputs.cend(); ++it) {
                portname = it->first;
                tos << TAB << "<TR><TD ALIGN=\"LEFT\" PORT=\"" << portname << "\">" << portname << "</TD></TR>\n";
            }
        
            // Closing the input ports table
            tos << TAB << "</TABLE></TD>\n";
        }
        
        if (!m_outputs.empty()) {
            // Table of output ports in the right cell
            tos << TAB << "<TD ALIGN=\"RIGHT\"><TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"4\">\n";
            
            // Create each line for each output port
            for (auto it = m_outputs.cbegin(); it != m_outputs.cend(); ++it) {
                portname = it->first;
                tos << TAB << "<TR><TD ALIGN=\"RIGHT\" PORT=\"" << portname << "\">" << portname << "</TD></TR>\n";
            }
            
            // Closing the output ports table
            tos << TAB << "</TABLE></TD>\n";
        }
        // Closing input and output ports table, cell and row
        tos << TAB << "</TR></TABLE></TD></TR>\n";
    }
    
    // Table of data ports
    if (!m_dataports.empty()) {
        // Create table for data ports
        tos << TAB << "<TR><TD><TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\" CELLPADDING=\"2\">\n";

        // Create each line for each data port
        for (auto it = m_dataports.cbegin(); it != m_dataports.cend(); ++it) {
            tos << TAB << "<TR><TD PORT=\"" << *it << "\"><I>" << *it << "</I></TD></TR>\n";
        }
        
        // Closing table for data ports
        tos << TAB << "</TABLE></TD></TR>\n";
    }
    
    // Closing node table
    tos << TAB << "</TABLE>>";
    
    
    // Extra properties
    if (!tprops.empty()) {
        tos << ",\n" << TAB << tprops;
    }
    tos << "];";
}


/**
 \param tos An output stream to write the result to.
 \param tprops Optional extra properties of the node can be specified in this string.
 */
void Node::export2dot_compact_cluster(std::ostream &tos, const std::string &tprops) const {
    // Node header: node's name, settings,...
    tos << TAB << "// Node: " << m_name << std::endl
    << TAB << "subgraph " << QUOTE << "cluster" << m_name << QUOTE << " {\n"
    << TAB << TAB << "label=\"" << m_name << "\";\n";
    
    // Extra properties
    if (!tprops.empty()) {
        tos << TAB << TAB << tprops << "\n\n";
    }
    
    std::string portname;
    
    // Input ports
    if (!m_inputs.empty()) {
        // Each port is a node
        for (auto it = m_inputs.cbegin(); it != m_inputs.cend(); ++it) {
            portname = m_name + '/' + it->first;
            tos << TAB << TAB << QUOTE << portname << QUOTE << " [shape=box,label=\"" << it->first << "\"];\n";
        }
    }
    
    // Output ports
    if (!m_outputs.empty()) {
        // Each port is a node
        for (auto it = m_outputs.cbegin(); it != m_outputs.cend(); ++it) {
            portname = m_name + '/' + it->first;
            tos << TAB << TAB << QUOTE << portname << QUOTE << " [shape=ellipse,label=\"" << it->first << "\"];\n";
        }
    }
    
    // Data ports
    if (!m_dataports.empty()) {
        // Each port is a node
        for (auto it = m_dataports.cbegin(); it != m_dataports.cend(); ++it) {
            portname = m_name + '/' + *it;
            tos << TAB << TAB << QUOTE << portname << QUOTE << " [shape=octagon,label=\"" << *it << "\"];\n";
        }
    }
    
    // Closing
    tos << TAB << '}';
}


/**
 \param tos An output stream to write the result to.
 \param t_cluster false [default] if each node is represented by a DOT's node and ports as DOT's ports; true if each node is a DOT's cluster and ports are DOT's nodes.
 \param tprops Optional extra properties of the graph can be specified in this string.
 */
void WorkSpace::export2dot(std::ostream &tos, bool t_cluster, const std::string &tprops) const {
    // Graph header
    tos << "digraph " << QUOTE << (m_name.empty()?"System":m_name) << QUOTE << " {" << std::endl
    << TAB << "// GraphViz's DOT description of the system exported from SMNChai" << std::endl
    //<< TAB << "label = " << QUOTE << (m_name.empty()?"System":m_name) << QUOTE << ";\n" << TAB << "labelloc = t;\n"
    << TAB << "rankdir = " << (t_cluster?"TB":"LR") << ';';
    
    // Extra properties
    if (!tprops.empty()) {
        tos << "\n" << TAB << tprops;
    }
    
    tos << "\n\n";
    
    // Output all nodes in the system
    tos
    << TAB << "/////////////////////////////////\n"
    << TAB << "// List of nodes in the system //\n"
    << TAB << "/////////////////////////////////\n\n";
    
    for (auto mynode = m_nodes.cbegin(); mynode != m_nodes.cend(); ++mynode) {
        if (t_cluster) {
            mynode->second.first.export2dot_compact_cluster(tos);
        } else {
            mynode->second.first.export2dot_compact(tos);
        }
        tos << std::endl << std::endl;
    }
    
    // Connect the ports
    tos
    << TAB << "//////////////////////////////////////\n"
    << TAB << "// Connections between nodes' ports //\n"
    << TAB << "//////////////////////////////////////\n\n";
    
    for (auto myconn = m_connections.cbegin(); myconn != m_connections.cend(); ++myconn) {
        {
            const auto nit = m_nodes.find(myconn->first.node_name);
            if (nit == m_nodes.end()) {
                throw smnchai_exception("In a connection, source node " + myconn->first.node_name + " does not exist.");
            } else {
                if (!nit->second.first.port_exists(myconn->first.port_name)) {
                    throw smnchai_exception("In a connection, source port " + myconn->first.port_name + " does not exist on node " + myconn->first.node_name);
                }
            }
        }
        
        {
            const auto nit = m_nodes.find(myconn->second.node_name);
            if (nit == m_nodes.end()) {
                throw smnchai_exception("In a connection, target node " + myconn->second.node_name + " does not exist.");
            } else {
                if (!nit->second.first.port_exists(myconn->second.port_name)) {
                    throw smnchai_exception("In a connection, target port " + myconn->second.port_name + " does not exist on node " + myconn->second.node_name);
                }
            }
        }
        
        if (t_cluster) {
            tos << TAB <<
            // From "source_node/source_port" TO
            QUOTE << myconn->first.node_name << '/' << myconn->first.port_name << QUOTE << " -> "
            // "target_node/target_port"
            << QUOTE << myconn->second.node_name << '/' << myconn->second.port_name << QUOTE << std::endl;
        } else {
            tos << TAB <<
            // From "source_node":source_port:e TO
            QUOTE << myconn->first.node_name << QUOTE << ':' << myconn->first.port_name << ":e -> "
            // "target_node":target_port:w
            << QUOTE << myconn->second.node_name << QUOTE << ':' << myconn->second.port_name << ":w\n";
        }
    }
    
    // The end
    tos << TAB << "// End of system description\n}\n";
}

/** Export the system to a DOT file.
 \param fn File name
 \param cluster true if each node is a cluster; false if each node is a simple node.
 */
void WorkSpace::export2dotfile(const std::string &fn, bool cluster) const {
    std::stringstream ss(std::ios_base::out);
    export2dot(ss, cluster);
    std::ofstream fs(fn);
    if (fs.is_open()) {
        fs << ss.str();
        fs.close();
    } else {
        throw smnchai_exception("Could not open file " + fn + " to write.");
    }
}