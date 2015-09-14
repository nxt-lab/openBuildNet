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

#include <obnsim_basic.h>
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

/** Convert a given non-empty string to a string compliant with XML ID (xsd:NMTOKEN).
 The resulting string should contain only letters, digits, periods (.), hyphens (-), underscores (_), and colons (:).
 It can start with any of these letters.
 It doesn't contain any whitespace.
 */
std::string convert_to_xml_id(const std::string &s) {
    assert(!s.empty());
    
    std::string r = OBNsim::Utils::trim(s);
    assert(!r.empty());
    
    // Find and replace all invalid characters with _, except for /
    std::size_t pos = 0;
    while ((pos = r.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-_:/", pos)) != std::string::npos) {
        r[pos++] = '_';
    }
    
    // Find and replace all /'s with ::
    pos = 0;
    while ((pos = r.find('/', pos)) != std::string::npos) {
        r.replace(pos, 1, "::");
        pos += 2;
    }
    
    return r;
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
            if (it->second.m_mask & (OBNsim::updatemask_t(1) << id)) {
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
            if (it->second.m_mask & (OBNsim::updatemask_t(1) << id)) {
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
            tos << TAB << "<TR><TD PORT=\"" << it->first << "\"><I>" << it->first << "</I></TD></TR>\n";
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
            portname = m_name + '/' + it->first;
            tos << TAB << TAB << QUOTE << portname << QUOTE << " [shape=octagon,label=\"" << it->first << "\"];\n";
        }
    }
    
    // Closing
    tos << TAB << '}';
}

/**
 \param tos An output stream to write the result to.
 */
void Node::export2graphml(std::ostream &tos) const {
    // Node as a (nested) graph
    tos << "<graph id=" << QUOTE << (convert_to_xml_id(m_name) + "::") << QUOTE << " edgedefault=\"directed\">" << std::endl;

    std::string portname;
    
    // Input ports
    if (!m_inputs.empty()) {
        // Each port is a node
        for (auto it = m_inputs.cbegin(); it != m_inputs.cend(); ++it) {
            portname = convert_to_xml_id(m_name + "::" + it->first);
            tos << "<node id=" << QUOTE << portname << QUOTE << ">\n<data key=\"d5\">\n"
            << "<y:ShapeNode><y:NodeLabel>"<< it->first << "</y:NodeLabel><y:Shape type=\"rectangle\"/></y:ShapeNode>\n</data>\n</node>\n";
        }
    }
    
    // Output ports
    if (!m_outputs.empty()) {
        // Each port is a node
        for (auto it = m_outputs.cbegin(); it != m_outputs.cend(); ++it) {
            portname = convert_to_xml_id(m_name + "::" + it->first);
            tos << "<node id=" << QUOTE << portname << QUOTE << ">\n<data key=\"d5\">\n"
            << "<y:ShapeNode><y:NodeLabel>"<< it->first << "</y:NodeLabel><y:Shape type=\"ellipse\"/></y:ShapeNode>\n</data>\n</node>\n";
        }
    }
    
    // Data ports
    if (!m_dataports.empty()) {
        // Each port is a node
        for (auto it = m_dataports.cbegin(); it != m_dataports.cend(); ++it) {
            portname = convert_to_xml_id(m_name + "::" + it->first);
            tos << "<node id=" << QUOTE << portname << QUOTE << ">\n<data key=\"d5\">\n"
            << "<y:ShapeNode><y:NodeLabel>"<< it->first << "</y:NodeLabel><y:Shape type=\"octagon\"/></y:ShapeNode>\n</data>\n</node>\n";
        }
    }
    
    // Closing
    tos << "</graph>";
}

/**
 \param m_name The name of the system/network, which will be the name of the graph.
 \param m_nodes Collection of the nodes to be exported (see WorkSpace::m_nodes).
 \param m_connections Collection of the links between those nodes (see WorkSpace::m_connections); only nodes in m_nodes can be the sources and targets of these links.
 \param tos An output stream to write the result to.
 \param t_cluster false [default] if each node is represented by a DOT's node and ports as DOT's ports; true if each node is a DOT's cluster and ports are DOT's nodes.
 \param tprops Optional extra properties of the graph can be specified in this string.
 */
void SMNChai::export2dot(const std::string m_name,
                const std::map<std::string, std::pair<Node,std::size_t> > &m_nodes,
                const std::forward_list< std::pair<PortInfo, PortInfo> > &m_connections,
                std::ostream &tos, bool t_cluster, const std::string &tprops)
{
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


/**
 \param m_name The name of the system/network, which will be the name of the graph.
 \param m_nodes Collection of the nodes to be exported (see WorkSpace::m_nodes).
 \param m_connections Collection of the links between those nodes (see WorkSpace::m_connections); only nodes in m_nodes can be the sources and targets of these links.
 \param tos An output stream to write the result to.
 \param t_cluster false [default] if each node is represented by a DOT's node and ports as DOT's ports; true if each node is a DOT's cluster and ports are DOT's nodes.
 \param tprops Optional extra properties of the graph can be specified in this string.
 */
void SMNChai::export2graphml(const std::string m_name,
                         const std::map<std::string, std::pair<Node,std::size_t> > &m_nodes,
                         const std::forward_list< std::pair<PortInfo, PortInfo> > &m_connections,
                         std::ostream &tos)
{
    // Graph header
    tos << R"header(<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:java="http://www.yworks.com/xml/yfiles-common/1.0/java" xmlns:sys="http://www.yworks.com/xml/yfiles-common/markup/primitives/2.0" xmlns:x="http://www.yworks.com/xml/yfiles-common/markup/2.0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:y="http://www.yworks.com/xml/graphml" xmlns:yed="http://www.yworks.com/xml/yed/3" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://www.yworks.com/xml/schema/graphml/1.1/ygraphml.xsd">
<key for="port" id="d0" yfiles.type="portgraphics"/>
<key for="port" id="d1" yfiles.type="portgeometry"/>
<key for="port" id="d2" yfiles.type="portuserdata"/>
<key attr.name="url" attr.type="string" for="node" id="d3"/>
<key attr.name="description" attr.type="string" for="node" id="d4"/>
<key for="node" id="d5" yfiles.type="nodegraphics"/>
<key for="graphml" id="d6" yfiles.type="resources"/>
<key attr.name="url" attr.type="string" for="edge" id="d7"/>
<key attr.name="description" attr.type="string" for="edge" id="d8"/>
<key for="edge" id="d9" yfiles.type="edgegraphics"/>
)header";

    tos << "<graph id=" << QUOTE << (m_name.empty()?"System":convert_to_xml_id(m_name)) << QUOTE << " edgedefault=\"directed\">" << std::endl;
    
    // Output all nodes in the system, as nested graphs
    for (auto mynode = m_nodes.cbegin(); mynode != m_nodes.cend(); ++mynode) {
        tos << "<node id=" << QUOTE << convert_to_xml_id(mynode->first) << QUOTE << ">\n";
        
        // The label for the nested graph (group in yEd)
        tos << R"grouplabel(<data key="d5">
<y:ProxyAutoBoundsNode>
<y:Realizers active="0">
<y:GroupNode>
<y:NodeLabel alignment="right" autoSizePolicy="node_width" modelName="internal" modelPosition="t" visible="true">)grouplabel"
        << mynode->first
        << R"grouplabel(</y:NodeLabel>
<y:Shape type="roundrectangle"/>
</y:GroupNode>
</y:Realizers>
</y:ProxyAutoBoundsNode>
</data>
)grouplabel";

        // The node as a nested graoup
        mynode->second.first.export2graphml(tos);
        tos << "\n</node>\n";
    }
    
    // Connect the ports
    std::size_t edge_id = 0;  // Store the edge ID
    for (auto myconn = m_connections.cbegin(); myconn != m_connections.cend(); ++myconn, ++edge_id) {
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

        tos << "<edge id=" << QUOTE << 'e' << edge_id << QUOTE
        << " source=" << QUOTE << convert_to_xml_id(myconn->first.node_name + "::" + myconn->first.port_name) << QUOTE
        << " target=" << QUOTE << convert_to_xml_id(myconn->second.node_name + "::" + myconn->second.port_name) << QUOTE << "/>\n";
    }
    
    // The end
    tos << "</graph>\n</graphml>";
}


/**
 \param fn File name
 \param cluster true if each node is a cluster; false if each node is a simple node.
 */
void WorkSpace::export2dotfile(const std::string &fn, bool cluster, const std::string &tprops) const {
    std::stringstream ss(std::ios_base::out);
    SMNChai::export2dot(m_name, m_nodes, m_connections, ss, cluster, tprops);
    std::ofstream fs(fn);
    if (fs.is_open()) {
        fs << ss.str();
        fs.close();
    } else {
        throw smnchai_exception("Could not open file " + fn + " to write.");
    }
}

/**
 \param fn File name
 */
void WorkSpace::export2graphmlfile(const std::string &fn) const {
    std::stringstream ss(std::ios_base::out);
    SMNChai::export2graphml(m_name, m_nodes, m_connections, ss);
    std::ofstream fs(fn);
    if (fs.is_open()) {
        fs << ss.str();
        fs.close();
    } else {
        throw smnchai_exception("Could not open file " + fn + " to write.");
    }
}
