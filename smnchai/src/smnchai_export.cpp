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
    
    // List of updates
    for (auto it = m_updates.cbegin(); it != m_updates.cend(); ++it) {
        tos << TAB << gen_update_name(it->first) << " [shape=box,label="
        << QUOTE << "UPDATE " << it->first << "\\nT = " << gen_time_rep(it->second) << QUOTE << "];" << std::endl;
    }
    
    std::string portname;
    
    // List of inputs
    for (auto it = m_inputs.cbegin(); it != m_inputs.cend(); ++it) {
        portname = std::string("\"") + 'I' + it->first + QUOTE;
        tos << TAB << portname
        << " [color=blue,label=" << QUOTE << it->first << QUOTE << "];" << std::endl;
        
        for (OBNsim::updatemask_t id = 0; id <= OBNsim::MAX_UPDATE_INDEX; ++id) {
            if (it->second & (OBNsim::updatemask_t(1) << id)) {
                // This UDPATE-id has this port as direct input
                tos << TAB << portname << "->" << gen_update_name(id) << ";\n";
            }
        }
        
        tos << std::endl;
    }
    
    // List of outputs
    for (auto it = m_outputs.cbegin(); it != m_outputs.cend(); ++it) {
        portname = std::string("\"") + 'O' + it->first + QUOTE;
        tos << TAB << portname
        << " [color=red,label=" << QUOTE << it->first << QUOTE << "];" << std::endl;
        
        for (OBNsim::updatemask_t id = 0; id <= OBNsim::MAX_UPDATE_INDEX; ++id) {
            if (it->second & (OBNsim::updatemask_t(1) << id)) {
                // This UDPATE-id has this port as output
                tos << TAB << gen_update_name(id) << "->" << portname << ";\n";
            }
        }
        
        tos << std::endl;
    }
    
    // Closing
    tos << "}" << std::endl;
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
        
        tos << TAB << "<TR><TD><TABLE BORDER=\"1\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\"><TR>\n"
        // Table of input ports in the left cell
        << TAB << "<TD ALIGN=\"LEFT\"><TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"2\" CELLPADDING=\"2\">\n";
        
        // Create each line for each input port
        for (auto it = m_inputs.cbegin(); it != m_inputs.cend(); ++it) {
            portname = it->first;
            tos << TAB << "<TR><TD ALIGN=\"LEFT\" PORT=\"" << portname << "\">" << portname << "</TD></TR>\n";
        }
        
        // Closing the input ports table
        tos << TAB << "</TABLE></TD>\n"
        // Table of output ports in the right cell
        << TAB << "<TD ALIGN=\"RIGHT\"><TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"2\" CELLPADDING=\"2\">\n";
        
        // Create each line for each output port
        for (auto it = m_outputs.cbegin(); it != m_outputs.cend(); ++it) {
            portname = it->first;
            tos << TAB << "<TR><TD ALIGN=\"RIGHT\" PORT=\"" << portname << "\">" << portname << "</TD></TR>\n";
        }
        
        // Closing the output ports table
        tos << TAB << "</TABLE></TD>\n"
        // Closing input and output ports table, cell and row
        << TAB << "</TR></TABLE></TD></TR>\n";
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
