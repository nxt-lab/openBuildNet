/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basic definitions.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cassert>
#include <obnsim_basic.h>
#include <cctype>

// Name of the GC port on any node
const char *OBNsim::NODE_GC_PORT_NAME = "_gc_";

// std::chrono::time_point<std::chrono::steady_clock> OBNsim::clockStart;

std::string OBNsim::Utils::trim(const std::string& s0) {
    std::string s(s0);
    std::size_t found = s.find_last_not_of(" \t\f\v\n\r");
    if (found != std::string::npos) {
        s.erase(found+1);
        found = s.find_first_not_of(" \t\f\v\n\r");
        s.erase(0, found);
    } else {
        s.clear();
    }
    
    return s;
}

std::string OBNsim::Utils::toUpper(const std::string& s0) {
    std::string s(s0);
    for (auto& c: s) {
        c = std::toupper(c);
    }
    return s;
}

std::string OBNsim::Utils::toLower(const std::string& s0) {
    std::string s(s0);
    for (auto& c: s) {
        c = std::tolower(c);
    }
    return s;
}

bool OBNsim::Utils::isValidIdentifier(const std::string &name) {
    if (name.empty() || name[0] == '_') {
        return false;
    }
    return name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") == std::string::npos;
}

bool OBNsim::Utils::isValidNodeName(const std::string &name) {
    if (name.empty() || name.front() == '_' || name.front() == '/' || name.back() == '/') {
        return false;
    }
    if (name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_/")
        != std::string::npos) {
        return false;
    }
    // Double forward slashes are invalid
    // Underscore following / is also invalid
    return (name.find("//") == std::string::npos) && (name.find("/_") == std::string::npos);
}

void OBNsim::ResizableBuffer::allocateData(std::size_t newsize) {
    m_data_size = newsize;
    
    if (m_data) {
        // If _allocsize >= _size, we reuse the memory block
        if (m_data_allocsize < m_data_size) {
            delete [] m_data;
        }
    }
    else {
        m_data_allocsize = 0;  // Make sure that _allocsize < _size
    }
    
    if (m_data_allocsize < m_data_size) {
        m_data_allocsize = (m_data_size & 0x0F)?(((m_data_size >> 4)+1) << 4):m_data_size;
        assert(m_data_allocsize >= m_data_size);
        m_data = new char[m_data_allocsize];
    }
}