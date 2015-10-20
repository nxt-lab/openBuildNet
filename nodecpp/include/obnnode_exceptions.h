/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Defines exception classes for OBNNode.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef nodecpp_mynode_obnnode_exceptions_h
#define nodecpp_mynode_obnnode_exceptions_h

#include <exception>
#include <stdexcept>
#include <cassert>

#include <obnnode_basic.h>

namespace OBNnode {
    class port_error: public std::runtime_error {
    public:
        const PortBase* m_port;
        // const std::string m_info;
        
        explicit port_error(const PortBase * port, const std::string& info = ""): std::runtime_error(info), m_port(port) //, m_info(info)
        {
            assert(port);
        }
    };
    
    class inputport_error: public port_error {
    public:
        enum ErrorType {
            ERR_RAWMSG,
            ERR_READVALUE
        } m_errortype;
        
        inputport_error(const PortBase * port, ErrorType errortype, const std::string& info = ""): port_error(port, info), m_errortype(errortype)
        { }
    };
    
    class outputport_error: public port_error {
    public:
        enum ErrorType {
            ERR_SENDMSG
        } m_errortype;
        
        outputport_error(const OutputPortBase * port, ErrorType errortype, const std::string& info = ""): port_error(port, info), m_errortype(errortype)
        { }
    };
}

#endif
