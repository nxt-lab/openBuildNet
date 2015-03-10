/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Interface for global reporting in the SMN.
 *
 * This file implements a generic interface to report information, warnings, and errors in the SMN.
 * the reporting mechanism depends on the implementation, e.g., it may display in the console, display in a GUI, and/or write to a log file.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBN_SIM_SMN_obnsim_report_h
#define OBN_SIM_SMN_obnsim_report_h

#include <string>

namespace OBNsmn {
    /** \brief Report an error with a code and a textual message.
     \param code An integer code.
     \param msg A text message.
     */
    void report_error(int code, std::string msg);
    
    /** \brief Report a warning with a code and a textual message.
     \param code An integer code.
     \param msg A text message.
     */
    void report_warning(int code, std::string msg);
    
    /** \brief Report a piece of information with a code and a textual message.
     \param code An integer code.
     \param msg A text message.
     */
    void report_info(int code, std::string msg);
}

#endif
