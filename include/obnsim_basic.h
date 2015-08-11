/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basic definitions.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSIM_BASIC_H
#define OBNSIM_BASIC_H

#include <cstdint>
#include <string>

namespace OBNsim {
    typedef int64_t simtime_t;  ///< Simulation time type, as number of nano-seconds from beginning.
    typedef uint64_t updatemask_t;  ///< Update mask type: each bit corresponds to one update, so the width of the type is the maximum number of updates.
    const int MAX_UPDATE_INDEX = 63;    ///< The maximum index of update type allowed = number of bits in updatemast_t - 1
    
    
    namespace Utils {
        
        /** \brief Trim a string from spaces at both ends. */
        std::string trim(const std::string& s0);
        
        /** \brief Check if a given name is a valid identifier.
         \param name A string to be checked.
         \return true if name is a valid identifier.
         */
        bool isValidIdentifier(const std::string &name);
        
        /** \brief Check if a given name is a valid node name.
         A valid node name consists of one or more valid identifiers separated by single forward slashes (/).
         The following are invalid node names:
         "_abc" (invalid identifier)
         "/abc/def" (begins with /)
         "abc/def/" (ends with /)
         "abc//def" (double slashes)
         "abc/_def" (second identifier is invalid)
         This name is valid: "abc/def"
         \param name A string to be checked.
         \return true if name is a valid node name.
         */
        bool isValidNodeName(const std::string &name);
    }
}


#endif // OBNSIM_BASIC_H

