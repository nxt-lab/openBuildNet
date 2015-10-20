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
#include <chrono>

namespace OBNsim {
    // Some constants
    extern const char *NODE_GC_PORT_NAME;
    
    // For debugging purposes
    // extern std::chrono::time_point<std::chrono::steady_clock> clockStart;
    
    typedef int64_t simtime_t;  ///< Simulation time type, as number of nano-seconds from beginning.
    typedef uint64_t updatemask_t;  ///< Update mask type: each bit corresponds to one update, so the width of the type is the maximum number of updates.
    const int MAX_UPDATE_INDEX = 63;    ///< The maximum index of update type allowed = number of bits in updatemast_t - 1
    
    
    namespace Utils {
        
        /** \brief Trim a string from spaces at both ends. */
        std::string trim(const std::string& s0);
        
        /** \brief Convert a string to upper-case. */
        std::string toUpper(const std::string& s);
        
        /** \brief Convert a string to lower-case. */
        std::string toLower(const std::string& s);
        
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
    
    /** A resizable buffer, used to store data for messages. */
    class ResizableBuffer {
        /** The binary data of the message */
        char* m_data = nullptr;
        
        /** The actual size of the message, not exceeding the allocated size. */
        std::size_t m_data_size = 0;
        
        /** The size of the allocated memory buffer (m_data). */
        std::size_t m_data_allocsize = 0;
        
    public:
        ~ResizableBuffer() {
            if (m_data) { delete [] m_data; }
        }
        
        /** Allocate the memory block _data given the new size. It will reuse memory if possible. It will change _size. */
        void allocateData(std::size_t newsize);
        
        /** Return the pointer to the buffer. The caller may write to the buffer, but not exceeding the allocated size. */
        char* data() {
            return m_data;
        }
        
        const char* data() const {
            return m_data;
        }
        
        /** Size of the buffer (not allocated size), in bytes. */
        std::size_t size() const {
            return m_data_size;
        }
        
        /** Allocated size of the buffer, in bytes. */
        std::size_t allocsize() const {
            return m_data_allocsize;
        }
        
    };
}


#endif // OBNSIM_BASIC_H

