/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_ext.h
 * \brief Generic external interface, regardless of the communication network, of the openBuildNet simulation framework.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_EXT_H_
#define OBNNODE_EXT_H_

#include <vector>
#include <functional>

// Report an error and may terminate.
void reportError(const char* msg);

// Report a warning
void reportWarning(const char* msg);

/** Types and functions the external interface can use. */
#ifdef __cplusplus
extern "C" {
#endif
    
    /** The update mask type, see obnsim_basic.h for the definition. That should match the definition here. */
    typedef uint64_t OBNUpdateMask;
    
    /** The event type */
    enum OBNEI_EventType {
        OBNEI_INIT = 0,         // Init of simulation
        OBNEI_Y = 1,            // Update Y
        OBNEI_X = 2,            // Update X
        OBNEI_TERM = 3,         // Termination of simulation
        OBNEI_RCV = 4           // A port has received a message
    };

    /** Type to pass arguments of an event */
    union OBNEI_EventArg {
        OBNUpdateMask mask;     // Update mask (see above)
        size_t index;     // Index, equivalent to uint32_t
    };
    
    
    /* === Node interface === */
    
    // Create a new node object, given nodeName, workspace, and optional server address.
    // Returns 0 if successful; >0 if node already exists; <0 if error.
    // id stores the ID of the new node.
    int createOBNNode(const char* name, const char* workspace, const char* server, size_t* id);
    
    // Delete a node, given its ID
    // Returns 0 if successful; <0 if node doesn't exist.
    int deleteOBNNode(size_t id);
    
    
    
    /* === Port interface === */
    
    /** Port type. */
    enum OBNEI_PortType {
        OBNEI_Port_Input = 0,
        OBNEI_Port_Output = 1,
        OBNEI_Port_Data = 2
    };
    
    /** Container type. */
    enum OBNEI_ContainerType {
        OBNEI_Container_Scalar = 0,
        OBNEI_Container_Vector = 1,
        OBNEI_Container_Matrix = 2,
        OBNEI_Container_Binary = 3      // Raw bytes
    };
    
    /** Element type. */
    enum OBNEI_ElementType {
        OBNEI_Element_logical = 0,
        OBNEI_Element_double = 1,
        OBNEI_Element_int32 = 2,
        OBNEI_Element_int64 = 3,
        OBNEI_Element_uint32 = 4,
        OBNEI_Element_uint64 = 5
    };
    
    /** Format of messages between ports. */
    enum OBNEI_FormatType {
        OBNEI_Format_ProtoBuf = 0      // The default
    };
    
    /** Structure containing information about a port. */
    struct OBNEI_PortInfo {
        OBNEI_PortType type;
        OBNEI_ContainerType container;
        OBNEI_ElementType element_type;
        bool strict;
    };
    
    // Create a new input port on a node
    // Arguments: node ID, port's name, format type, container type, element type, strict or not
    // Returns port's id; or negative number if error.
    // id is an integer starting from 0.
    int createInputPort(size_t id,
                        const char* name,
                        OBNEI_FormatType format,
                        OBNEI_ContainerType container,
                        OBNEI_ElementType element,
                        bool strict);
    
    
    // Create a new output port on a node
    // Arguments: node ID, port's name, format type, container type, element type
    // Returns port's id; or negative number if error.
    // id is an integer starting from 0.
    int createOutputPort(size_t id,
                         const char* name,
                         OBNEI_FormatType format,
                         OBNEI_ContainerType container,
                         OBNEI_ElementType element);
    
    
    // Synchronous sending: request an output port to send its current value/message immediately and wait until it can be sent.
    // Note that this function does not accept a value to be sent; instead the value/message of the port is set by another function.
    // Args: node ID, port's ID
    // Returns: zero if successful
    // This function will return an error if the given port is not a physical output port.
    int outputSendSync(size_t nodeid, size_t portid);
    
    // Is there a value pending at an input port?
    // Args: node ID, port's ID
    // Returns: true if >0, false if =0, error if <0.
    int inputPending(size_t nodeid, size_t portid);
    
    // Returns information about a port.
    // Arguments: node ID, port's ID, pointer to an OBNEI_PortInfo structure to receive info
    // Returns: 0 if successful.
    int portInfo(size_t nodeid, size_t portid, OBNEI_PortInfo* pInfo);
    
    
    // Read the current value of a non-strict input port, or pop the top/front value of a strict input port.
    // Args: node object pointer, port's ID
    // Returns: value in an appropriate Matlab's type
    // For strict ports, if there is no value pending, a default / empty value will be returned.
    // If the port contains binary data, the function will return a string containing the binary data.
    
    
    
    
    /* === Misc === */
    // Returns the maximum ID allowed for an update type.
    int maxUpdateID();
    
    // Returns the last error/warning message as a C null-terminated string.
    // Do not try to modify the string.
    const char* lastErrorMessage();
    const char* lastWarningMessage();
    
#ifdef __cplusplus
}
#endif


/** Internal definitions used by implementation. */
namespace OBNNodeExtInt {
    
    /** These functions are used to lock and unlock a pointer.
     They MUST be defined by the actual external interface (e.g., Matlab's MEX, Julia, Python, etc.).
     By locking a pointer, the pointer (i.e., its memory region) is locked and remains valid, and the dynamic library should not be unloaded until all locked objects are released.
     By unlocking a pointer, the locking is undone. When all locked objected have been unlocked, the dynamic library may be unloaded.
     Note that unlocking a pointer does not delete the pointer; it simply removes the lock.
     
     Example: for Matlab's MEX interface, these correspond to mexLock() and mexUnlock(), and the pointer argument is not used.
     */
    void lockPointer(void*);
    void unlockPointer(void*);
    
    /** Manage object instances. Inspired by MEXPLUS. */
    template<class T>
    class Session {
    public:
        typedef std::vector< std::shared_ptr<T> > InstanceList;
        
        /** Create an instance.
         */
        static std::size_t create(T* instance) {
            InstanceList* instances = getInstances();
            
            // Find an empty slot
            std::size_t id = 0;
            auto sz = instances->size();
            for (; id < sz; ++id) {
                if (!(instances->at(id)))
                    break;
            }
            if (id < sz) {
                instances->at(id).reset(instance);      // Assign new pointer to this empty slot
            } else {
                instances->emplace_back(instance);      // Add new element
            }
            lockPointer((void*)instance);
            return id;
        }
        /** Destroy an instance.
         */
        static bool destroy(std::size_t id) {
            T* p = get(id);     // Get the pointer to unlock
            if (p) {
                unlockPointer((void*)p);
                // Destroy the object and reset the shared_ptr to null (i.e., empty)
                getInstances()->at(id).reset();
                return true;
            }
            return false;
        }
        /** Retrieve an instance or nullptr if no instance is found.
         */
        static T* get(std::size_t id) {
            InstanceList* instances = getInstances();
            if (id < instances->size() && id >= 0) {
                auto& p = instances->at(id);
                if (p) {
                    return p.get();
                }
            }
            return nullptr;
        }
        /** Check if the given id exists.
         */
        static bool exist(std::size_t id) {
            InstanceList* instances = getInstances();
            return id < instances->size() && id >= 0 && instances->at(id);
        }
        /** Check if an element exists with a predicate function.
         */
        static bool exist(std::function<bool(const T&)> pred) {
            InstanceList* instances = getInstances();
            for (auto it = instances->begin(); it != instances->end(); ++it) {
                if (*it) {
                    if (pred(*(*it))) {
                        return true;
                    }
                }
            }
            return false;
        }
        /** Clear all session instances.
         */
        static void clear() {
            InstanceList* instances = getInstances();
            for (auto it = instances->begin(); it != instances->end(); ++it) {
                if (*it) {
                    unlockPointer((void*)(it->get()));
                }
            }
            instances->clear();
        }
        
    private:
        /** Constructor prohibited.
         */
        Session() {}
        ~Session() {}
        
        /** Get static instance storage.
         */
        static InstanceList* getInstances() {
            static InstanceList instances;
            return &instances;
        }
    };
}


#endif /* OBNNODE_EXT_H_ */
