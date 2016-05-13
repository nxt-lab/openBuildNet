/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_ext_internal.h
 * \brief Internal definitions used by implementation of the external interface.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef obnnode_ext_internal_h
#define obnnode_ext_internal_h

#include <vector>
#include <functional>

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
            lockPointer(static_cast<void*>(instance));
            return id;
        }
        /** Destroy an instance.
         */
        static bool destroy(std::size_t id) {
            T* p = get(id);     // Get the pointer to unlock
            if (p) {
                unlockPointer(static_cast<void*>(p));
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
                    unlockPointer(static_cast<void*>(it->get()));
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


#endif /* obnnode_ext_internal_h */
