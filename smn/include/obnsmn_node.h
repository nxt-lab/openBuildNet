/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
  * Header file for the node class, which holds information about a node in the simulation network.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */


#ifndef OBNSIM_NODE_H
#define OBNSIM_NODE_H

#include <cassert>
#include <string>
#include <vector>
#include <map>
#include <tuple>

#include <obnsmn_basic.h>
#include <obnsim_msg.pb.h>

namespace OBNsmn {
    
    class GCThread;  // will be used later to be a friend class
    
    /** \brief A node in the network.
     
     This class represents a node in the simulation network. It contains all information about the node that is required by the GC.
     It also contains memory variables used by the GC for synchronous simulation.
     
     The next update time of a node is determined by two updating mechanisms:
     
     - The regular/periodic update, specified by its output groups, and is calculated by calcNextUpdateTime(): the next update time is in the variable next_update_time, and the group mask is in next_update_grps.
     - The irregular update, requested by the node to the SMN: these updates are managed by a sorted list. The next irregular update time, if exists, can be obtained by method nextIrregularUpdate() and if it is used, it should be removed from the list by method popIrregularUpdate().
     
     The GC will contain a list of objects of this class to represent all the nodes in the network.
     */
    class OBNNode {
        friend class GCThread;

    public:
        /**
         \param name Identifier of the node.
         \param nOutputGrps Number of output groups, must be >=0.
         */
        OBNNode(const std::string& _name, int _nOutputGrps):
        name(_name), output_groups(_nOutputGrps)
        {
        }
        
        virtual ~OBNNode() { }
        
        /** \brief Asynchronously send a message to the node.
         
         Sends an SMN2N message to the node asynchronously, i.e. it does not wait for the message to be actually sent to the node.
         The message object (msg) contains the necessary data of the message (type, time, data) which are set by the caller (the GC).
         The ID field (which indicates the ID of the node) is not set by the caller (GC), but may be set by this method depending on the communication protocol. Therefore the message object may be modified upon returning from this method, but it should not cause any problem.
         \param nodeID The node's ID, which is its index in the list of all nodes, managed by the GC.
         \param msg The message object, of type OBNSimMsg::SMN2N, that contains the message data.
         \return True if successful.
         \see sendMessageSync()
         */
        virtual bool sendMessage(int nodeID, OBNSimMsg::SMN2N &msg) = 0;
        
        /* The following method is not used. */
        /* \brief Synchronously send a message to the node.
         
         Sends an SMN2N message to the node synchronously, i.e. it will wait until the message has been sent to the node.
         The method may time out, depending on the configuration of the communication framework.
         The message object (msg) contains the necessary data of the message (type, time, data) which are set by the caller (the GC).
         The ID field (which indicates the ID of the node) is not set by the caller (GC), but may be set by this method depending on the communication protocol. Therefore the message object may be modified upon returning from this method, but it should not cause any problem.
         \param nodeID The node's ID, which is its index in the list of all nodes, managed by the GC.
         \param msg The message object, of type OBNSimMsg::SMN2N, that contains the message data.
         \return True if successful.
         \see sendMessage()
         */
        // virtual bool sendMessageSync(int nodeID, OBNSimMsg::SMN2N &msg) = 0;
        
        /** \brief Returns the output mask of the next regular update of this node. */
        outputmask_t getNextUpdateMask() const { return next_update_grps; }

    private:    // ====== DATA ======== //
        const std::string name; ///< Node's name (identifier as a string)
        
        simtime_t next_update_time = -1; ///< The next time instant it will be updated (synchronized by GC). It can be < 0 if no output groups are periodic
        outputmask_t next_update_grps;  ///< Mask value to identify the output groups that will be updated next
        
        /** \brief Output group structure.
         
         This structure represents an output group of a node, which includes all information and variables for the group.
         */
        struct OutputGroup {
            simtime_t period;   ///< Sampling period of the group, <=0 if not periodic, >0 if periodic.
            outputmask_t mask;  ///< Bit mask that represents this output group
            simtime_t next_update;   ///< Next time instant when this group will be updated (periodic case)
        };
        
        std::vector<OutputGroup> output_groups; ///< Vector of all output groups
        
        /** \brief Sorted list of requested irregular updates.
         
         The list of requested irregular updates is stored in a map, sorted by the update time instants as the keys.
         The values are the corresponding output group masks.
         For each update, all output groups specified by the mask will be updated (in the correct dependency order) at that instant;
         the remaining mask bits (that do not correspond to any output group) will be passed to the node at the beginning of the update step.
         */
        std::map<simtime_t, outputmask_t> irreg_updates;

    public:    // ====== METHODS ====== //
        
        /** \brief Configure an output group.
         Configure an output group with a given period and mask.
         
         \param idx Index of the output group to be configured, must be between 0 and (number of groups)-1
         \param period The sampling time of the group, <=0 if irregular (non-periodic).
         \param mask Unique mask value (bit pattern) of this output group.
         */
        void setOutputGroup(size_t idx, simtime_t period, outputmask_t mask) {
            setOutputGroup(idx, period);
            output_groups[idx].mask = mask;
        }
        
        /** \brief Configure an output group with automatic mask.
         Configure an output group with a given period and automatic bit mask where the bit corresponding to the index is set to 1.
         
         \param idx Index of the output group to be configured, must be between 0 and (number of groups)-1
         \param period The sampling time of the group, <=0 if irregular (non-periodic).
         */
        void setOutputGroup(size_t idx, simtime_t period) {
            output_groups[idx].period = period;
            output_groups[idx].mask = (1 << idx);
            output_groups[idx].next_update = 0;
        }
        
    private:
        /** \brief Initialize the node to (re)start a simulation. */
        bool initialize();
 
        /** \brief Calculate next update/sync instants. */
        void calcNextUpdateTime(outputmask_t grps);
        
        /** \brief Get the next update time, whether regular or irregular. */
        std::tuple<simtime_t, unsigned char, outputmask_t> getNextUpdateTime() const;
                
        
        /** \brief Insert an irregular update.
         
         Inserts an irregular update. It will be sorted in increasing order of the time instants.
         If an irregular update at the same time instant is already present, the new request will replace the old one.
         It is the responsibility of the caller to make sure that the requested time is in the future; otherwise, time will not be guaranteed to progress forward.
         
         \param reqT The requested update time instant (must be in the future, but not checked).
         \param mask Bit mask specifying the output groups to be updated.
         */
        void insertIrregularUpdate(simtime_t reqT, outputmask_t mask) {
            //assert(reqT >= 0);
            
            // Insert the new update, replace existing one if necessary
            irreg_updates[reqT] = mask;
        }
      
        /** \brief Get the next irregular update
         
         Return the next irregular update, if exists. Does not remove the update from the list.
         \param reqT Reference to the returned update time.
         \param mask Reference to the returned output group mask.
         \return True if there is a next irregular update; false if no irregular update.
         */
        bool nextIrregularUpdate(simtime_t &reqT, outputmask_t &mask) const {
            if (irreg_updates.empty()) {
                return false;
            }
            auto it = irreg_updates.begin();
            reqT = it->first;
            mask = it->second;
            return true;
        }
        
        /** \brief Pop (remove) the next irregular update

         Pop (remove) the next irregular update, if exists.
         \return True if there is a next irregular update; false if no irregular update.
         \see nextIrregularUpdate
         */
        bool popIrregularUpdate() {
            if (irreg_updates.empty()) {
                return false;
            }
            
            irreg_updates.erase(irreg_updates.begin());
            return true;
        }
    };

}

#endif
