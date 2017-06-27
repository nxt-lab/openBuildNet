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
#include <unordered_map>

#include <obnsmn_basic.h>
#include <obnsim_msg.pb.h>

namespace OBNsmn {
    
    class GCThread;  // will be used later to be a friend class
    
    /** \brief A node in the network.
     
     This class represents a node in the simulation network. It contains all information about the node that is required by the GC.
     It also contains memory variables used by the GC for synchronous simulation.
     
     The next update time of a node is determined by two updating mechanisms:
     
     - The regular/periodic updates, specified by its update types with positive periods, and are calculated by calcNextUpdateTime(): the next update time is in the variable next_update_time, and the group mask is in next_update_grps.
     - The irregular updates, requested by the node to the SMN: these updates are managed by a sorted list. The next irregular update time, if exists, can be obtained by method nextIrregularUpdate() and if it is used, it should be removed from the list by method popIrregularUpdate().
     
     The GC will contain a list of objects of this class to represent all the nodes in the network.
     */
    class OBNNode {
        friend class GCThread;

    public:
        /**
         \param name Identifier of the node.
         \param nUpdates Number of update types, must be >=0.
         */
        OBNNode(const std::string& _name, int _nUpdates): needUPDATEX(true), name(_name), update_types(_nUpdates) {
        }
        
        virtual ~OBNNode() { }
        
        /** \brief Send a message to the node.
         
         Sends an SMN2N message to the node (synchronously, i.e. it waits for the message to be actually sent to the node).
         The message object (msg) contains the necessary data of the message (type, time, data) which are set by the caller (the GC).
         The ID field (which indicates the ID of the node) is not set by the caller (GC), but may be set by this method depending on the communication protocol. Therefore the message object may be modified upon returning from this method, but it should not cause any problem.
         \param nodeID The node's ID, which is its index in the list of all nodes, managed by the GC.
         \param msg The message object, of type OBNSimMsg::SMN2N, that contains the message data.
         \return True if successful.
         \see sendMessageSync()
         */
        virtual bool sendMessage(int nodeID, OBNSimMsg::SMN2N &msg) = 0;
        
        
        /** \brief Returns the update type mask of the next update of this node. */
        updatemask_t getNextUpdateMask() const { return next_update_mask; }
        
        bool needUPDATEX;       ///< Whether this node needs the UPDATE_X message to update its internal state
        
    private:    // ====== DATA ======== //
        const std::string name; ///< Node's name (identifier as a string)
        
        // For regular updates
        simtime_t next_regupdate_time = -1; ///< The next time instant it will be regularly updated (synchronized by GC). It can be < 0 if no update types are periodic
        updatemask_t next_regupdate_mask;  ///< Mask value to identify the regular update types that will be updated next
        
        // For next update, including both regular and irregular
        enum {
            REGULAR_UPDATE_ONLY,    ///< The next update is regular only
            IRREGULAR_UPDATE_ONLY,  ///< The next update is irregular only
            BOTH_UPDATES            ///< The next update is both regular and irregular
        } next_update_type;
        updatemask_t next_update_mask;      ///< Mask value to identify the next update types (inc. both regular and irregular)
        
        /** \brief Update type structure.
         
         This structure represents an update type of a node, which includes all information and variables for the update.
         */
        struct UpdateType {
            simtime_t period;   ///< Sampling period of the update, <=0 if not periodic, >0 if periodic.
            updatemask_t mask;  ///< Bit mask that represents this update type
            simtime_t next_update;   ///< Next time instant when this group will be updated (periodic case)
        };
        
        std::vector<UpdateType> update_types; ///< Vector of all update types
        
        /** \brief Sorted list of requested irregular updates.
         
         The list of requested irregular updates is stored in a map, sorted by the update time instants as the keys.
         The values are the corresponding update type masks.
         For each update, all update types specified by the mask will be combined with regular updates at the same instant (if exist) and will be updated in the correct dependency order at that instant.
         Update types specified by the mask but do not exist will be omitted.
         */
        std::map<simtime_t, updatemask_t> irreg_updates;
        
        struct TriggerType {
            updatemask_t    srcMask;    ///< The mask of source blocks (of this node)
            int             tgtNode;    ///< ID of the target node
            updatemask_t    tgtMask;    ///< The mask of target blocks (of tgtNode)
        };
        
        /** \brief List of nodes and blocks triggered by the execution of blocks of this node.
         **/
        std::vector<TriggerType> trigger_list;
        
        bool has_trigger_list{false};   ///< Whether this node has a trigger list (blocks of this node can trigger other blocks)

    public:    // ====== METHODS ====== //
        
        /** \brief Configure an update type.
         Configure an update type with a given period and mask.
         
         \param idx Index of the update type to be configured, must be between 0 and (number of updates)-1
         \param period The sampling time of the update, <=0 if irregular (non-periodic).
         \param mask Unique mask value (bit pattern) of this update type.
         */
        void setUpdateType(size_t idx, simtime_t period, updatemask_t mask) {
            update_types[idx].period = period;
            update_types[idx].next_update = 0;
            update_types[idx].mask = mask;
        }
        
        /** \brief Configure an update type with automatic mask.
         Configure an update type with a given period and automatic bit mask where the bit corresponding to the index is set to 1.
         
         \param idx Index of the update type to be configured, must be between 0 and (number of updates)-1
         \param period The sampling time of the update, <=0 if irregular (non-periodic).
         */
        void setUpdateType(size_t idx, simtime_t period) {
            setUpdateType(idx, period, 1 << idx);
        }
        
        /** \brief Type for a list of nodes and blocks to be triggered. 
         **/
        typedef std::unordered_map<int, updatemask_t> TriggerListType;
        
        /** \brief Get blocks triggered by given blocks of this node.
         
         Returns the list of nodes and blocks (update types) triggered by given blocks of this node, directly to a list. If a node already exists in the list, modify the block mask.
         
         \param blks Mask value of the updating blocks of this node.
         \param lst Current trigger list.
         **/
        void triggerBlocks(updatemask_t blks, TriggerListType &lst);
        
        /** \brief Add a new trigger.
         
         Add a new trigger from given blocks of this node to a certain node and its blocks.
         \param srcBlks Source blocks (of this node).
         \param tgtNode Index of the target node.
         \param tgtBlks Target blocks (of the target node).
         **/
        void addTrigger(updatemask_t srcBlks, int tgtNode, updatemask_t tgtBlks);
        
    private:
        /** \brief Initialize the node to (re)start a simulation. */
        bool initialize();
 
        /** \brief Calculate next update/sync instants. */
        void finishCurrentUpdate();
        
        /** \brief Get the next update time, whether regular or irregular. */
        simtime_t getNextUpdate();
                
        
        /** \brief Insert an irregular update.
         
         Inserts an irregular update. It will be sorted in increasing order of the time instants.
         If an irregular update at the same time instant is already present, the new request will replace the old one.
         It is the responsibility of the caller to make sure that the requested time is in the future; otherwise, time will not be guaranteed to progress forward.
         
         \param reqT The requested update time instant (must be in the future, but not checked).
         \param mask Bit mask specifying the update types to be updated.
         */
        void insertIrregularUpdate(simtime_t reqT, updatemask_t mask) {
            //assert(reqT >= 0);
            
            // Insert the new update, replace existing one if necessary
            irreg_updates[reqT] = mask;
        }
      
        /** \brief Get the next irregular update
         
         Return the next irregular update, if exists. Does not remove the update from the list.
         \param reqT Reference to the returned update time.
         \param mask Reference to the returned update type mask.
         \return True if there is a next irregular update; false if no irregular update.
         */
        bool nextIrregularUpdate(simtime_t &reqT, updatemask_t &mask) const {
            if (irreg_updates.empty()) {
                return false;
            }
            auto it = irreg_updates.begin();
            reqT = it->first;
            mask = it->second;
            return true;
        }
    };

}

#endif
