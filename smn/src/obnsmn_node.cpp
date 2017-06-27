/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implement a node in the network.
 *
 * C++ file for the node class, which holds information about a node in the simulation network.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>
#include <cassert>
#include <obnsmn_node.h>

using namespace OBNsmn;

/**
 Finish the current update:
 - If the current update involves regular update types: calculates the next periodic update instants of update types in next_regupdate_mask, and updates next_regupdate_time and next_regupdate_mask.
 - If the current update involves an irregular update: pop/remove it from the list of irregular updates.
 
 This method should be called by the GC after each update iteration, particularly after getNextUpdate() for the current iteration.
 */
void OBNNode::finishCurrentUpdate() {
    if (next_update_type != IRREGULAR_UPDATE_ONLY) {
        // The current update involves regular update types
        
        // temporary variables for node's next periodic update
        simtime_t t = -1;
        updatemask_t m = 0;
        
        // Recalculate the update types that are in next_regupdate_mask
        // Each node's bit-mask must be unique and non-zero, so we iterate until next_regupdate_mask becomes 0
        for (auto it = update_types.begin(); it != update_types.end(); ++it) {
            if (it->period <= 0) {
                continue;    // if period <= 0, the output is non-periodic, so skip it
            }
            
            if (next_regupdate_mask && ((next_regupdate_mask & it->mask) == it->mask)) {
                it->next_update += it->period;
                next_regupdate_mask ^= it->mask;  // remove the mask from grps
            }
            
            if (it->next_update == t) {
                m |= it->mask;  // include this update type in the mask
            }
            else if ((it->next_update < t) || (t < 0)) {
                // Found an earlier time, or not yet set
                t = it->next_update;
                m = it->mask;
            }
        }
        
        next_regupdate_time = t;
        next_regupdate_mask = m;
    }
    
    if (next_update_type != REGULAR_UPDATE_ONLY) {
        // The current update involves an irregular update, so we must pop/remove it from the list of irregular updates
        assert(!irreg_updates.empty());
        irreg_updates.erase(irreg_updates.begin());
    }
}


/** This method does not recalculate the update time. It calculates the smaller of the regular and irregular update instants, calculates the combined update mask, and records whether the update is a regular one, or an irregular one, or both.
 
 This method is called when the GC calculates the next update instant.  The result may become invalid after the node requests for an irregular update, so this method should be called everytime the GC needs to calculate the next update instant.
 
 Note that the returned update time can be < 0 if there is no periodic update and no irregular update.
 \return The next update time.
 */
simtime_t OBNNode::getNextUpdate() {
    simtime_t irTime;
    updatemask_t mask;
    
    // NOTE THAT next_update_time can be < 0 if no output groups are periodic
    if (nextIrregularUpdate(irTime, mask)) {
        // There is an irregular update, which must be >= 0
        if ((irTime <= next_regupdate_time) || (next_regupdate_time < 0)) {
            if (irTime==next_regupdate_time) {
                // Both of them at the same time
                next_update_type = BOTH_UPDATES;
                next_update_mask = next_regupdate_mask | mask;
            } else {
                next_update_type = IRREGULAR_UPDATE_ONLY;
                next_update_mask = mask;
            }
            return irTime;
        }
    }
    
    // Only regular update
    next_update_type = REGULAR_UPDATE_ONLY;
    next_update_mask = next_regupdate_mask;
    return next_regupdate_time;
}



/**
 Initialize the node's state to (re)start a simulation.
 Mostly concern with resetting internal clocks and memory variables of the node.
 
 \return True if successful.
 */
bool OBNsmn::OBNNode::initialize() {
    // std::cout << "Initialize node..." << std::endl;
    
    next_regupdate_mask = 0;
    next_regupdate_time = -1;  // initialized to -1, in case all update types are irregular
    
    // Reset the next update time of all output groups, and collect the mask bits of all periodic groups
    for (auto it = update_types.begin(); it != update_types.end(); ++it) {
        it->next_update = 0;
        if (it->period > 0) {
            next_regupdate_mask |= it->mask;
            next_regupdate_time = 0;
        }
    }
    
    // Reset the irregular update list
    irreg_updates.clear();
    
    return true;
}

// Get blocks triggered by given blocks of this node.
void OBNsmn::OBNNode::triggerBlocks(updatemask_t blks, TriggerListType &lst) {
    if (!has_trigger_list) return;
    
    for (auto&& trg: trigger_list) {
        if (blks & trg.srcMask) {
            // given block(s) are in the list -> trigger target blocks
            if (lst.count(trg.tgtNode)) {
                // Exist -> Combine the masks
                lst[trg.tgtNode] |= trg.tgtMask;
            } else {
                lst[trg.tgtNode] = trg.tgtMask;
            }
        }
    }
}

// Add a new trigger
void OBNsmn::OBNNode::addTrigger(updatemask_t srcBlks, int tgtNode, updatemask_t tgtBlks) {
    has_trigger_list = true;
    
    // Loop through the current list, see if we can combine with an existing trigger
    for (auto&& trg: trigger_list) {
        if (trg.tgtNode == tgtNode) {
            if (trg.srcMask == srcBlks) {
                // Combine target
                trg.tgtMask |= tgtBlks;
                return;
            } else if (trg.tgtMask == tgtBlks) {
                // Combine source
                trg.srcMask |= srcBlks;
                return;
            } else if ((trg.srcMask & srcBlks) == srcBlks && (trg.tgtMask & tgtBlks) == tgtBlks) {
                // If new trigger is contained in existing trigger -> nothing to add
                return;
            } else if ((trg.srcMask | srcBlks) == srcBlks && (trg.tgtMask | tgtBlks) == tgtBlks) {
                // If new trigger contains existing trigger -> replace
                trg.srcMask = srcBlks;
                trg.tgtMask = tgtBlks;
                return;
            }
        }
    }
    
    // Not combined --> add new
    trigger_list.push_back({srcBlks, tgtNode, tgtBlks});
}
