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
 Calculates the next (periodic) update/sync instants of specified output groups as well as of the node.
 
 \param grps Bit mask value to specify the output groups that will be calculated.
 */
void OBNNode::calcNextUpdateTime(outputmask_t grps) {
    // temporary variables for node's next periodic update
    simtime_t t = -1;
    outputmask_t m = 0;
    
    // Each node's bit-mask must be unique and non-zero, so we iterate until grps becomes 0
    for (auto it = output_groups.begin(); it != output_groups.end(); ++it) {
        if (it->period <= 0) {
            continue;    // if period <= 0, the output is non-periodic, so skip it
        }
        
        if (grps && ((grps & it->mask) == it->mask)) {
            it->next_update += it->period;
            grps ^= it->mask;  // remove the mask from grps
        }
        
        if (it->next_update == t) {
            m |= it->mask;  // include this group in the mask
        }
        else if ((it->next_update < t) || (t < 0)) {
            // Found an earlier time, or not yet set
            t = it->next_update;
            m = it->mask;
        }
    }
    
    next_update_time = t;
    next_update_grps = m;
}


/** This method does not recalculate the update time. It simply calculate the smaller of the regular and irregular update instants.
 
 Note that the returned update time can be < 0 if there is no periodic output group and no irregular update.
 \return A tuple of (1) the next update time, (2) 1 if regular update only, 2 if irregular update only, 3 if both, (3) irregular update mask>
 */
std::tuple<simtime_t, unsigned char, outputmask_t> OBNNode::getNextUpdateTime() const {
    simtime_t irTime;
    outputmask_t mask;
    
    // NOTE THAT next_update_time can be < 0 if no output groups are periodic
    if (nextIrregularUpdate(irTime, mask)) {
        // There is an irregular update, which must be >= 0
        if ((irTime <= next_update_time) || (next_update_time < 0)) {
            return std::make_tuple(irTime, (irTime==next_update_time)?3:2, mask);
        }
    }
    return std::make_tuple(next_update_time, 1, 0);
}



/**
 Initialize the node's state to (re)start a simulation.
 Mostly concern with resetting internal clocks and memory variables of the node.
 
 \return True if successful.
 */
bool OBNsmn::OBNNode::initialize() {
    // std::cout << "Initialize node..." << std::endl;
    
    next_update_grps = 0;
    next_update_time = -1;  // initialized to -1, in case all output groups are irregular
    
    // Reset the next update time of all output groups, and collect the mask bits of all periodic groups
    for (auto it = output_groups.begin(); it != output_groups.end(); ++it) {
        it->next_update = 0;
        if (it->period > 0) {
            next_update_grps |= it->mask;
            next_update_time = 0;
        }
    }
    
    // Reset the irregular update list
    irreg_updates.clear();
    
    return true;
}