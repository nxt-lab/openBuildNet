/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basics of nodechai for MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <exception>
#include "nodechai_mqtt.h"


namespace NodeChai {
    bool NodeChai::NodeFactoryMQTT::callback_x(int t_id, const OBNnode::UpdateType::UPDATE_CALLBACK& t_f) {
        if (!check_node_object()) {
            return false;
        }
        
        if (t_id < 0 || t_id > OBNsim::MAX_UPDATE_INDEX) {
            m_last_error = "The ID of the computation block is out of range.";
            return false;
        }
        
        // Check if the computation block already exists
        bool createBlock = (m_node->m_updates.size() <= t_id) || (!m_node->m_updates[t_id].enabled);
        
        if (createBlock) {
            return add_block(t_id, OBNnode::UpdateType::UPDATE_CALLBACK(), t_f);
        } else {
            // Replace the callback with this new function
            m_node->m_updates[t_id].x_callback = t_f;
            return true;
        }
    }
    
    bool NodeChai::NodeFactoryMQTT::callback_y(int t_id, const OBNnode::UpdateType::UPDATE_CALLBACK& t_f) {
        if (!check_node_object()) {
            return false;
        }
        
        if (t_id < 0 || t_id > OBNsim::MAX_UPDATE_INDEX) {
            m_last_error = "The ID of the computation block is out of range.";
            return false;
        }

        // Check if the computation block already exists
        bool createBlock = (m_node->m_updates.size() <= t_id) || (!m_node->m_updates[t_id].enabled);
        
        if (createBlock) {
            return add_block(t_id, t_f);
        } else {
            // Replace the callback with this new function
            m_node->m_updates[t_id].y_callback = t_f;
            return true;
        }
    }
    
    bool NodeChai::NodeFactoryMQTT::callback_init(const std::function<void ()>& f) {
        if (!check_node_object()) {
            return false;
        }
        
        m_node->m_onInitialization_callback = f;
        return true;
    }
    
    bool NodeChai::NodeFactoryMQTT::callback_term(const std::function<void ()>& f) {
        if (!check_node_object()) {
            return false;
        }
        
        m_node->m_onTermination_callback = f;
        return true;
    }
}