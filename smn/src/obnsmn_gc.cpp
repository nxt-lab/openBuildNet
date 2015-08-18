/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implement the Global clock.
 *
 * Implementation of the Global clock class (GCThread).
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>
#include <algorithm>
#include <obnsmn_gc.h>
#include <obnsmn_report.h>


/** \file
 # How is the main GC algorithm implemented?
 
 The main GC algorithm is essentially a state machine.
 However, it is complex because there is an unknown, potentially large, number of nodes that can send events to the GC.
 These events are usually low level, but many of them can be processed collectively in a general way (e.g. all ACK events).
 There is code that pre-processes the low-level events to produce high-level events.
 There is also code that processes any event that the state machine doesn't take.
 
 ## Special events
 
 There are several special types of events:

 - Wait-for: it's often that state machine needs to wait for all certain nodes to send a certain acknowledgement message for the state machine to advance (e.g. SIM_Y_ACK message).
 - Timer: there are cases when the state machine needs to react when it has been staying in a state for longer than a certain amount of time.
 
 ### Wait-for event
 
 When the variable waitfor_active = true, a wait-for event is active.
 A vector of boolean bits waitfor_states stores the current state of the waiting (vector<bool>).
 A variable waitfor_type stores the expected message type (e.g. SIM_Y_ACK).
 When a wait-for starts, respective bits of waitfor_states are reset, corresponding to the nodes that need to send ACK.
 Also, waitfor_active is set to true and waitfor_type is set.
 All ACK events are processed to check the bits in waitfor_states.
 When all the bits are set, the wait is over.
 To make the code more efficient, we will not check all the bits but keep track of the number of nodes what we are waiting for.
 Everytime a node sends an ACK, we check if the bit is 0 or already 1.
 If the bit is flipped, we will decrease the number of nodes we are waiting for; and when that number reaches 0, we know that all nodes have sent their ACKs.
 
 ### Timer event
 
 A variable will tell if a timer event is active.
 A variable contains the absolute end time.
 No thread is needed to impolement the timer.
 Instead, in waiting for the condition variable to wake up the thread (whenever a system request or an event is pushed), we use wait_until to set the maximum timeout.
 The wait condition also checks for timeout, and breaks the waiting loop if a timeout happens.
 Currently, the timer event is only used for wait-for (ACK) event as timeout.
 
 ## The main algorithm
 
 The main algorithm is sequential (cf. the main design document), which will call helper functions, e.g. to wait for ACKs, to send messages to nodes, etc.
 
 In each simulation iteration, GC will do the following steps:
 - Create a list of irregular update nodes and a list of regular update nodes (which may have overlaps).
 - Send irregular UPDATE_Y to all nodes in first list, wait for ACK.
 - Create a run-time node dependency graph from list 2, then repeat until the graph is empty
    - Extract next update nodes, send UPDATE_Y to them, wait for ACK.
 - Finally, send UPDATE_X to all nodes in both lists, without duplications. Wait for ACK.
 
 Lists 1 & 2 are maintained as a single pre-allocated array with three columns:
 1. Node ID
 2. Type of update: 1 if regular only, 2 if irregular only, 3 if both
 3. Irregular update mask, if column 2 is not 1.
 
 The array is pre-allocated with maximum number of rows (number of nodes), and a variable keeps track of the current length of the array.
 The algorithm goes through the array and picks the appropriate nodes to send messages to.
 */


using namespace OBNsmn;


// This function is the entry point for the GC thread.
void GCThread::GCThreadMain() {
    std::cout << "The GC thread." << std::endl;

    // Set to false if there is a critical error that the simulation should terminate immediately,
    //  even without sending TERM signals, but still does necessary cleanups.
    bool noCriticalError;
    
    // Send the SIM_INIT message to all nodes to start the simulation
    OBNSimMsg::MSGDATA *pMsgData = new OBNSimMsg::MSGDATA();
    pMsgData->set_t(initial_wallclock); // initial wallclock time
    pMsgData->set_i(sim_time_unit);     // simulation time unit, in microseconds
    
    noCriticalError = gc_send_to_all(0, OBNSimMsg::SMN2N_MSGTYPE_SIM_INIT, OBNSimMsg::N2SMN_MSGTYPE_SIM_INIT_ACK, pMsgData);

    // Wait for ACKs from all nodes, checking for initialization errors
    noCriticalError = noCriticalError && gc_wait_for_ack([this](const SMNNodeEvent* ev) {
        if (ev->has_i && (ev->i != 0)) {
            report_error(0, "Node \"" + _nodes[ev->nodeID]->name + "\" had initialization error #" + std::to_string(ev->i));
            return false;
        }
        return true;
    });
    
    // Running while the current state is not STOPPED
    while (noCriticalError && gc_exec_state != GCSTATE_TERMINATING) {
        if (!startNextUpdate()) {
            // Error, can't continue simulation
            break;
        }
        
        
        // Update-Y
        if (gc_update_size > 0) {
            // Create the run-time node graph, for regular updates
            rtNodeGraph = _nodeGraph->getRTNodeDepGraph(gc_update_list.begin(), gc_update_size);
            
            // Send regular UPDATE_Y messages to nodes in the run-time graph, in correct order
            bool success;
            while (!rtNodeGraph->empty()) {
                if (!(success = gc_send_update_y())) {
                    // Error, stop simulation
                    break;
                }
                
                // Wait for ACKs while processing all events: returns true if there is an error (e.g. timeout)
                if (!gc_wait_for_ack()) {
                    success = false;
                    break;
                }
            }
            if (!success) {
                break;
            }
        }
        
        // Send UPDATE_X to all nodes that are updated in this iteration
        if (!gc_send_update_x()) {
            // Error, stop simulation
            break;
        }
        
        // Wait for ACKs while processing all events: returns true if there is an error (e.g. timeout)
        if (!gc_wait_for_ack()) {
            break;
        }


        gc_timer_reset();   // Turn off the timer, just in case
        
        // If the GC is paused, we wait until it is either resumed or stepped.
        OBNEventQueueType::item_type ev;    // To receive the node event
        OBNSysRequestType sysreq;  // To receive the system request
        
        while (gc_exec_state == GCSTATE_PAUSED) {
            // Obtain the next event, there should not be any timeout
            gc_wait_for_next_event(ev, sysreq);
            
            // Process some urgent system request
            if (sysreq == SYSREQ_TERMINATE) {
                // stop the simulation immediately by setting GC state to terminating and break from the loop
                gc_exec_state = GCSTATE_TERMINATING;
                break;
            }
            
            // Process the node event
            if (ev) {
                if (!gc_process_node_events(ev.get())) {
                    // There is an error, must terminate
                    gc_exec_state = GCSTATE_TERMINATING;
                    break;
                }
            }
            
            // Process the system request
            // Only the following need to be processed (and cleared after that):
            //   SYSREQ_STOP to stop
            //   SYSREQ_PAUSE to pause
            //   SYSREQ_RESUME to resume to running if possible
            if (sysreq != SYSREQ_NONE) {
                gc_process_sysreq(sysreq);
                
                resetSysRequest(); // here we can reset the request because it's definitely processed
                
                // If sysreq is STEP, break from the loop to run one iteration
                if (sysreq == SYSREQ_STEP) {
                    break;
                }
            }
        }

    }
    
    // The simulation is going to be terminated, send terminating messages to all nodes, unless there was a critical error
    if (noCriticalError) {
        noCriticalError = gc_send_to_all(current_sim_time, OBNSimMsg::SMN2N_MSGTYPE_SIM_TERM);
    }

    // Signal simple threads, which are associated with this GC, to terminate
    simple_thread_terminate = true;
}


/**
 Initialize the simulation before it can start, e.g. reset the clock, reset the node's state.
 
 \return True if successful.
 */
bool GCThread::initialize() {
    std::cout << "Initiaize GC ..." << std::endl;
    
    // Check that _nodes has at least one node
    if ((maxID = _nodes.size() - 1) < 0) {
        report_error(0, "There are no nodes in the system; at least one node must be present.");
        return false;
    }
    
    // Check simulation time unit
    if (sim_time_unit <= 0) {
        report_error(0, "Invalid simulation time unit (" + std::to_string(sim_time_unit) + ").");
        return false;
    }

    // Check final time
    if (final_sim_time <= 0) {
        report_error(0, "Invalid final simulation time (" + std::to_string(final_sim_time) + ").");
        return false;
    }
    
    // Check that _nodeGraph is set, but doesn't check if the graph is valid
    if (!_nodeGraph) {
        report_error(0, "Internal error: There is no node dependency graph.");
        return false;
    }
    
    // Initialize all the nodes
    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
        if (!(*it)->initialize()) {
            return false;
        }
    }
    
    simple_thread_terminate = false;  // Reset the simple thread termination signal
    gc_exec_state = GCSTATE_RUNNING;  // Initially, the simulation is running in the normal mode.
    
    resetSysRequest();
    
    // Start the simulation clock at < 0 to ensure that the next update will be at t=0,
    // otherwise it will be an error (no progress).
    current_sim_time = -1;
    
    // Reset the wait-for mechanism
    gc_waitfor_active = false;  // Turn off wait-for event
    gc_waitfor_bits.resize(_nodes.size());  // One bit for each node
    std::fill(gc_waitfor_bits.begin(), gc_waitfor_bits.end(), true);    // Set all bits

    
    // Turn off timer event
    gc_timer_reset();
    
    // Pre-allocate the update info list
    gc_update_list.resize(_nodes.size());
    gc_update_size = 0;
    
    return true;
}


/** This method starts a new update iteration of the GC algorithm by
 - Calculate the next update time and the list of update details.
 - Determine if the simulation will continue at that next update time.
 - Update the current simulation time.
 
 \return true if the simulation can continue
 */
bool GCThread::startNextUpdate() {
    // All nodes should have already updated their next update times
    simtime_t t = -1;
    simtime_t t_node;
    
    // Reset the update info list
    gc_update_size = 0;
    auto updateIt = gc_update_list.begin();
    
    // Iterate the node list and find the next update time, while filling in the update info list
    int nodeID = 0;
    for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
        t_node = (*it)->getNextUpdate();
        
        // NOTE THAT node's update time can be < 0, which means it is completely irregular and no next update time
        if (t_node >= 0) {
            if (t_node == t) {
                // Add the node to the list
                gc_update_size++;  // increase total nodes
                
                (updateIt++)->nodeID = nodeID;
            }
            else if ((t_node < t) || (t < 0)) {
                // Found earlier update (or first one), reset everything and add node
                t = t_node;
                gc_update_size = 1;
                
                //*(gc_update_list.begin()) = *updateIt;  // copy record to first
                updateIt = gc_update_list.begin();      // reset pointer
                (updateIt++)->nodeID = nodeID;
            }
        }
        nodeID++;
    }
    
    // Continue if and only if not exceeding end time and there is progress (i.e. there is a next update time)
    if (gc_update_size == 0 || t <= current_sim_time) {
        report_error(0, "There is no progress (no next update time) after simulation time " + std::to_string(current_sim_time));
        return false;
    }
    if (t > final_sim_time) {
        report_info(0, "Reached final simulation time; stop now.");
        return false;
    }
    
    // Now that the list of updating nodes is determined, we populate the update type masks of these nodes into the list
    updateIt = gc_update_list.begin();
    for (int k = 0; k < gc_update_size; ++k, ++updateIt) {
        updateIt->updateMask = _nodes[updateIt->nodeID]->getNextUpdateMask();
    }
    
    // Update simulation time, and continue the simulation
    current_sim_time = t;
    
    return true;
}

/**
 This method waits for the next event to arrive at the GC, which include:
 node event (message), system request, and time-out.
 \param ev OBNEvent pointer (shared_ptr) that will receive the next event, if available. It is null if no event.
 \param req Current system request value.
 \return false if timed out; true otherwise.
 */
bool GCThread::gc_wait_for_next_event(OBNEventQueueType::item_type& ev, OBNSysRequestType& req) {
    // We will control directly both locks, to achieve the best performance by saving one lock() and one unlock()
    // It may not seem much, but in overloaded communication, it saves time.
    
    bool gc_timer_fired = false;  // Has the timer event fired?
    
    std::unique_lock<std::mutex> slock(mSysReq);  // Lock on the system request
    std::unique_lock<std::mutex> qlock(OBNEventQueue.getMutex());  // Lock on the queue access
    
    // Because we have locks on both SysRequest and the event queue, we can access them directly
    // PRE-CONDITION: both slock and qlock are locked.
    while (SYSREQ_NONE == _SysRequest && OBNEventQueue.empty_with_lock()) {
        // If a timer event is active and the end time has passed, break the loop
        gc_timer_fired = gc_timer_active && gc_timer_endtime <= std::chrono::steady_clock::now();
        if (gc_timer_fired) {
            // Because the timer event has fired, we need to reset it
            gc_timer_reset();
            break;
        }
        
        // Unlock queue immediately to allow communication threads to push to it, which happens more frequently than system request being set.
        qlock.unlock();
        
        // If a timer event is active, use wait_until(), otherwise use normal wait()
        if (gc_timer_active) {
            mWakeupCondition.wait_until(slock, gc_timer_endtime);
        } else {
            mWakeupCondition.wait(slock);
        }
        
        // At this point, slock is locked, but qlock is not, so we need to lock qlock
        qlock.lock();
    }
    // POST-CONDITION: both slock and qlock are locked.
    
    // Copy the next event, if available, and unlock qlock for others to push to
    ev = OBNEventQueue.try_pop_with_lock();
    // qlock.unlock();  // unlocked automatically when returning from this function
    
    // Get system request, note that we have the lock on SysRequest
    req = _SysRequest;
    // slock.unlock(); // unlocked automatically when returning from this function
    
    return !gc_timer_fired;
}


/** This method blocks the execution until the current wait-for event finishes, or until the simulation must stop abnormally (e.g. due to an error).
 While blocking, all events are processed as usual (e.g. irregular update requests, GC services, logging).
 
 The method accepts a functor or lambda function that checks the event object (const SMNNodeEvent*) and returns true if the ACK event is valid, or false if not (and the wait-for will stop with an error).
 The default is [](const OBNsmn::SMNNodeEvent* ev) {return true;} which always accepts the event.
 
 \return false if the simulation must stop unexpectedly (e.g. error); true if the wait-for has finished and the simulation can continue normally.
 */
template <class F>
bool GCThread::gc_wait_for_ack(F f) {
    if (!gc_waitfor_active) {
        // Clearly if no wait-for is active, we don't need to block
        return true;
    }
    
    OBNEventQueueType::item_type ev;    // To receive the node event
    OBNSysRequestType sysreq;  // To receive the system request
    bool timed_out;

    // Loop until the wait-for is done, or until an unexpected termination
    // The loop is broken out by conditions inside the loop
    while (true) {
        // Obtain the next event
        timed_out = !gc_wait_for_next_event(ev, sysreq);
        
        // Process some urgent system request
        if (sysreq == SYSREQ_TERMINATE) {
            // stop the simulation immediately, even in the middle of a time step
            return false;
        }

        // Process the node event
        if (ev) {
            // Process ACK event for wait-for
            if (ev->category == SMNNodeEvent::EVT_ACK) {
                if (gc_waitfor_type == ev->type) {
                    // Call f(): if true then  mark the ACK event; if false then it's an error -> quit
                    if (!f(ev.get())) {
                        return false;
                    }
                    gc_waitfor_mark(ev->nodeID);
                    if (gc_waitfor_alldone()) {
                        gc_waitfor_active = false;  // Turn off waitfor
                        return true;
                    }
                }
                else {
                    report_warning(0, "Unexpected ACK received from node " + std::to_string(ev->nodeID) + " with type " + std::to_string(ev->type));
                    if (!gc_process_node_events(ev.get())) {
                        return false;
                    }
                }
            }
            else if (!gc_process_node_events(ev.get())) {
                return false;
            }
        }
        
        // Process the system request
        // Only the following need to be processed (and cleared after that):
        //   SYSREQ_STOP to stop
        //   SYSREQ_PAUSE to pause
        //   SYSREQ_RESUME to resume to running if possible
        if (sysreq != SYSREQ_NONE) {
            gc_process_sysreq(sysreq);
            
            // If sysreq is not STEP, it will be reset to NONE because it is consumed / processed
            if (sysreq != SYSREQ_STEP) {
                resetSysRequest();
            }
        }
        
        // Process time out
        // For now, we will terminate the simulation immediately.
        // In the future, we may have different options here, e.g. termination vs. raising a warning but continuing.
        if (timed_out) {
            report_error(0, "Timeout while waiting for ACKs at simulation time " + std::to_string(current_sim_time));
            return false;
        }
    }
    
    return true;  // This statement is actually never reached!
}



/** This method implements the default processing algorithm for node events, e.g. to respond to irregular update requests.
 It is usually the last function used to process any node event.
 \param pEv Non-nil pointer to the node event object to be processed.
 \return false if the simulation must stop unexpectedly (e.g. error); true otherwise.
 */
bool GCThread::gc_process_node_events(OBNsmn::SMNNodeEvent* pEv) {
    switch (pEv->type) {
        case OBNSimMsg::N2SMN_MSGTYPE_SIM_EVENT:
        {
            OBNSimMsg::SMN2N msg;
            msg.set_msgtype(OBNSimMsg::SMN2N_MSGTYPE_SIM_EVENT_ACK);
            msg.set_time(current_sim_time);
            
            auto *data = new OBNSimMsg::MSGDATA;

            // Request for an irregular update
            // It must contain the requested time
            if (pEv->has_t) {
                data->set_t(pEv->t);
                
                if (pEv->t > current_sim_time) {
                    // Requested time is in the future: it's accepted
                    data->set_i(0);  // OK
                    _nodes[pEv->nodeID]->insertIrregularUpdate(pEv->t, (pEv->has_i)?pEv->i:0);
                    //report_info(0, "Accept event for node " + _nodes[pEv->nodeID]->name + " for mask " + std::to_string((pEv->has_i)?pEv->i:0) + " at time " + std::to_string(pEv->t));
                }
                else {
                    // Requested time is invalid: denied
                    report_warning(0, "An invalid irregular update request from node " + std::to_string(pEv->nodeID) +
                                   " (" + _nodes[pEv->nodeID]->name + ") with past time.");

                    data->set_i(1);  // Error code 1 = invalid requested time (past time)
                }
            }
            else {
                report_warning(0, "An irregular update request from node " + std::to_string(pEv->nodeID) +
                               " (" + _nodes[pEv->nodeID]->name + ") doesn't specify time; it is ignored.");
                data->set_t(0);
                data->set_i(2);  // Error code 2 = no time requested
            }
            
            msg.set_allocated_data(data);
            _nodes[pEv->nodeID]->sendMessage(pEv->nodeID, msg);
            
            break;
        }
    
        default:
            report_warning(0, "Unrecognized and unprocessed message of type " + std::to_string(pEv->type) +
                           " from node " + std::to_string(pEv->nodeID) + " (" + _nodes[pEv->nodeID]->name + ")");
            break;
    }
    
    return true;
}


/** Process system request, mostly to switch between modes.
 If it is RUNNING, it can switch to PAUSED or TERMINATING.
 If it is PAUSED, it can switch to RUNNING (by RESUME) or TERMINATING (by STOP).
 If it is TERMINATING, it can switch to RUNNING (by RESUME) or PAUSED (by PAUSE) before it reaches TERMINATED, which is unrecoverable.
 \param req The system request.
 */
void GCThread::gc_process_sysreq(OBNSysRequestType req) {
    switch (gc_exec_state) {
        case GCSTATE_RUNNING:
            if (req == SYSREQ_STOP) {
                gc_exec_state = GCSTATE_TERMINATING;
            } else if (req == SYSREQ_PAUSE) {
                gc_exec_state = GCSTATE_PAUSED;
            }
            break;
        case GCSTATE_PAUSED:
            if (req == SYSREQ_RESUME) {
                gc_exec_state = GCSTATE_RUNNING;
            } else if (req == SYSREQ_STOP) {
                gc_exec_state = GCSTATE_TERMINATING;
            }
            break;
        case GCSTATE_TERMINATING:
            if (req == SYSREQ_RESUME) {
                gc_exec_state = GCSTATE_RUNNING;
            } else if (req == SYSREQ_PAUSE) {
                gc_exec_state = GCSTATE_PAUSED;
            }
            break;
    }
}


/**
 This method extracts the list of nodes that are next to be updated, sends (regular) UPDATE_Y messages to them.
 Only if messages are sent to nodes, it will start wait-for for them and may start a new timer event if timeout is used for UPDATE_Y.
 It also starts a new timer event if timeout is used for UPDATE_Y.
 
 \return true if successful; false if not (error)
*/
bool GCThread::gc_send_update_y() {
    assert(rtNodeGraph);  // rt_node_graph must be non-empty

    if (gc_waitfor_active) {
        // It's an error that wait-for is still active
        report_error(0, "Internal error: wait-for event is active before sending regular Y updates.");
        return false;
    }
    
    auto updateList = rtNodeGraph->getAndRemoveIndependentNodes();  // Get the list of updating nodes
    if (updateList.empty()) {
        // If no nodes are removed but the graph is non-empty, there is a cyclic condition (algebraic loop), which is an error
        if (rtNodeGraph->empty()) {
            return true;
        }

        report_error(0, "An algebraic loop (dependency cycle) occurs at time " + std::to_string(current_sim_time));
        return false;
    }
    
    // Send the UPDATE_Y messages to the nodes
    OBNSimMsg::SMN2N msg;
    msg.set_msgtype(OBNSimMsg::SMN2N_MSGTYPE_SIM_Y);
    msg.set_time(current_sim_time);
    
    for (auto node: updateList) {
        // We don't set ID here because it's dependent on the comm protocol (see node.sendMessage())
        // msg.set_id(node);

        OBNSimMsg::MSGDATA *msgdata = new OBNSimMsg::MSGDATA();
        msgdata->set_i(_nodes[node]->getNextUpdateMask());
        msg.set_allocated_data(msgdata);
        _nodes[node]->sendMessage(node, msg);
        
        // Because the regular update of this node is used, we update it to the next regular update.
        _nodes[node]->finishCurrentUpdate();
    }
    
    // Set up wait-for
    if (!gc_waitfor_start(updateList, OBNSimMsg::N2SMN::SIM_Y_ACK)) {
        return false;
    }
    
    // Set up timeout if necessary
    if (ack_timeout > 0) {
        gc_timer_start(ack_timeout);
    }
    else {
        gc_timer_reset();
    }
    
    return true;
}


/* This method sends irregular UPDATE_Y to nodes in the updating list.
 Only if messages are sent to nodes, it will start wait-for for them and may start a new timer event if timeout is used for UPDATE_Y.
 Here we will directly start the wait-for event without using the method gc_waitfor_start().

 \sa gc_update_list
 \return true if successful; false if not (error)
 */
/*
bool GCThread::gc_send_update_y_irregular() {
    if (gc_waitfor_active) {
        // It's an error that wait-for is still active
        report_error(0, "Internal error: wait-for event is active before sending irregular Y updates.");
        return false;
    }
    
    gc_waitfor_num = 0;
    
    // Send the irregular UPDATE_Y messages to the irregular update nodes in gc_update_list
    OBNSimMsg::SMN2N msg;
    msg.set_msgtype(OBNSimMsg::SMN2N_MSGTYPE_SIM_YI);
    msg.set_time(current_sim_time);
    
    for (IrregUpdateListIterator it(this); !it.atEnd(); ++it) {
        auto ID = it.getID();
        
        // We don't set ID here because it's dependent on the comm protocol (see node.sendMessage())
        // msg.set_id(ID);
        
        OBNSimMsg::MSGDATA *msgdata = new OBNSimMsg::MSGDATA();
        msgdata->set_i(it.getMask());
        msg.set_allocated_data(msgdata);

        _nodes[ID]->sendMessage(ID, msg);
        
        // Because the irregular update of this node is used, we remove/pop it from the list
        _nodes[ID]->popIrregularUpdate();
        
        // Mark the corresponding bit for wait-for event
        gc_waitfor_bits[ID] = false;
        ++gc_waitfor_num;  // One more node to wait for ACK
    }
    
    // If no messages were sent then no need for wait-for event nor time-out
    if (gc_waitfor_num > 0) {
        // Set up wait-for
        gc_waitfor_active = true;
        gc_waitfor_type = OBNSimMsg::N2SMN::SIM_YI_ACK;
        
        // Set up timeout if necessary
        if (ack_timeout > 0) {
            gc_timer_start(ack_timeout);
        }
        else {
            gc_timer_reset();
        }
    }
    
    return true;
}*/


/** This method sends UPDATE_X to nodes in the updating list and start wait-for for them.
 Here we will directly start the wait-for event without using the method gc_waitfor_start().
 It also starts a new timer event if timeout is used for UPDATE_X.
 \sa gc_update_list
 \return true if successful; false if not (error)
 */
bool GCThread::gc_send_update_x() {
    assert(gc_update_size > 0);
    
    if (gc_waitfor_active) {
        // It's an error that wait-for is still active
        report_error(0, "Internal error: wait-for event is active before sending regular X updates.");
        return false;
    }
    
    
    // Send the UPDATE_X messages to all nodes in gc_update_list
    int numUpdateX = 0;    // Number of nodes receiving UPDATE_X
    
    OBNSimMsg::SMN2N msg;
    msg.set_msgtype(OBNSimMsg::SMN2N_MSGTYPE_SIM_X);
    msg.set_time(current_sim_time);
    
    for (size_t k = 0; k < gc_update_size; ++k) {
        auto ID = gc_update_list[k].nodeID;
        
        if (_nodes[ID]->needUPDATEX) {
            // We don't set ID here because it's dependent on the comm protocol (see node.sendMessage())
            // msg.set_id(ID);
            
            _nodes[ID]->sendMessage(ID, msg);
            
            // Mark the corresponding bit for wait-for event
            gc_waitfor_bits[ID] = false;
            
            ++numUpdateX;
        }
    }

    if (numUpdateX > 0) {
        // Set up wait-for
        gc_waitfor_num = numUpdateX;
        gc_waitfor_active = true;
        gc_waitfor_type = OBNSimMsg::N2SMN::SIM_X_ACK;
        
        // Set up timeout if necessary
        if (ack_timeout > 0) {
            gc_timer_start(ack_timeout);
        }
        else {
            gc_timer_reset();
        }
    }
    
    return true;
}


/** Sends a given simple message to all nodes without waiting for ACKs.
 The message is simple with no custom data.
 \param t The time value sent with the message.
 \param msgtype The type of the message sent to nodes.
 \param pData Pointer to an optional message data block; null pointer (default) if no data. The function will take ownership of this data, therefore the data block should be created dynamically and not released by the caller.
 \return true if successful; false if not (error)
 */
bool GCThread::gc_send_to_all(simtime_t t, OBNSimMsg::SMN2N::MSGTYPE msgtype, OBNSimMsg::MSGDATA *pData) {
    // Send the message to all nodes
    OBNSimMsg::SMN2N msg;
    msg.set_msgtype(msgtype);
    msg.set_time(t);
    
    // Set message data (none = clear if NULL)
    msg.set_allocated_data(pData);

    int k = 0;
    for (auto it = _nodes.begin(); it != _nodes.end(); ++it, ++k) {
        // We don't set ID here because it's dependent on the comm protocol (see node.sendMessage())
        // msg.set_id(k);
        if (!(*it)->sendMessage(k, msg)) {
            report_error(0, "Error while sending message (" + std::to_string(msgtype) +
                         ") to node #" + std::to_string(k) +
                         " (" + (*it)->name + ").");
            return false;
        }
    }
    
    return true;
}


/** Sends a given simple message to all nodes and starts wait-for event (for all nodes).
 The message is simple with no custom data.
 It also resets the timer.
 \param t The time value sent with the message.
 \param msgtype The type of the message sent to nodes.
 \param acktype The type of the ACK message sent from nodes.
 \param pData Pointer to an optional message data block; null pointer (default) if no data. The function will take ownership of this data, therefore the data block should be created dynamically and not released by the caller.
 \return true if successful; false if not (error)
 */
bool GCThread::gc_send_to_all(simtime_t t, OBNSimMsg::SMN2N::MSGTYPE msgtype, OBNSimMsg::N2SMN::MSGTYPE acktype, OBNSimMsg::MSGDATA *pData) {
    if (gc_waitfor_active) {
        // It's an error that wait-for is still active
        report_error(0, "Internal error: wait-for event is active before sending message #");
        return false;
    }
    
    // Send the message to all nodes
    if (!gc_send_to_all(t, msgtype, pData)) {
        return false;
    }
    
    // Set up the wait-for event
    std::fill(gc_waitfor_bits.begin(), gc_waitfor_bits.end(), false);    // Reset all bits
    gc_waitfor_num = _nodes.size();
    gc_waitfor_type = acktype;
    gc_waitfor_active = true;
    
    // Set up timeout if necessary
    gc_timer_reset();
    if (ack_timeout > 0) {
        gc_timer_start(ack_timeout);
    }
    
    return true;
}