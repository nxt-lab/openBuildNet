/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnsim_yarp.h
 * \brief MEX interface for YarpNode of the openBuildNet simulation framework.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSIM_YARP_H_
#define OBNSIM_YARP_H_


#include <cstdio>       // printf
#include <vector>

#include <obnnode.h>    // node.C++ framework


/* IMPORTANT NOTE:
 *
 * Because of the tight inter-dependency between a node object and its port objects, and the way Matlab's MEX works, care must be taken to make sure that when a node is destroyed, the port objects are also be destroyed and not orphaned.
 * For this reason, unlike the node.C++ framework, the node object will manage all its port objects explicitly, i.e. port objects are not managed by Matlab at all.
 * The node object will provide an interface for accessing the port objects managed by it, indirectly through some kind of IDs, not via pointers.
 */

/// Report an error and (usually) terminate.
void reportError(const char* msgID, const char* msg) {
    mexErrMsgIdAndTxt(msgID, msg);
}


/** The main node class for Matlab. */
class YarpNodeMatlab: public OBNnode::YarpNode {
public:
    
    /** Structure containing info about a port in this node. */
    struct PortInfo {
        OBNnode::YarpPortBase *port;
        enum { INPUTPORT, OUTPUTPORT, DATAPORT } type;
        char container;     ///< 's', 'v', 'm', 'b'
        enum { NONE, DOUBLE, LOGICAL, INT32, UINT32, INT64, UINT64 } elementType;
        bool strict;    ///< Only for input ports
    };
    
    /** Vector of all port objects belonging to this node, which are explicitly managed by the node object. */
    std::vector<PortInfo> _all_ports;
    
    /** \brief Meta-function for creating all kinds of input ports supported by this class. */
    int createInputPort(char container, const std::string &element, const std::string &name, bool strict);
    
    /** \brief Meta-function for creating all kinds of output ports supported by this class. */
    int createOutputPort(char container, const std::string &element, const std::string &name);
    
    YarpNodeMatlab(const std::string& name, const std::string& ws = ""): YarpNode(name, ws) {
    }
    
    virtual ~YarpNodeMatlab();
    

    // === Events between the C++ node and Matlab ===
    
    /** Type to pass arguments of an event */
    union MLEventArg {
        OBNnode::updatemask_t mask;
    };
    
    /** The event type */
    enum MLEventType {
        MLE_INIT = OBNSimMsg::SMN2N_MSGTYPE_SIM_INIT,
        MLE_UPDATEY = OBNSimMsg::SMN2N_MSGTYPE_SIM_Y,
        MLE_UPDATEX = OBNSimMsg::SMN2N_MSGTYPE_SIM_X,
        MLE_TERM = OBNSimMsg::SMN2N_MSGTYPE_SIM_TERM
    };

    /** The current event (returned by runStep) */
    struct {
        MLEventType type;
        MLEventArg arg;
    } _current_event;
    
    /** \brief Run the node until it stops or until a callback event. */
    int runStep(double timeout);

    void stopRunningNode();
private:

};


#endif /* OBNSIM_YARP_H_ */