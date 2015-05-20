/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnsim_yarp.cc
 * \brief MEX interface for YarpNode of the openBuildNet simulation framework.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>       // cout
#include <vector>
#include <algorithm>    // std::copy

#include <mex.h>        // Standard MEX interface for C
#include <mexplus.h>    // MEX interface for C++

#include <obnnode.h>    // node.C++ framework
#include <obnsim_yarp.h>

using namespace mexplus;
using namespace OBNnode;
using std::string;

#define YNM_PORT_CLASS_BY_NAME(BASE,PCLS,CTNR,TYPE,...) \
  TYPE=="double"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<double> >(__VA_ARGS__)):(\
    TYPE=="logical"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<bool> >(__VA_ARGS__)):(\
      TYPE=="int32"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int32_t> >(__VA_ARGS__)):(\
        TYPE=="int64"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int64_t> >(__VA_ARGS__)):(\
          TYPE=="uint32"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint32_t> >(__VA_ARGS__)):(\
            TYPE=="uint64"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint64_t> >(__VA_ARGS__)):nullptr)))));

#define YNM_PORT_CLASS_BY_NAME_STRICT(BASE,PCLS,CTNR,TYPE,STRICT,...) \
  TYPE=="double"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<double>,STRICT >(__VA_ARGS__)):(\
    TYPE=="logical"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<bool>,STRICT >(__VA_ARGS__)):(\
      TYPE=="int32"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int32_t>,STRICT >(__VA_ARGS__)):(\
        TYPE=="int64"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<int64_t>,STRICT >(__VA_ARGS__)):(\
          TYPE=="uint32"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint32_t>,STRICT >(__VA_ARGS__)):(\
            TYPE=="uint64"?static_cast<BASE*>(new PCLS< OBN_PB,CTNR<uint64_t>,STRICT >(__VA_ARGS__)):nullptr)))));

#define YNM_PORT_ELEMENT_TYPE_BY_NAME(TYPE) \
  TYPE=="double"?YarpNodeMatlab::PortInfo::DOUBLE:(\
    TYPE=="logical"?YarpNodeMatlab::PortInfo::LOGICAL:(\
      TYPE=="int32"?YarpNodeMatlab::PortInfo::INT32:(\
        TYPE=="int64"?YarpNodeMatlab::PortInfo::INT64:(\
          TYPE=="uint32"?YarpNodeMatlab::PortInfo::UINT32:(\
            TYPE=="uint64"?YarpNodeMatlab::PortInfo::UINT64:YarpNodeMatlab::PortInfo::NONE)))));


/** This is a meta-function for creating all kinds of input ports supported by this class.
 It creates an input port with the specified type, name and configuration, adds it to the node object, then open it.
 \param container A character that specifies the container type; available options are: 's' for scalar, 'v' for dynamic-length vector, 'm' for dynamic-size 2-D matrix, 'b' for binary (raw) data.
 \param element A string specifying the data type of the elements of the container type (except for binary type 'b'); available options are similar to Matlab's types: 'logical', 'int32', 'int64', 'uint32', 'uint64', 'double'
 \param name A valid name of the port in this node.
 \param strict Whether the input port uses strict reading.
 \return The unique ID of the port in this node (which must be non-negative), or a negative integer if there was an error.
 */
int YarpNodeMatlab::createInputPort(char container, const std::string &element, const std::string &name, bool strict) {
    YarpNodeMatlab::PortInfo portinfo;
    portinfo.type = YarpNodeMatlab::PortInfo::INPUTPORT;
    switch (container) {
        case 's':
        case 'S':
            if (strict) {
                //portinfo.port = YNM_PORT_CLASS_BY_NAME_STRICT(YarpPortBase,YarpInput,obn_scalar,element,true,name);
                portinfo.port = nullptr;
            } else {
                portinfo.port = YNM_PORT_CLASS_BY_NAME_STRICT(YarpPortBase,YarpInput,obn_scalar,element,false,name);
            }
            portinfo.container = 's';
            break;

        case 'v':
        case 'V':
            if (strict) {
                //portinfo.port = YNM_PORT_CLASS_BY_NAME_STRICT(YarpPortBase,YarpInput,obn_vector,element,true,name);
                portinfo.port = nullptr;
            } else {
                portinfo.port = YNM_PORT_CLASS_BY_NAME_STRICT(YarpPortBase,YarpInput,obn_vector,element,false,name);
            }
            portinfo.container = 'v';
            break;
            
        case 'm':
        case 'M':
            reportError("YARPNODE:createOutputPort", "Matrix type is currently unsupported.");
            return -1;
            if (strict) {
                //port = YNM_PORT_CLASS_BY_NAME_STRICT(YarpPortBase,YarpInput,obn_matrix,element,true,name);
                portinfo.port = nullptr;
            } else {
                portinfo.port = YNM_PORT_CLASS_BY_NAME_STRICT(YarpPortBase,YarpInput,obn_matrix,element,false,name);
            }
            portinfo.container = 'm';
            break;
            
        case 'b':
        case 'B':
            if (strict) {
                //portinfo.port = new YarpInput<OBN_BIN,bool,true>(name);
                portinfo.port = nullptr;
            } else {
                portinfo.port = new YarpInput<OBN_BIN,bool,false>(name);
            }
            portinfo.container = 'b';
            break;
            
        default:
            reportError("YARPNODE:createInputPort", "Unknown container type.");
            return -1;
    }
    
    if (!portinfo.port) {
        // port is nullptr => error
        reportError("YARPNODE:createInputPort", "Unknown element type.");
        return -2;
    }
    
    // Add the port to the node
    bool result = addInput(portinfo.port);
    if (!result) {
        // failed to add to node
        delete portinfo.port;
        reportError("YARPNODE:createInputPort", "Could not add new port to node.");
        return -3;
    }
    
    auto id = _all_ports.size();
    portinfo.elementType = YNM_PORT_ELEMENT_TYPE_BY_NAME(element);
    portinfo.strict = strict;
    _all_ports.push_back(portinfo);
    return id;
}

/** \brief Meta-function for creating all kinds of output ports supported by this class. */
int YarpNodeMatlab::createOutputPort(char container, const std::string &element, const std::string &name) {
    YarpOutputPortBase *port;
    YarpNodeMatlab::PortInfo portinfo;
    portinfo.type = YarpNodeMatlab::PortInfo::OUTPUTPORT;
    switch (container) {
        case 's':
        case 'S':
            port = YNM_PORT_CLASS_BY_NAME(YarpOutputPortBase,YarpOutput,obn_scalar,element,name);
            portinfo.container = 's';
            break;
            
        case 'v':
        case 'V':
            port = YNM_PORT_CLASS_BY_NAME(YarpOutputPortBase,YarpOutput,obn_vector,element,name);
            portinfo.container = 'v';
            break;
            
        case 'm':
        case 'M':
            reportError("YARPNODE:createOutputPort", "Matrix type is currently unsupported.");
            return -1;
            port = YNM_PORT_CLASS_BY_NAME(YarpOutputPortBase,YarpOutput,obn_matrix,element,name);
            portinfo.container = 'm';
            break;
            
        case 'b':
        case 'B':
            port = new YarpOutput<OBN_BIN,bool>(name);
            portinfo.container = 'b';
            break;
            
        default:
            reportError("YARPNODE:createOutputPort", "Unknown container type.");
            return -1;
    }
    
    if (!port) {
        // port is nullptr => error
        reportError("YARPNODE:createOutputPort", "Unknown element type.");
        return -2;
    }
    
    // Add the port to the node
    bool result = addOutput(port);
    if (!result) {
        // failed to add to node
        delete port;
        reportError("YARPNODE:createOutputPort", "Could not add new port to node.");
        return -3;
    }
    
    auto id = _all_ports.size();
    portinfo.port = port;
    portinfo.elementType = YNM_PORT_ELEMENT_TYPE_BY_NAME(element);
    _all_ports.push_back(portinfo);
    return id;
}


YarpNodeMatlab::~YarpNodeMatlab() {
    // If the node is still running, we need to stop it first
    stopRunningNode();
    
    // Delete all port objects belonging to this node
    for (auto p:_all_ports) {
        delete p.port;
    }
}

/** This method stops the current simulation if it is running.
 If a simulation is on-going, it sets the internal state of the node to stop the simulation, and notify the SMN/GC to stop.
 */
void YarpNodeMatlab::stopRunningNode() {
    
}


/** This method runs the simulation until the next event that requires a callback (e.g. an UPDATE_Y message from the GC), or until the node stops (because the simulation terminates, because of errors...).
 \return 0 if everything is going well, 1 if timeout (but the simulation won't stop automatically), 2 if the simulation has stopped (for any reason).
 */
int YarpNodeMatlab::runStep(double timeout) {
    return 0;
}


/* ============ MEX interface ===============*/

// Instance manager
//template class mexplus::Session<YPort>;
template class mexplus::Session<YarpNodeMatlab>;

#define READ_INPUT_SCALAR_HELPER(A,B,C) \
  YarpInput<A,obn_scalar<B>,C> *p = dynamic_cast<YarpInput<A,obn_scalar<B>,C>*>(portinfo.port); \
  if (p) { output.set(0, p->get()); } \
  else { reportError("YARPNODE:readInput", "Internal error: port object type does not match its declared type."); }

#define READ_INPUT_VECTOR_HELPER(ETYPE,FUNC) \
  YarpInput<OBN_PB,obn_vector<ETYPE>,false> *p = dynamic_cast<YarpInput<OBN_PB,obn_vector<ETYPE>,false>*>(portinfo.port); \
  if (p) { \
    auto &pv = p->get(); \
    MxArray ml(MxArray::FUNC(pv.size())); \
    std::copy(pv.data(), pv.data()+pv.size(), ml.getData<ETYPE>()); \
    output.set(0, ml.release()); \
  } \
  else { reportError("YARPNODE:readInput", "Internal error: port object type does not match its declared type."); }


#define WRITE_OUTPUT_SCALAR_HELPER(A,B) \
  YarpOutput<A,obn_scalar<B>> *p = dynamic_cast<YarpOutput<A,obn_scalar<B>>*>(portinfo.port); \
  if (p) { *p = input.get<B>(2); } \
  else { reportError("YARPNODE:writeOutput", "Internal error: port object type does not match its declared type."); }


// To convert a vector from mxArray to Eigen, with possibility that their types are different
// - Use MEXPLUS to convert from mxArray to vector<D> of appropriate type
// - Use Eigen::Map to create an Eigen's vector on the data returned by the vector object.
// - Assign the map object to the Eigen vector object (inside the port object).
#define WRITE_OUTPUT_VECTOR_HELPER(ETYPE) \
  YarpOutput<OBN_PB,obn_vector<ETYPE>> *p = dynamic_cast<YarpOutput<OBN_PB,obn_vector<ETYPE>>*>(portinfo.port); \
  if (p) { \
    std::vector<ETYPE> v(input.get<std::vector<ETYPE>>(2)); \
    *p = (*(*p)).Map(v.data(), v.size()); \
  } \
  else { reportError("YARPNODE:writeOutput", "Internal error: port object type does not match its declared type."); }

namespace {
    /* === Port interface === */

    // Read the current value of a non-strict physical input port
    // Args: node object pointer, port's ID
    // Returns: value in an appropriate Matlab's type
    // This function will return an error if the given port is not a non-strict physical input port
    // If the port contains binary data, the function will return a string containing the binary data.
    MEX_DEFINE(readInput) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 1);

        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        unsigned int id = input.get<unsigned int>(1);
        if (id >= ynode->_all_ports.size()) {
            reportError("YARPNODE:readInput", "Invalid port ID to read from.");
            return;
        }

        // Obtain the port
        YarpNodeMatlab::PortInfo portinfo = ynode->_all_ports[id];
        if (portinfo.type != YarpNodeMatlab::PortInfo::INPUTPORT || portinfo.strict) {
            reportError("YARPNODE:readInput", "Given port is not a non-strict physical input.");
            return;
        }
        
        // Query its value based on its type, casting to the appropriate type
        switch (portinfo.container) {
            case 's':
                switch (portinfo.elementType) {
                    case YarpNodeMatlab::PortInfo::DOUBLE: {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, double, false)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::LOGICAL: {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, bool, false)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT32: {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, int32_t, false)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT64: {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, int64_t, false)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT32: {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, uint32_t, false)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT64: {
                        READ_INPUT_SCALAR_HELPER(OBN_PB, uint64_t, false)
                        break;
                    }
                        
                    default:
                        reportError("YARPNODE:readInput", "Internal error: port's element type is invalid.");
                        break;
                }
                break;
                
            case 'v':
                switch (portinfo.elementType) {
                    case YarpNodeMatlab::PortInfo::DOUBLE: {
                        READ_INPUT_VECTOR_HELPER(double,Numeric<double>)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::LOGICAL: {
                        READ_INPUT_VECTOR_HELPER(bool,Logical)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT32: {
                        READ_INPUT_VECTOR_HELPER(int32_t,Numeric<int32_t>)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT64: {
                        READ_INPUT_VECTOR_HELPER(int64_t,Numeric<int64_t>)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT32: {
                        READ_INPUT_VECTOR_HELPER(uint32_t,Numeric<uint32_t>)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT64: {
                        READ_INPUT_VECTOR_HELPER(uint64_t,Numeric<uint64_t>)
                        break;
                    }
                        
                    default:
                        reportError("YARPNODE:readInput", "Internal error: port's element type is invalid.");
                        break;
                }
                break;
                
            case 'm':
                reportError("YARPNODE:readInput", "Matrix type is not yet supported.");
                break;
                
            case 'b': {
                // A binary string is read, the element type is ignored
                YarpInput<OBN_BIN,bool,false> *p = dynamic_cast<YarpInput<OBN_BIN,bool,false>*>(portinfo.port);
                if (p) {
                    output.set(0, p->get());
                } else {
                    reportError("YARPNODE:readInput", "Internal error: port object type does not match its declared type.");
                }
                break;
            }
                
            default:    // This should never happen
                reportError("YARPNODE:readInput", "Internal error: port's container type is invalid.");
                break;
        }
    }

    // Set the value of a physical output port, but does not send it immediately.
    // Usually the value will be sent out at the end of the event callback (UPDATEY).
    // Args: node object pointer, port's ID, value (in appropriate Matlab type)
    // Returns: none
    // This function will return an error if the given port is not a physical output port, or if the given value cannot be converted to the port's type (e.g. write a string to a numeric port).
    // If the port contains binary data, the value must be a string containing the binary data.
    MEX_DEFINE(writeOutput) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 3);
        OutputArguments output(nlhs, plhs, 0);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        unsigned int id = input.get<unsigned int>(1);
        if (id >= ynode->_all_ports.size()) {
            reportError("YARPNODE:writeOutput", "Invalid port ID to write to.");
            return;
        }
        
        // Obtain the port
        YarpNodeMatlab::PortInfo portinfo = ynode->_all_ports[id];
        if (portinfo.type != YarpNodeMatlab::PortInfo::OUTPUTPORT) {
            reportError("YARPNODE:writeOutput", "Given port is not a physical output.");
            return;
        }
        
        // std::cout << portinfo.container << " of " << portinfo.elementType << std::endl;
        
        // Set its value based on its type, trying to convert from the given Matlab type
        switch (portinfo.container) {
            case 's':
                switch (portinfo.elementType) {
                    case YarpNodeMatlab::PortInfo::DOUBLE: {
                        WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, double)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::LOGICAL: {
                        WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, bool)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT32: {
                        WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, int32_t)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT64: {
                        WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, int64_t)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT32: {
                        WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, uint32_t)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT64: {
                        WRITE_OUTPUT_SCALAR_HELPER(OBN_PB, uint64_t)
                        break;
                    }
                        
                    default:
                        reportError("YARPNODE:writeOutput", "Internal error: port's element type is invalid.");
                        break;
                }
                break;
                
            case 'v':
                switch (portinfo.elementType) {
                    case YarpNodeMatlab::PortInfo::DOUBLE: {
                        WRITE_OUTPUT_VECTOR_HELPER(double)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::LOGICAL: {
                        // This case is special because vector<bool> does not have data()
                        YarpOutput<OBN_PB,obn_vector<bool>> *p = dynamic_cast<YarpOutput<OBN_PB,obn_vector<bool>>*>(portinfo.port);
                        if (p) {
                            std::vector<bool> v(input.get<std::vector<bool>>(2));
                            (*(*p)).resize(v.size());
                            // Copy element-by-element
                            for (std::size_t i = 0; i < v.size(); ++i) {
                                (*(*p))(i) = v[i];
                            }
                        } else {
                            reportError("YARPNODE:writeOutput", "Internal error: port object type does not match its declared type.");
                        }
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT32: {
                        WRITE_OUTPUT_VECTOR_HELPER(int32_t)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::INT64: {
                        WRITE_OUTPUT_VECTOR_HELPER(int64_t)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT32: {
                        WRITE_OUTPUT_VECTOR_HELPER(uint32_t)
                        break;
                    }
                        
                    case YarpNodeMatlab::PortInfo::UINT64: {
                        WRITE_OUTPUT_VECTOR_HELPER(uint64_t)
                        break;
                    }
                        
                    default:
                        reportError("YARPNODE:writeOutput", "Internal error: port's element type is invalid.");
                        break;
                }
                break;
                
            case 'm':
                reportError("YARPNODE:writeOutput", "Matrix type is not yet supported.");
                break;
                
            case 'b': {
                // A binary string is written, the element type is ignored
                YarpOutput<OBN_BIN,bool> *p = dynamic_cast<YarpOutput<OBN_BIN,bool>*>(portinfo.port);
                if (p) {
                    p->message(input.get<std::string>(2));
                } else {
                    reportError("YARPNODE:writeOutput", "Internal error: binary port object type does not match its declared type.");
                }
                break;
            }
                
            default:    // This should never happen
                reportError("YARPNODE:writeOutput", "Internal error: port's container type is invalid.");
                break;
        }
    }
        
    // Request an output port to send its current value/message immediatley and wait until it can be sent.
    // Note that this function does not accept a value to be sent; instead the value/message of the port is set by another function.
    // Args: node object pointer, port's ID
    // Returns: none
    // This function will return an error if the given port is not a physical output port.
    MEX_DEFINE(sendSync) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 0);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        unsigned int id = input.get<unsigned int>(1);
        if (id >= ynode->_all_ports.size()) {
            reportError("YARPNODE:sendSync", "Invalid port ID.");
            return;
        }
        
        // Obtain the port
        YarpNodeMatlab::PortInfo portinfo = ynode->_all_ports[id];
        if (portinfo.type != YarpNodeMatlab::PortInfo::OUTPUTPORT) {
            reportError("YARPNODE:sendSync", "Given port is not a physical output.");
            return;
        }
        
        // Cast the pointer to an output port object and send
        YarpOutputPortBase *p = dynamic_cast<YarpOutputPortBase*>(portinfo.port);
        if (p) {
            p->sendSync();
        } else {
            reportError("YARPNODE:sendSync", "Internal error: port object type does not match its declared type.");
        }
    }

//    // Get full Yarp name of a port
//    MEX_DEFINE(yarpName) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 2);
//        OutputArguments output(nlhs, plhs, 1);
//        
//        YarpNode *ynode = Session<YarpNode>::get(input.get(0));
//        YarpNode::portPtr yport = ynode->portObject(input.get<unsigned int>(1));
//        if (yport != nullptr) {
//            output.set(0, std::string(yport->getName()));
//        }
//        else {
//            mexErrMsgIdAndTxt("MATLAB:myarp:enableCallback", "Port doesn't exist.");
//        }
//    }
//    
//    
//    // Set port to use callback
//    MEX_DEFINE(enableCallback) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 2);
//        OutputArguments output(nlhs, plhs, 0);
//        
//        YarpNode *ynode = Session<YarpNode>::get(input.get(0));
//        YarpNode::portPtr yport = ynode->portObject(input.get<unsigned int>(1));
//        if (yport != nullptr) {
//            yport->useCallback();
//        }
//        else {
//            mexErrMsgIdAndTxt("MATLAB:myarp:enableCallback", "Port doesn't exist.");
//        }
//    }
//    
//    // Disable callback of a port
//    MEX_DEFINE(disableCallback) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 2);
//        OutputArguments output(nlhs, plhs, 0);
//        
//        YarpNode *ynode = Session<YarpNode>::get(input.get(0));
//        YarpNode::portPtr yport = ynode->portObject(input.get<unsigned int>(1));
//        if (yport != nullptr) {
//            yport->disableCallback();
//        }
//        else {
//            mexErrMsgIdAndTxt("MATLAB:myarp:disableCallback", "Port doesn't exist.");
//        }
//    }
//
//    MEX_DEFINE(sendString) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 3);
//        OutputArguments output(nlhs, plhs, 0);
//        
//        YarpNode *ynode = Session<YarpNode>::get(input.get(0));
//        YarpNode::portPtr yport = ynode->portObject(input.get<unsigned int>(1));
//        if (yport != nullptr) {
//            yport->sendString(input.get<string>(2));
//        }
//        else {
//            mexErrMsgIdAndTxt("sendString", "Port doesn't exist.");
//        }
//    }
//    
//    MEX_DEFINE(readString) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 3);
//        OutputArguments output(nlhs, plhs, 1);
//        
//        YarpNode *ynode = Session<YarpNode>::get(input.get(0));
//        YarpNode::portPtr yport = ynode->portObject(input.get<unsigned int>(1));
//        if (yport != nullptr) {
//            output.set(0, yport->readString(input.get<bool>(2)));;
//        }
//        else {
//            mexErrMsgIdAndTxt("sendString", "Port doesn't exist.");
//        }
//    }
//
//    
////    // Is there a message pending at a port?
////    MEX_DEFINE(isPendingMessage) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
////        InputArguments input(nrhs, prhs, 1);
////        OutputArguments output(nlhs, plhs, 1);
////        
////        YPort *yport = Session<YPort>::get(input.get(0));
////        output.set(0, yport->hasMessage());
////    }
//    
//    // Get the ID of a port by its name
//    // Returns ID of port; 0 if port doesn't exist
//    MEX_DEFINE(portID) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 2);
//        OutputArguments output(nlhs, plhs, 1);
//        
//        YarpNode *ynode = Session<YarpNode>::get(input.get(0));
//        output.set(0, ynode->portID(input.get<string>(1)));
//    }
    


    /* === YarpNode interface === */
    
    // Defines MEX API for wait
    // Block until one of the ports signals ready
    // Return the event type and ID of the port
//    MEX_DEFINE(wait) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 1);
//        OutputArguments output(nlhs, plhs, 2);
//        
//        YarpNode *m = Session<YarpNode>::get(input.get(0));
//        YarpNode::Event ev = m->wait();
//        
//        output.set(0, ev.type);
//        output.set(1, ev.index);
//    }
//    
//    // Defines MEX API for waitTimeout
//    // Block until one of the ports signals ready or timeout
//    // Return [true if there is event, event type, the ID of the port]
//    MEX_DEFINE(waitTimeout) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 2);
//        OutputArguments output(nlhs, plhs, 3);
//        
//        YarpNode::Event ev;
//        YarpNode *m = Session<YarpNode>::get(input.get(0));
//        
//        if (m->waitTimeout(input.get<double>(1), ev)) {
//            output.set(0, true);
//            output.set(1, ev.type);
//            output.set(2, ev.index);
//        }
//        else {
//            output.set(0, false);
//            output.set(1, 0);
//            output.set(2, 0);
//        }
//    }
//    
//    // Defines MEX API for check
//    // Similar to waitTimeout() but does not block; if there is no event pending, it will return immediately.
//    // Return [true if there is event, event type, the ID of the port]
//    MEX_DEFINE(check) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
//        InputArguments input(nrhs, prhs, 1);
//        OutputArguments output(nlhs, plhs, 3);
//        
//        YarpNode::Event ev;
//        YarpNode *m = Session<YarpNode>::get(input.get(0));
//        
//        if (m->checkEvent(ev)) {
//            output.set(0, true);
//            output.set(1, ev.type);
//            output.set(2, ev.index);
//        }
//        else {
//            output.set(0, false);
//            output.set(1, 0);
//            output.set(2, 0);
//        }
//    }
    
    // Create a new node object
    // Args: nodeName, workspace
    MEX_DEFINE(nodeNew) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 1);
        
        // Create a node
        YarpNodeMatlab *y = new YarpNodeMatlab(input.get<string>(0), input.get<string>(1));
        output.set(0, Session<YarpNodeMatlab>::create(y));
    }
    
    // Delete a node object
    // Args: node's pointer
    MEX_DEFINE(nodeDelete) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 0);
        
        Session<YarpNodeMatlab>::destroy(input.get(0));
    }

    // Create a new physical input port managed by this node
    // Arguments: node object pointer, container type (s,v,m,b), element type (logical,double,int32,int64,uint32,uint64), port's name
    // An optional name-value pair of "strict" value.
    // Returns id; or negative number if error.
    // id is an integer starting from 0
    MEX_DEFINE(createInput) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 4, 1, "strict");
        OutputArguments output(nlhs, plhs, 1);
        
        // Create port
        int id = Session<YarpNodeMatlab>::get(input.get(0))->createInputPort(input.get<char>(1), input.get<string>(2), input.get<string>(3), input.get<bool>("strict", false));
        
        output.set(0, id);
    }

    // Create a new physical output port managed by this node
    // Arguments: node object pointer, container type (s,v,m,b), element type (logical,double,int32,int64,uint32,uint64), port's name
    // Returns id; or a negative number if error.
    // id is an integer starting from 0
    MEX_DEFINE(createOutput) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 4, 0);
        OutputArguments output(nlhs, plhs, 1);
        
        // Create port
        int id = Session<YarpNodeMatlab>::get(input.get(0))->createOutputPort(input.get<char>(1), input.get<string>(2), input.get<string>(3));
        
        output.set(0, id);
    }
    
    // Return information about a port.
    // Arguments: node object pointer, port's ID
    // Returns: type, container, elementType, strict
    MEX_DEFINE(portInfo) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 4);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        unsigned int id = input.get<unsigned int>(1);
        if (id >= ynode->_all_ports.size()) {
            reportError("YARPNODE:portInfo", "Invalid port ID.");
            return;
        }
        
        // Obtain the port
        YarpNodeMatlab::PortInfo portinfo = ynode->_all_ports[id];
        
        // Return the information
        output.set(0,
                   portinfo.type==YarpNodeMatlab::PortInfo::INPUTPORT?'i':
                   (portinfo.type==YarpNodeMatlab::PortInfo::OUTPUTPORT?'o':'d'));
        output.set(1, portinfo.container);
        switch (portinfo.elementType) {
            case YarpNodeMatlab::PortInfo::DOUBLE:
                output.set(2, "double");
                break;
            case YarpNodeMatlab::PortInfo::INT32:
                output.set(2, "int32");
                break;
            case YarpNodeMatlab::PortInfo::INT64:
                output.set(2, "int64");
                break;
            case YarpNodeMatlab::PortInfo::UINT32:
                output.set(2, "uint32");
                break;
            case YarpNodeMatlab::PortInfo::UINT64:
                output.set(2, "uint64");
                break;
            case YarpNodeMatlab::PortInfo::LOGICAL:
                output.set(2, "logical");
                break;
            default:
                output.set(2, "");
                break;
        }
        output.set(3, portinfo.strict);
    }

    /* === General Yarp interface === */
    
    // Connect one port to another port
    MEX_DEFINE(connect) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 1);
        
        output.set(0, yarp::os::Network::connect(input.get<string>(0), input.get<string>(1)));
    }

    // Extended version of connect, where we can specify carrier, quiet, and possibly other options
    // Inputs: source port, dest port, carrier, quiet, and maybe other options (may add later)
    MEX_DEFINE(connectExt) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 4);
        OutputArguments output(nlhs, plhs, 1);
        
        output.set(0, yarp::os::Network::connect(input.get<string>(0), input.get<string>(1), input.get<string>(2), input.get<bool>(3)));
    }

    
    // Disconnect a connection between two ports
    MEX_DEFINE(disconnect) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 1);
        
        output.set(0, yarp::os::Network::disconnect(input.get<string>(0), input.get<string>(1)));
    }
    
    // Does a connection exist?
    MEX_DEFINE(isConnected) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 1);
        
        output.set(0, yarp::os::Network::isConnected(input.get<string>(0), input.get<string>(1)));
    }

} // namespace

MEX_DISPATCH // Don't forget to add this if MEX_DEFINE() is used.
