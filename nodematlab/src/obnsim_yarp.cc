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
#include <functional>

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
    stopSimulation();
    
    // Delete all port objects belonging to this node
    for (auto p:_all_ports) {
        delete p.port;
    }
}


/** This method runs the simulation until the next event that requires a callback (e.g. an UPDATE_Y message from the GC), or until the node stops (because the simulation terminates, because of errors...).
 \return 0 if everything is going well and there is an event pending, 1 if timeout (but the simulation won't stop automatically, it's still running), 2 if the simulation has stopped (properly, not because of an error), 3 if the simulation has stopped due to an error (the node's state becomes NODE_ERROR)
 */
int YarpNodeMatlab::runStep(double timeout) {
    if (_node_state == NODE_ERROR) {
        // We can't continue in error state
        reportError("YARPNODE:runStep", "Node is in error state; can't continue simulation; please stop the node (restart it) to clear the error state before continuing.");
        return 3;
    }

    // If there is a pending event, which means the current event has just been processed in Matlab, we must resume the execution of the event object to finish it, then we can continue
    if (_ml_pending_event) {
        assert(_current_node_event);    // It must store a valid event object
        _current_node_event->executePost(this);
        _ml_pending_event = false;      // Clear the current event
    }

    // Run until there is a pending ML event
    while (!_ml_pending_event) {
        switch (_node_state) {
            case NODE_RUNNING:  // Node is running normally, so keep running it until the next event
            case NODE_STARTED:
                // Wait for the next event and process it
                if (timeout <= 0.0) {
                    // Without timeout
                    _current_node_event = _event_queue.wait_and_pop();
                    assert(_current_node_event);
                    _current_node_event->executeMain(this);  // Execute the event
                    
                    // if there is no pending event, we must finish the event's execution now
                    // if there is, we don't need to because it will be finished later when this function is resumed
                    if (!_ml_pending_event) {
                        _current_node_event->executePost(this);
                    }
                } else {
                    // With timeout
                    _current_node_event = _event_queue.wait_and_pop_timeout(timeout);
                    if (_current_node_event) {
                        _current_node_event->executeMain(this);  // Execute the event if not timeout
                        
                        // if there is no pending event, we must finish the event's execution now
                        // if there is, we don't need to because it will be finished later when this function is resumed
                        if (!_ml_pending_event) {
                            _current_node_event->executePost(this);
                        }
                    } else {
                        // If timeout then we return but won't stop
                        onRunTimeout();
                        return 1;
                    }
                }
                
                break;
                
            case NODE_STOPPED:  // Start the simulation from beginning
                // Make sure that the SMN port is opened (but won't connect it)
                if (!openSMNPort()) {
                    // Error
                    _node_state = NODE_ERROR;
                    reportError("YARPNODE:runStep", "Could not open the SMN port. Check the network and the name server.");
                    break;
                }
                
                // Initialize the node's state
                initializeForSimulation();
                
                // Switch to STARTED to wait for INIT message from the SMN
                _node_state = NODE_STARTED;
                break;
                
            default:
                reportError("YARPNODE:runStep", "Internal error: invalid node's state.");
                break;
        }
        
        // At this point, if node state is not RUNNING or STARTED, we must stop it
        if (_node_state != NODE_RUNNING && _node_state != NODE_STARTED) {
            break;
        }
    }
    
    // Return appropriate value depending on the current state
    if (_node_state == NODE_STOPPED) {
        // Stopped (properly)
        return 2;
    } else if (_node_state == NODE_ERROR) {
        // error
        return 3;
    }
    
    // this can only be reached if there is a pending event
    return 0;
}


/* ============ MEX interface ===============*/

// Instance manager
template class mexplus::Session<YarpNodeMatlab>;
template class mexplus::Session<YarpNode::WaitForCondition>;

#define READ_INPUT_SCALAR_HELPER(A,B,C) \
  YarpInput<A,obn_scalar<B>,C> *p = dynamic_cast<YarpInput<A,obn_scalar<B>,C>*>(portinfo.port); \
  if (p) { output.set(0, p->get()); } \
  else { reportError("YARPNODE:readInput", "Internal error: port object type does not match its declared type."); }


// For vectors, regardless of how vectors are stored in Matlab's mxArray vs. in Eigen, we can simply copy the exact number of values from one to another.
template <typename ETYPE>
mxArray* read_input_vector_helper(std::function<mxArray*(int, int)> FUNC, OBNnode::YarpPortBase *port) {
    YarpInput<OBN_PB,obn_vector<ETYPE>,false> *p = dynamic_cast<YarpInput<OBN_PB,obn_vector<ETYPE>,false>*>(port);
    if (p) {
        auto &pv = p->get();
        MxArray ml(FUNC(pv.size(), 1));
        std::copy(pv.data(), pv.data()+pv.size(), ml.getData<ETYPE>());
        return ml.release();
    } else {
        reportError("YARPNODE:readInput", "Internal error: port object type does not match its declared type.");
        return nullptr;
    }
}

// For matrices, the order in which Matlab and Eigen store matrices (column-major or row-major) will affect how data can be copied.  Because both Eigen and Matlab use column-major order by default, we can safely copy the data in memory between them without affecting the data.  For completeness, there are commented code lines that safely transfer data by accessing element-by-element, but this would be slower.
template <typename ETYPE>
mxArray* read_input_matrix_helper(std::function<mxArray*(int, int)> FUNC, OBNnode::YarpPortBase *port) {
    YarpInput<OBN_PB,obn_matrix<ETYPE>,false> *p = dynamic_cast<YarpInput<OBN_PB,obn_matrix<ETYPE>,false>*>(port);
    if (p) {
        auto &pv = p->get();
        auto nr = pv.rows(), nc = pv.cols();
        MxArray ml(FUNC(nr, nc));
        
        // The following lines copy the whole chunk of raw data from Eigen to MEX/Matlab, it's faster but requires that Matlab and Eigen must use the same order type
        std::copy(pv.data(), pv.data()+nr*nc, ml.getData<ETYPE>());
        return ml.release();
        
// // The following lines copy element-by-element: it's safe but slow.
//        for (std::size_t c = 0; c < nc; ++c) {
//            for (std::size_t r = 0; r < nr; ++r) {
//                ml.set(r, c, pv.coeff(r, c));
//            }
//        }
    } else {
        reportError("YARPNODE:readInput", "Internal error: port object type does not match its declared type.");
        return nullptr;
    }
}


#define WRITE_OUTPUT_SCALAR_HELPER(A,B) \
  YarpOutput<A,obn_scalar<B>> *p = dynamic_cast<YarpOutput<A,obn_scalar<B>>*>(portinfo.port); \
  if (p) { *p = input.get<B>(2); } \
  else { reportError("YARPNODE:writeOutput", "Internal error: port object type does not match its declared type."); }


// To convert a vector from mxArray to Eigen, with possibility that their types are different
// - Use MEXPLUS to convert from mxArray to vector<D> of appropriate type
// - Use Eigen::Map to create an Eigen's vector on the data returned by the vector object.
// - Assign the map object to the Eigen vector object (inside the port object).
template <typename ETYPE>
void write_output_vector_helper(const InputArguments &input, OBNnode::YarpPortBase *port) {
    YarpOutput<OBN_PB,obn_vector<ETYPE>> *p = dynamic_cast<YarpOutput<OBN_PB,obn_vector<ETYPE>>*>(port);
    if (p) {
        auto from = MxArray(input.get(2)); // MxArray object
        auto &to = *(*p);
        to = to.Map(from.getData<ETYPE>(), from.size());
        
// // Another way to copy
//        std::vector<ETYPE> v(input.get<std::vector<ETYPE>>(2));
//        *p = (*(*p)).Map(v.data(), v.size());
    } else {
        reportError("YARPNODE:writeOutput", "Internal error: port object type does not match its declared type.");
    }
}

// For matrices, the order in which Matlab and Eigen store matrices (column-major or row-major) will affect how data can be copied.  Because both Eigen and Matlab use column-major order by default, we can safely copy the data in memory between them without affecting the data.  For completeness, there are commented code lines that safely transfer data by accessing element-by-element, but this would be slower.
template <typename ETYPE>
void write_output_matrix_helper(const InputArguments &input, OBNnode::YarpPortBase *port) {
    YarpOutput<OBN_PB,obn_matrix<ETYPE>> *p = dynamic_cast<YarpOutput<OBN_PB,obn_matrix<ETYPE>>*>(port);
    if (p) {
        auto from = MxArray(input.get(2)); // MxArray object
        auto &to = *(*p);
        auto nr = from.rows(), nc = from.cols();
        to = to.Map(from.getData<ETYPE>(), nr, nc);
        
//        // The following lines copy element-by-element: it's safe but slow.
//        to.resize(nr, nc);
//        for (std::size_t c = 0; c < nc; ++c) {
//            for (std::size_t r = 0; r < nr; ++r) {
//                to(r, c) = from.at<ETYPE>(r, c);
//            }
//        }
    } else {
        reportError("YARPNODE:writeOutput", "Internal error: port object type does not match its declared type.");
    }
}

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
                    case YarpNodeMatlab::PortInfo::DOUBLE:
                        output.set(0, read_input_vector_helper<double>(MxArray::Numeric<double>, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::LOGICAL:
                        output.set(0, read_input_vector_helper<bool>(MxArray::Logical, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::INT32:
                        output.set(0, read_input_vector_helper<int32_t>(MxArray::Numeric<int32_t>, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::INT64:
                        output.set(0, read_input_vector_helper<int64_t>(MxArray::Numeric<int64_t>, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::UINT32:
                        output.set(0, read_input_vector_helper<uint32_t>(MxArray::Numeric<uint32_t>, portinfo.port));
                        break;
                
                    case YarpNodeMatlab::PortInfo::UINT64:
                        output.set(0, read_input_vector_helper<uint64_t>(MxArray::Numeric<uint64_t>, portinfo.port));
                        break;
                        
                    default:
                        reportError("YARPNODE:readInput", "Internal error: port's element type is invalid.");
                        break;
                }
                break;
                
            case 'm':
                switch (portinfo.elementType) {
                    case YarpNodeMatlab::PortInfo::DOUBLE:
                        output.set(0, read_input_matrix_helper<double>(MxArray::Numeric<double>, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::LOGICAL:
                        output.set(0, read_input_matrix_helper<bool>(MxArray::Logical, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::INT32:
                        output.set(0, read_input_matrix_helper<int32_t>(MxArray::Numeric<int32_t>, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::INT64:
                        output.set(0, read_input_matrix_helper<int64_t>(MxArray::Numeric<int64_t>, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::UINT32:
                        output.set(0, read_input_matrix_helper<uint32_t>(MxArray::Numeric<uint32_t>, portinfo.port));
                        break;
                        
                    case YarpNodeMatlab::PortInfo::UINT64:
                        output.set(0, read_input_matrix_helper<uint64_t>(MxArray::Numeric<uint64_t>, portinfo.port));
                        break;
                        
                    default:
                        reportError("YARPNODE:readInput", "Internal error: port's element type is invalid.");
                        break;
                }
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
                    case YarpNodeMatlab::PortInfo::DOUBLE:
                        write_output_vector_helper<double>(input, portinfo.port);
                        break;
                        
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
                        
                    case YarpNodeMatlab::PortInfo::INT32:
                        write_output_vector_helper<int32_t>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::INT64:
                        write_output_vector_helper<int64_t>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::UINT32:
                        write_output_vector_helper<uint32_t>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::UINT64:
                        write_output_vector_helper<uint64_t>(input, portinfo.port);
                        break;
                        
                    default:
                        reportError("YARPNODE:writeOutput", "Internal error: port's element type is invalid.");
                        break;
                }
                break;
                
            case 'm':
                switch (portinfo.elementType) {
                    case YarpNodeMatlab::PortInfo::DOUBLE:
                        write_output_matrix_helper<double>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::LOGICAL:
                        write_output_matrix_helper<bool>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::INT32:
                        write_output_matrix_helper<int32_t>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::INT64:
                        write_output_matrix_helper<int64_t>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::UINT32:
                        write_output_matrix_helper<uint32_t>(input, portinfo.port);
                        break;
                        
                    case YarpNodeMatlab::PortInfo::UINT64:
                        write_output_matrix_helper<uint64_t>(input, portinfo.port);
                        break;
                        
                    default:
                        reportError("YARPNODE:writeOutput", "Internal error: port's element type is invalid.");
                        break;
                }
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
    
    // Returns the current simulation time of the node.
    // Args: node object pointer
    // Returns: current simulation time as an integer
    MEX_DEFINE(simTime) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 1);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        output.set(0, ynode->currentSimulationTime());
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
    
    
    // Runs the node's simulation until the next event, or until the node stops or has errors
    // Args: node object pointer, timeout (double, can be <= 0)
    // Returns: status (see YarpNodeMatlab::runStep()), event type (a string), event argument (of an appropriate data type depending on the event type)
    MEX_DEFINE(runStep) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 2);
        OutputArguments output(nlhs, plhs, 3);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));

        // Call node's runStep
        int result = ynode->runStep(input.get<double>(1));
        
        
        // Convert the result to outputs of this MEX function
        output.set(0, result);
        if (ynode->_ml_pending_event) {
            switch (ynode->_ml_current_event.type) {
                case OBNnode::YarpNodeMatlab::MLE_Y:
                    output.set(1, "Y");
                    output.set(2, ynode->_ml_current_event.arg.mask);
                    break;
                    
                case OBNnode::YarpNodeMatlab::MLE_X:
                    output.set(1, "X");
                    output.set(2, ynode->_ml_current_event.arg.mask);   // This mask is set to the current update mask
                    break;
                    
                case OBNnode::YarpNodeMatlab::MLE_INIT:
                    output.set(1, "INIT");
                    output.set(2, 0);
                    break;
                    
                case OBNnode::YarpNodeMatlab::MLE_TERM:
                    output.set(1, "TERM");
                    output.set(2, 0);
                    break;
                    
                default:
                    reportError("YARPNODE:runStep", "Internal error: unrecognized Matlab event type.");
                    break;
            }
        } else {
            output.set(1, "");
            output.set(2, 0);
        }
    }
    
    // Request an irregular future update.
    // This is a blocking call, possibly with a timeout, that waits until it receives the response from the SMN or until a timeout.
    // Args: node object pointer, future time (integer value in the future), update mask of the requested update (int64), timeout (double, can be <= 0)
    // Returns: status of the request: 0 if successful (accepted), -1 if timeout (failed), -2 if request is invalid, >0 if other errors (failed, see OBN documents for details).
    MEX_DEFINE(futureUpdate) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 4);
        OutputArguments output(nlhs, plhs, 1);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        auto *pCond = ynode->requestFutureUpdate(input.get<simtime_t>(1), input.get<updatemask_t>(2), false);
        if (pCond) {
            output.set(0, ynode->resultFutureUpdate(pCond, input.get<double>(3)));
        } else {
            output.set(0, -2);
        }
    }
    
    
    // Request/notify the SMN to stop, then terminate the node's simulation regardless of whether the request was accepted or not. See YarpNodeMatlab::stopSimulation for details.
    // Args: node object pointer
    // Return: none
    MEX_DEFINE(stopSim) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 0);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        ynode->stopSimulation();
    }
    
    // This method requests the SMN/GC to stop the simulation (by sending a request message to the SMN).
    // See YarpNodeMatlab::requestStopSimulation() for details.
    // Args: node object pointer
    // Return: none
    MEX_DEFINE(requestStopSim) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 0);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        ynode->requestStopSimulation();
    }

    
    // Check if the current state of the node is ERROR
    // Args: node object pointer
    // Returns: true/false
    MEX_DEFINE(isNodeErr) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 1);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        output.set(0, ynode->hasError());
    }
    
    // Check if the current state of the node is RUNNING
    // Args: node object pointer
    // Returns: true/false
    MEX_DEFINE(isNodeRunning) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 1);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        output.set(0, ynode->nodeState() == OBNnode::YarpNode::NODE_RUNNING);
    }
    
    // Check if the current state of the node is STOPPED
    // Args: node object pointer
    // Returns: true/false
    MEX_DEFINE(isNodeStopped) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 1);
        OutputArguments output(nlhs, plhs, 1);
        
        YarpNodeMatlab *ynode = Session<YarpNodeMatlab>::get(input.get(0));
        output.set(0, ynode->nodeState() == OBNnode::YarpNode::NODE_STOPPED);
    }
    
    
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
    
    // Returns the maximum ID allowed for an update type.
    // Args: none
    // Returns: an integer
    MEX_DEFINE(maxUpdateID) (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
        InputArguments input(nrhs, prhs, 0);
        OutputArguments output(nlhs, plhs, 1);
        
        output.set(0, OBNsim::MAX_UPDATE_INDEX);
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
