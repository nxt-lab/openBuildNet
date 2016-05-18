/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file extmqtt.cpp
 * \brief Tests the OBN External Interface Library using the C API.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <iostream>
#include <obnnode_extmqtt.h>

// Implements the functions to lock and unlock a pointer, in this case are empty
void OBNNodeExtInt::lockPointer(void*) { }
void OBNNodeExtInt::unlockPointer(void*) { }

// Print error message
void print_obn_error(int errcode) {
    std::cout << "OBN error code: " << errcode << '(' << lastErrorMessage() << ").\n";
}

// The node ID
size_t node_id;

// Destroy the node and clean up
void cleanup() {
    std::cout << "Clean up ... ";
    int result = deleteOBNNode(node_id);
    if (result == 0) {
        std::cout << "SUCCEEDED!\n";
    } else {
        std::cout << "FAILED!\n";
        print_obn_error(result);
    }
}

// Print node status
void print_node_status() {
    std::cout << "Node status: ";
    if (nodeIsRunning(node_id) > 0) {
        std::cout << "Running.\n";
    } else if (nodeIsStopped(node_id) > 0) {
        std::cout << "Stopped.\n";
    } else if (nodeIsError(node_id) > 0) {
        std::cout << "Error.\n";
    } else {
        std::cout << "Unknown state.\n";
    }
}

// Utility macros
#define CREATE_INPUT(name,format,container,element,strict) \
std::cout << "Creating input port " << #name << " ... "; \
int input_##name; \
input_##name = createInputPort(node_id, "input_"#name, format, container, element, strict); \
if (input_##name >= 0) { std::cout << "Port ID: " << input_##name << '\n'; } \
else { std::cout << "FAILED!\n"; print_obn_error(input_##name); cleanup(); return 2; }

#define CREATE_OUTPUT(name,format,container,element) \
std::cout << "Creating output port " << #name << " ... "; \
int output_##name; \
output_##name = createOutputPort(node_id, "output_"#name, format, container, element); \
if (output_##name >= 0) { std::cout << "Port ID: " << output_##name << '\n'; } \
else { std::cout << "FAILED!\n"; print_obn_error(output_##name); cleanup(); return 3; }


// Meta function to print a scalar input port
bool print_scalar_input(int portid) {
    // Get info about port
    OBNEI_PortInfo portinfo;
    int result = portInfo(node_id, portid, &portinfo);
    if (result != 0) {
        // Error
        return false;
    }
    // std::cout << "Type: " << portinfo.type << "; Container: " << portinfo.container << "; Element: " << portinfo.element_type << "; Strict: " << portinfo.strict << '\n';

    // Get and print the scalar
    if (portinfo.type != OBNEI_Port_Input || portinfo.container != OBNEI_Container_Scalar) {
        return false;
    }
    switch (portinfo.element_type) {
        case OBNEI_Element_double: {
            std::cout << "Scalar double: ";
            double value;
            while (inputPending(node_id, portid)) {
                result = inputScalarDoubleGet(node_id, portid, &value);
                if (result != 0) {
                    std::cout << "FAILED.\n";
                } else {
                    std::cout << value << '\n';
                }
            }
            break;
        }
            
        default:
            return false;
            break;
    }
    
    return true;
}


// Meta function to print a vector input port
bool print_vector_input(int portid) {
    // Get info about port
    OBNEI_PortInfo portinfo;
    int result = portInfo(node_id, portid, &portinfo);
    if (result != 0) {
        // Error
        return false;
    }
    // std::cout << "Type: " << portinfo.type << "; Container: " << portinfo.container << "; Element: " << portinfo.element_type << "; Strict: " << portinfo.strict << '\n';
    
    // Get and print the value
    if (portinfo.type != OBNEI_Port_Input || portinfo.container != OBNEI_Container_Vector) {
        return false;
    }
    switch (portinfo.element_type) {
        case OBNEI_Element_double: {
            std::cout << "Vector double: ";
            while (inputPending(node_id, portid)) {
                void* Man;
                size_t nelems;
                result = inputVectorDoubleGet(node_id, portid, &Man, NULL, &nelems);
                if (result != 0) {
                    std::cout << "FAILED.\n";
                    break;
                }
                
                double* values = new double[nelems];
                inputVectorDoubleRelease(Man, values);
                for (auto i = 0; i < nelems; ++i) {
                    std::cout << values[i] << ' ';
                }
                std::cout << '\n';
                delete [] values;
            }
            
            break;
        }
            
        default:
            return false;
            break;
    }
    
    return true;
}

// Meta function to print a matrix input port
bool print_matrix_input(int portid) {
    // Get info about port
    OBNEI_PortInfo portinfo;
    int result = portInfo(node_id, portid, &portinfo);
    if (result != 0) {
        // Error
        return false;
    }
    // std::cout << "Type: " << portinfo.type << "; Container: " << portinfo.container << "; Element: " << portinfo.element_type << "; Strict: " << portinfo.strict << '\n';
    
    // Get and print the value
    if (portinfo.type != OBNEI_Port_Input || portinfo.container != OBNEI_Container_Matrix) {
        return false;
    }
    switch (portinfo.element_type) {
        case OBNEI_Element_double: {
            std::cout << "Matrix double: \n";
            while (inputPending(node_id, portid)) {
                void* Man;
                size_t nrows, ncols;
                result = inputMatrixDoubleGet(node_id, portid, &Man, NULL, &nrows, &ncols);
                if (result != 0) {
                    std::cout << "FAILED.\n";
                    break;
                }
                
                double* values = new double[nrows*ncols];
                inputMatrixDoubleRelease(Man, values);
                std::cout << '[';
                for (auto r = 0; r < nrows; ++r) {
                    if (r > 0) {
                        std::cout << '\n';
                    }
                    for (auto c = 0; c < ncols; ++c) {
                        std::cout << values[c*nrows + r] << ' ';
                    }
                }
                std::cout << "]\n";
                delete [] values;
            }
            
            break;
        }
            
        default:
            return false;
            break;
    }
    
    return true;
}



int main() {
    int result;     // result of calling API functions
    
    /* ==== Creating node ==== */
    std::cout << "Create a node with local MQTT server ... ";
    result = createOBNNode("extnode", "testext", nullptr, &node_id);
    if (result == 0) {
        std::cout << "Node ID: " << node_id << '\n';
    } else {
        std::cout << "FAILED!\n";
        print_obn_error(result);
        return 1;
    }
    print_node_status();
    
    
    /* ==== Creating input ports ==== */
    CREATE_INPUT(scalardouble, OBNEI_Format_ProtoBuf, OBNEI_Container_Scalar, OBNEI_Element_double, false)
    //CREATE_INPUT(scalarint32, OBNEI_Format_ProtoBuf, OBNEI_Container_Scalar, OBNEI_Element_int32, false)

    CREATE_INPUT(vectordouble, OBNEI_Format_ProtoBuf, OBNEI_Container_Vector, OBNEI_Element_double, false)
    //CREATE_INPUT(vectorint32, OBNEI_Format_ProtoBuf, OBNEI_Container_Vector, OBNEI_Element_int32, false)

    CREATE_INPUT(matrixdouble, OBNEI_Format_ProtoBuf, OBNEI_Container_Matrix, OBNEI_Element_double, false)
    //CREATE_INPUT(matrixint32, OBNEI_Format_ProtoBuf, OBNEI_Container_Matrix, OBNEI_Element_int32, false)
    
    //CREATE_INPUT(scalardoublestrict, OBNEI_Format_ProtoBuf, OBNEI_Container_Scalar, OBNEI_Element_double, true)
    //CREATE_INPUT(scalarint32strict, OBNEI_Format_ProtoBuf, OBNEI_Container_Scalar, OBNEI_Element_int32, true)
    
    CREATE_INPUT(vectordoublestrict, OBNEI_Format_ProtoBuf, OBNEI_Container_Vector, OBNEI_Element_double, true)
    //CREATE_INPUT(vectorint32strict, OBNEI_Format_ProtoBuf, OBNEI_Container_Vector, OBNEI_Element_int32, true)
    
    //CREATE_INPUT(matrixdoublestrict, OBNEI_Format_ProtoBuf, OBNEI_Container_Matrix, OBNEI_Element_double, true)
    //CREATE_INPUT(matrixint32strict, OBNEI_Format_ProtoBuf, OBNEI_Container_Matrix, OBNEI_Element_int32, true)
    
    
    /* ==== Creating input ports ==== */
    CREATE_OUTPUT(scalardouble, OBNEI_Format_ProtoBuf, OBNEI_Container_Scalar, OBNEI_Element_double)
    //CREATE_OUTPUT(scalarint32, OBNEI_Format_ProtoBuf, OBNEI_Container_Scalar, OBNEI_Element_int32)
    
    CREATE_OUTPUT(vectordouble, OBNEI_Format_ProtoBuf, OBNEI_Container_Vector, OBNEI_Element_double)
    //CREATE_OUTPUT(vectorint32, OBNEI_Format_ProtoBuf, OBNEI_Container_Vector, OBNEI_Element_int32)
    
    CREATE_OUTPUT(matrixdouble, OBNEI_Format_ProtoBuf, OBNEI_Container_Matrix, OBNEI_Element_double)
    //CREATE_OUTPUT(matrixint32, OBNEI_Format_ProtoBuf, OBNEI_Container_Matrix, OBNEI_Element_int32)
    
    
    /* ==== Run the simulation ==== */
    std::cout << "Starting to run simulation ...\n";
    
    double vector_values[] = {27.0, 9.0}; size_t nelems = 2;
    double matrix_values[] = {1.0, 2.0, 3.0, 4.0}; size_t ncols = 2; size_t nrows = 2;
    
    double timeout = 20.0;  // timeout in seconds
    
    unsigned int event_type;
    OBNEI_EventArg event_args;
    double curtime;
    
    while ((result = simRunStep(node_id, timeout, &event_type, &event_args)) == 0) {
        nodeSimulationTime(node_id, 0, &curtime);
        
        // Obtain the event and process it
        switch ((OBNEI_EventType)event_type) {
            case OBNEI_Event_Y:
                print_scalar_input(input_scalardouble);
                print_vector_input(input_vectordouble);
                print_matrix_input(input_matrixdouble);
                print_vector_input(input_vectordoublestrict);
                outputScalarDoubleSet(node_id, output_scalardouble, 21.9);
                outputVectorDoubleSet(node_id, output_vectordouble, vector_values, nelems);
                outputMatrixDoubleSet(node_id, output_matrixdouble, matrix_values, nrows, ncols);
                break;
                
            case OBNEI_Event_INIT:
                std::cout << "At " << curtime << "s: INIT.\n";
                break;
                
            case OBNEI_Event_TERM:
                std::cout << "At " << curtime << "s: TERM.\n";
                break;
                
            default:
                break;
        }
    }
    
    std::cout << '\n';
    switch (result) {
        case 1:
            std::cout << "Simulation timed out.\n";
            break;
            
        case 2:
            std::cout << "Simulation stopped properly.\n";
            break;
            
        case 3:
            std::cout << "Simulation stopped with error.\n";
            std::cout << "Last error: " << lastErrorMessage() << '\n';
            break;
            
        default:
            std::cout << "Other error.\n";
            break;
    }
    

    /* ==== Closing node ==== */
    std::cout << "Close the node ... ";
    result = deleteOBNNode(node_id);
    if (result == 0) {
        std::cout << "SUCCEEDED!\n";
    } else {
        std::cout << "FAILED!\n";
        print_obn_error(result);
        return 1;
    }
    
    return 0;
}