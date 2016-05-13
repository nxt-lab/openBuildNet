/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_ext.h
 * \brief Generic external interface, regardless of the communication network, of the openBuildNet simulation framework.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_EXT_H_
#define OBNNODE_EXT_H_

#include <vector>
#include <functional>

/** Types and functions the external interface can use. */
#ifdef __cplusplus
extern "C" {
#endif
    
    /** The update mask type, see obnsim_basic.h for the definition. That should match the definition here. */
    typedef uint64_t OBNUpdateMask;
    
    /** The event type */
    enum OBNEI_EventType {
        OBNEI_INIT = 0,         // Init of simulation
        OBNEI_Y = 1,            // Update Y
        OBNEI_X = 2,            // Update X
        OBNEI_TERM = 3,         // Termination of simulation
        OBNEI_RCV = 4           // A port has received a message
    };

    /** Type to pass arguments of an event */
    union OBNEI_EventArg {
        OBNUpdateMask mask;     // Update mask (see above)
        size_t index;     // Index, equivalent to uint32_t
    };
    
    
    /* === Node interface === */
    
    // Create a new node object, given nodeName, workspace, and optional server address.
    // Returns 0 if successful; >0 if node already exists; <0 if error.
    // id stores the ID of the new node.
    int createOBNNode(const char* name, const char* workspace, const char* server, size_t* id);
    
    // Delete a node, given its ID
    // Returns 0 if successful; <0 if node doesn't exist.
    int deleteOBNNode(size_t id);
    
    
    
    /* === Port interface === */
    
    /** Port type. */
    enum OBNEI_PortType {
        OBNEI_Port_Input = 0,
        OBNEI_Port_Output = 1,
        OBNEI_Port_Data = 2
    };
    
    /** Container type. */
    enum OBNEI_ContainerType {
        OBNEI_Container_Scalar = 0,
        OBNEI_Container_Vector = 1,
        OBNEI_Container_Matrix = 2,
        OBNEI_Container_Binary = 3      // Raw bytes
    };
    
    /** Element type. */
    enum OBNEI_ElementType {
        OBNEI_Element_logical = 0,
        OBNEI_Element_double = 1,
        OBNEI_Element_int32 = 2,
        OBNEI_Element_int64 = 3,
        OBNEI_Element_uint32 = 4,
        OBNEI_Element_uint64 = 5
    };
    
    /** Format of messages between ports. */
    enum OBNEI_FormatType {
        OBNEI_Format_ProtoBuf = 0      // The default
    };
    
    /** Structure containing information about a port. */
    struct OBNEI_PortInfo {
        OBNEI_PortType type;
        OBNEI_ContainerType container;
        OBNEI_ElementType element_type;
        bool strict;
    };
    
    // Create a new input port on a node
    // Arguments: node ID, port's name, format type, container type, element type, strict or not
    // Returns port's id; or negative number if error.
    // id is an integer starting from 0.
    int createInputPort(size_t id,
                        const char* name,
                        OBNEI_FormatType format,
                        OBNEI_ContainerType container,
                        OBNEI_ElementType element,
                        bool strict);
    
    
    // Create a new output port on a node
    // Arguments: node ID, port's name, format type, container type, element type
    // Returns port's id; or negative number if error.
    // id is an integer starting from 0.
    int createOutputPort(size_t id,
                         const char* name,
                         OBNEI_FormatType format,
                         OBNEI_ContainerType container,
                         OBNEI_ElementType element);
    
    
    // Synchronous sending: request an output port to send its current value/message immediately and wait until it can be sent.
    // Note that this function does not accept a value to be sent; instead the value/message of the port is set by another function.
    // Args: node ID, port's ID
    // Returns: zero if successful
    // This function will return an error if the given port is not a physical output port.
    int outputSendSync(size_t nodeid, size_t portid);
    
    // Is there a value pending at an input port?
    // Args: node ID, port's ID
    // Returns: true if >0, false if =0, error if <0.
    int inputPending(size_t nodeid, size_t portid);
    
    // Returns information about a port.
    // Arguments: node ID, port's ID, pointer to an OBNEI_PortInfo structure to receive info
    // Returns: 0 if successful.
    int portInfo(size_t nodeid, size_t portid, OBNEI_PortInfo* pInfo);
    
    
    /** These functions read the current value of a non-strict scalar input port, or pop the top/front value of a strict scalar input port.
     Args: node ID, port's ID, pointer to scalar variable to receive the value.
     Returns: 0 if successful; <0 if error; >0 if no value actually read (e.g., no pending value on a strict port).
     For strict ports, if there is no value pending, the receiving variable won't be changed and the function will return 1.
     */
    int inputScalarDoubleGet(size_t nodeid, size_t portid, double* pval);      // Float64
    int inputScalarBoolGet(size_t nodeid, size_t portid, bool* pval);          // C++ bool (1 byte)
    int inputScalarInt32Get(size_t nodeid, size_t portid, int32_t* pval);      // Int32
    int inputScalarInt64Get(size_t nodeid, size_t portid, int64_t* pval);      // Int64
    int inputScalarUInt32Get(size_t nodeid, size_t portid, uint32_t* pval);    // UInt32
    int inputScalarUInt64Get(size_t nodeid, size_t portid, uint64_t* pval);    // UInt64
    
    
    /** These functions read (or pop) the value from a non-strict (or strict) vector/matrix input port.
     For each port, there are two functions: *Get and *Release.
     - The *Get(nodeid, portid, void** pMan, <elem-type>** pVals, size_t* nrows, size_t* ncols) [for vector version, there is no ncols] gets the array of values and put its pointer to pVals (if pVals = NULL this won't be done) and also returns its dimensions (nrows, ncols) as well as a management object in pMan.  It is critical that pMan is received and used later on because it will be used to release the memory used in C.
     - *Release(void* pMan, <elem-type>* pBuf) copies the values to buffer pBuf if pBuf is not nill, then releases the management object.  When a port is read by *Get, it may allocate temporary memory and may be locked (so that new incoming messages will not override the current value).  It's CRITICALLY IMPORTANT to call *Release on the returned management object to release the memory and the lock on the port.  The copying is optional, only if pBuf is valid, which must be allocated by the external language.  This can be used instead of copying the data from pVals to the buffer in the external language (e.g., if pVals in *Get is NULL or ignored).
     
     There are two ways to use the returned values:
     - The safest way is to copy the values to a vector / matrix managed by the external language, via pVals returned by *Get or via pBuf in *Release.
     - The values in pVals can also be used directly BUT MUST NOT BE CHANGED (i.e., they are constant) and *Release MUST BE CALLED after this is done.  For example if we simply want to take the sum of the elements, this can be the most efficient way.  However, note that between *Get and *Release, the port is usually locked, hence whatever operations are done on the values should be quick.  Another use case could be to access some elements to decide if we want to use the values in further calculation, in that case we can copy the values over, otherwise we just ignore them.
     
     Function *Get(...) returns 0 if successful, <0 if error; >0 if no value actually read (e.g., no pending value on a strict port).
     For strict ports, if there is no value pending, the receiving variables won't be changed and the function will return 1, and there is no need to call *Release (obviously).  In other words, only call *Release if *Get returns 0.
     */
    int inputVectorDoubleGet(size_t nodeid, size_t portid, void** pMan, const double** pVals, size_t* nelems);      // Float64
    void inputVectorDoubleRelease(void* pMan, double* pBuf);
    
    int inputVectorBoolGet(size_t nodeid, size_t portid, void** pMan, const bool** pVals, size_t* nelems);          // C++ bool (1 byte)
    void inputVectorBoolRelease(void* pMan, bool* pBuf);
    
    int inputVectorInt32Get(size_t nodeid, size_t portid, void** pMan, const int32_t** pVals, size_t* nelems);      // Int32
    void inputVectorInt32Release(void* pMan, int32_t* pBuf);
    
    int inputVectorInt64Get(size_t nodeid, size_t portid, void** pMan, const int64_t** pVals, size_t* nelems);      // Int64
    void inputVectorInt64Release(void* pMan, int64_t* pBuf);
    
    int inputVectorUInt32Get(size_t nodeid, size_t portid, void** pMan, const uint32_t** pVals, size_t* nelems);    // UInt32
    void inputVectorUInt32Release(void* pMan, uint32_t* pBuf);
    
    int inputVectorUInt64Get(size_t nodeid, size_t portid, void** pMan, const uint64_t** pVals, size_t* nelems);    // UInt64
    void inputVectorUInt64Release(void* pMan, uint64_t* pBuf);
    
    
    int inputMatrixDoubleGet(size_t nodeid, size_t portid, void** pMan, const double** pVals, size_t* nrows, size_t* ncols);      // Float64
    void inputMatrixDoubleRelease(void* pMan, double* pBuf);
    
    int inputMatrixBoolGet(size_t nodeid, size_t portid, void** pMan, const bool** pVals, size_t* nrows, size_t* ncols);          // C++ bool (1 byte)
    void inputMatrixBoolRelease(void* pMan, bool* pBuf);
    
    int inputMatrixInt32Get(size_t nodeid, size_t portid, void** pMan, const int32_t** pVals, size_t* nrows, size_t* ncols);      // Int32
    void inputMatrixInt32Release(void* pMan, int32_t* pBuf);
    
    int inputMatrixInt64Get(size_t nodeid, size_t portid, void** pMan, const int64_t** pVals, size_t* nrows, size_t* ncols);      // Int64
    void inputMatrixInt64Release(void* pMan, int64_t* pBuf);
    
    int inputMatrixUInt32Get(size_t nodeid, size_t portid, void** pMan, const uint32_t** pVals, size_t* nrows, size_t* ncols);    // UInt32
    void inputMatrixUInt32Release(void* pMan, uint32_t* pBuf);
    
    int inputMatrixUInt64Get(size_t nodeid, size_t portid, void** pMan, const uint64_t** pVals, size_t* nrows, size_t* ncols);    // UInt64
    void inputMatrixUInt64Release(void* pMan, uint64_t* pBuf);

    
    /** These functions read (or pop) the array of bytes from a non-strict (or strict) binary input port.
     These work in the same way as those for vector/matrix ports, but with byte arrays.
     In other words, consider a binary input port as a vector-of-bytes input port.
     */
    int inputBinaryGet(size_t nodeid, size_t portid, void** pMan, const char** pVals, size_t* nbytes);
    void inputBinaryRelease(void* pMan, char* pBuf);
    
    
    /* === Misc === */
    // Returns the maximum ID allowed for an update type.
    int maxUpdateID();
    
    // Returns the last error/warning message as a C null-terminated string.
    // Do not try to modify the string.
    const char* lastErrorMessage();
    const char* lastWarningMessage();
    
#ifdef __cplusplus
}
#endif


// Macros defined for exporting functions in DLL

#if defined _WIN32 || defined __CYGWIN__
#ifdef BUILDING_DLL
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __declspec(dllimport)
#endif
#elif defined __APPLE__ || defined __linux__
#define EXPORT __attribute__ ((visibility ("default")))
#else
#error "Unknown platform."
#endif

// Report an error and may terminate.
void reportError(const char* msg);

// Report a warning
void reportWarning(const char* msg);

#endif /* OBNNODE_EXT_H_ */
