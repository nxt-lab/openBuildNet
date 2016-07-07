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

#include <stdint.h>


/** Types and functions the external interface can use. */
#ifdef __cplusplus
extern "C" {
#endif

    /** The update mask type, see obnsim_basic.h for the definition. That should match the definition here. */
    typedef uint64_t OBNUpdateMask;

    /** The type used for simulation time, see obnsim_basic.h for the definition of simtime_t. That should match the definition here. */
    typedef int64_t OBNSimTimeType;

    /** The event type */
    enum OBNEI_EventType {
        OBNEI_Event_INIT = 0,           // Init of simulation
        OBNEI_Event_Y = 1,              // Update Y
        OBNEI_Event_X = 2,              // Update X
        OBNEI_Event_TERM = 3,           // Termination of simulation
        OBNEI_Event_RCV = 4,            // A port has received a message
        OBNEI_Event_RESTART = 5         // Restarting simulation (INIT while it's running)
    };

    /** Type to pass arguments of an event */
    struct OBNEI_EventArg {
        OBNUpdateMask mask;     // Update mask (see above)
        size_t index;           // Index
    };


    /* === Node interface === */

    // Create a new node object, given nodeName, workspace, and optional server address.
    // Returns 0 if successful; >0 if node already exists; <0 if error.
    // id stores the ID of the new node.
    int createOBNNode(const char* name, const char* workspace, const char* server, size_t* id);

    // Delete a node, given its ID
    // Returns 0 if successful; <0 if node doesn't exist.
    int deleteOBNNode(size_t id);

    // Request/notify the SMN to stop, then terminate the node's simulation regardless of whether the request was accepted or not. See MQTTNodeExt::stopSimulation for details.
    // Args: node ID
    // Return: 0 if successful
    int nodeStopSimulation(size_t nodeid);

    // Requests the SMN/GC to stop the simulation (by sending a request message to the SMN) but does not terminate the node.
    // If the SMN/GC accepts the request, it will broadcast a TERM message to all nodes, which in turn will terminate this node.
    // See MQTTNodeExt::requestStopSimulation() for details.
    // Args: node ID
    // Return: 0 if successful
    int nodeRequestStopSimulation(size_t nodeid);

    // Check if the current state of the node is STOPPED
    // Args: node ID
    // Returns: true if >0, false if =0, error if <0.
    int nodeIsStopped(size_t nodeid);

    // Check if the current state of the node is ERROR
    // Args: node ID
    // Returns: true if >0, false if =0, error if <0.
    int nodeIsError(size_t nodeid);

    // Check if the current state of the node is RUNNING
    // Args: node ID
    // Returns: true if >0, false if =0, error if <0.
    int nodeIsRunning(size_t nodeid);

    // Returns the current simulation time of the node with a desired time unit.
    // Args: node ID, the time unit, double* time
    // Returns: 0 if successful
    // *time receives the current simulation time as a double (real number)
    // The time unit is an integer specifying the desired time unit. The allowed values are:
    // 0 = second, -1 = millisecond, -2 = microsecond, 1 = minute, 2 = hour
    int nodeSimulationTime(size_t nodeid, int timeunit, double* T);
    
    // Returns the atomic time unit, an integer in microseconds, of the simulation.
    // Args: node ID, OBNSimTimeType* tu
    // Returns: 0 if successful
    int nodeTimeUnit(size_t nodeid, OBNSimTimeType* tu);

    // Returns the current wallclock time of the node.
    // Args: node ID, long* time
    // Returns: 0 if successful
    // *time receives the current wallclock time as a POSIX time value.
    int nodeWallClockTime(size_t nodeid, int64_t* T);


    /* === Node simulation control interface === */

    // Get the next port event (e.g. message received) with a possible timeout.
    // Args: node ID, timeout (double in seconds, can be <= 0.0 if no timeout, i.e. returns immediately), unsigned int* event_type, size_t* portID
    // Returns: 0 if successful, >0 if timeout, <0 if other errors
    // If returning 0: *event_type is the type of port event (an integer cast from OBNEI_EventType, OBNEI_Event_RCV for message received); *portID is the index of the port associated with the event.
    int simGetPortEvent(size_t nodeid, double timeout, unsigned int* event_type, size_t* portid);


    // Runs the node's simulation until the next event, or until the node stops or has errors
    // Args: node ID, timeout (double in seconds, can be <= 0.0 if no timeout), unsigned int* event_type, OBNEI_EventArg* event_args
    // Returns: 0 if everything is going well and there is an event pending, 1 if timeout (but the simulation won't stop automatically, it's still running), 2 if the simulation has stopped (properly, not because of an error), 3 if the simulation has stopped due to an error (the node's state becomes NODE_ERROR), <0 if other error (e.g., node ID is invalid).  Check the last error message for specifics.
    // If returning 0: *event_type is the type of port event (an integer cast from OBNEI_EventType); *event_args are the event arguments depending on the event type (see the structure for details).
    int simRunStep(size_t nodeid, double timeout, unsigned int* event_type, OBNEI_EventArg* event_args);
    
    
    // Sets the result of the event processing, which is an integer value. Some events use this result (e.g., INIT).
    // Args: node ID, result (int64)
    // Returns: 0 if successful; <0 if other error (e.g., node ID is invalid).  Check the last error message for specifics.
    int simSetEventResult(size_t nodeid, int64_t result);
    

    // Request an irregular future update.
    // This is a blocking call, possibly with a timeout, that waits until it receives the response from the SMN or until a timeout.
    // Args: node ID, future time (integer value in the future), update mask of the requested update, timeout (double, can be <= 0)
    // Returns: status of the request: 0 if successful (accepted), -1 if timeout (failed), -2 if request is invalid, >0 if other errors (failed, see OBN documents for details).
    int simRequestFutureUpdate(size_t nodeid, OBNSimTimeType t, OBNUpdateMask mask, double timeout);


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

    // Request to connect a given port to a port on a node.
    // Arguments: node ID, port's ID, source port's name (string)
    // Returns: 0 if successful, otherwise error ID (last error message contains the error message).
    int portConnect(size_t nodeid, size_t portid, const char* srcport);

    // Enables the message received event at an input port
    // Args: node ID, port's ID
    // Returns: 0 if successful
    int portEnableRcvEvent(size_t nodeid, size_t portid);
    

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


    /** These functions set the value of a scalar output port, but does not send it immediately.
     Usually the value will be sent out at the end of the event callback (UPDATE_Y).
     Args: node ID, port's ID, scalar value.
     Returns: 0 if successful; <0 if error
     */
    int outputScalarDoubleSet(size_t nodeid, size_t portid, double val);      // Float64
    int outputScalarBoolSet(size_t nodeid, size_t portid, bool val);          // C++ bool (1 byte)
    int outputScalarInt32Set(size_t nodeid, size_t portid, int32_t val);      // Int32
    int outputScalarInt64Set(size_t nodeid, size_t portid, int64_t val);      // Int64
    int outputScalarUInt32Set(size_t nodeid, size_t portid, uint32_t val);    // UInt32
    int outputScalarUInt64Set(size_t nodeid, size_t portid, uint64_t val);    // UInt64


    /** These functions set the vector/matrix value of a vector/matrix output port, but does not send it immediately.
     Usually the value will be sent out at the end of the event callback (UPDATE_Y).
     The data are copied over to the port's internal memory, so there is no need to maintain the array pval after calling these functions (i.e., the caller is free to deallocate the memory of pval).
     Args: node ID, port's ID, <elem-type>* source, size_t nrows, size_t ncols  (for vector: size_t nelems)
     Returns: 0 if successful; <0 if error
     */
    int outputVectorDoubleSet(size_t nodeid, size_t portid, const double* pval, size_t nelems);      // Float64
    int outputVectorBoolSet(size_t nodeid, size_t portid, const bool* pval, size_t nelems);          // C++ bool (1 byte)
    int outputVectorInt32Set(size_t nodeid, size_t portid, const int32_t* pval, size_t nelems);      // Int32
    int outputVectorInt64Set(size_t nodeid, size_t portid, const int64_t* pval, size_t nelems);      // Int64
    int outputVectorUInt32Set(size_t nodeid, size_t portid, const uint32_t* pval, size_t nelems);    // UInt32
    int outputVectorUInt64Set(size_t nodeid, size_t portid, const uint64_t* pval, size_t nelems);    // UInt64

    int outputMatrixDoubleSet(size_t nodeid, size_t portid, const double* pval, size_t nrows, size_t ncols);      // Float64
    int outputMatrixBoolSet(size_t nodeid, size_t portid, const bool* pval, size_t nrows, size_t ncols);          // C++ bool (1 byte)
    int outputMatrixInt32Set(size_t nodeid, size_t portid, const int32_t* pval, size_t nrows, size_t ncols);      // Int32
    int outputMatrixInt64Set(size_t nodeid, size_t portid, const int64_t* pval, size_t nrows, size_t ncols);      // Int64
    int outputMatrixUInt32Set(size_t nodeid, size_t portid, const uint32_t* pval, size_t nrows, size_t ncols);    // UInt32
    int outputMatrixUInt64Set(size_t nodeid, size_t portid, const uint64_t* pval, size_t nrows, size_t ncols);    // UInt64


    /** This function sets the binary value of a binary output port, but does not send it immediately.
     Usually the value will be sent out at the end of the event callback (UPDATE_Y).
     Args: node ID, port's ID, const char* source, size_t nbytes
     Returns: 0 if successful; <0 if error
     */
    int outputBinarySet(size_t nodeid, size_t portid, const char* pval, size_t nbytes);


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
