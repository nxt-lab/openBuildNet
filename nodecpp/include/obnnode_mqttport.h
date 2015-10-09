/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief MQTT communication interface.
 *
 * Implement the communication interface with MQTT.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_MQTTPORT_H
#define OBNNODE_MQTTPORT_H

#ifndef OBNNODE_COMM_MQTT
#error To use this library the program must be compiled with MQTT support.
#endif

#include <cassert>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_map>
#include <vector>

#include "MQTTAsync.h"

#include "obnnode_exceptions.h"
#include "obnnode_basic.h"


namespace OBNnode {
    /** \brief Interface of an MQTT input port
     */
    class IMQTTInputPort {
    public:
        /** Parse a binary message into the port.
         If there is an error, this function should throw an exception (see obnnode_exceptions.h for predefined errors).
         The exception can be caught and delegated to the main thread, which will handle them properly.
         \param msg Pointer to the start of the message data.
         \param msglen Number of bytes of the message data.
         */
        virtual void parse_message(void* msg, int msglen) = 0;
    };
    
    /** \brief The object that manages all MQTT communications (i.e. the MQTT communication thread).
     
     Uses the Async communication interface of Paho MQTT library.
     */
    class MQTTClient {
    private:
        static const int QOS;   // The desired QOS
        
        NodeBase* m_node;  ///< The node object to which this client is attached
        
        MQTTAsync m_client;     ///< The MQTT client, used for all communication needs
        
        // std::atomic_bool m_running{false};  ///< Status of this client
        
        // The result variable, mutex and condition variable is used by MQTT callbacks to notify the main execution.
        std::atomic_int m_notify_result{0}; ///< Result of the action that was notified
        int m_notify_done;  // Only done when this value is 0
        std::mutex m_notify_mutex;
        std::condition_variable m_notify_var;
        
        /** Notify that one action is done.
         \param setDone true [default] if the function should set m_notify_done = 0 (so that wait on m_notify_var will end); false if the function should only decrease m_notify_done.
         */
        void notify_done(bool setDone = true) {
            std::lock_guard<std::mutex> mylock(m_notify_mutex);
            if (setDone) {
                m_notify_done = 0;
            } else {
                --m_notify_done;
            }
            m_notify_var.notify_all();
        }
        
        /** Client ID */
        std::string m_client_id;
        
        /** MQTT server address. */
        std::string m_server_address{"tcp://localhost:1883"};
        
        /** Map of topics to list of subscribing input ports. */
        std::unordered_map< std::string, std::vector<IMQTTInputPort*> > m_topics;
        
        std::mutex m_topics_mutex;  ///< Mutex to access the list of topics
        
        /** \brief Subscribe to all topics of the current input ports.
         \param resubscribe Set to true if this is a resubscription request => if still fails, it's communication error
         */
        void subscribeAllTopics(bool resubscribe = false);
        
        /** \brief Subscribe to a given topic.
         The subscription is asynchronous; when it's done, notify_done() is called and m_notify_result contains the result (0 = successful).
         */
        void subscribeTopic(const std::string& topic);
        
        /** \brief Unsubscribe from the given topic. */
        void unsubscribeTopic(const std::string& topic);
        
    public:
        /**
         Construct the MQTT client object, associated with a given node.
         */
        MQTTClient(NodeBase* pnode = nullptr): m_node(pnode) { }
        MQTTClient(const MQTTClient&) = delete;
        MQTTClient(MQTTClient&&) = delete;
        
        //virtual
        ~MQTTClient() {
            // Close the client, shutdown thread, close the port
            stop();
        }
        
        bool isRunning() const {
            return MQTTAsync_isConnected(m_client);
        }
        
        /** Set the client ID. */
        void setClientID(const std::string& _clientID) {
            assert(!_clientID.empty());
            m_client_id = _clientID;
        }
        
        /** Set the server address */
        void setServerAddress(const std::string& addr) {
            assert(!addr.empty());
            m_server_address = addr;
        }
        
        /** Set the associated node object. */
        void setNodeObject(NodeBase* pnode) {
            m_node = pnode;
        }
        
        /** \brief Subscribe a given input port to a given topic (i.e. output port in MQTT).
         
         If the topic already exists, the given port will be added to the vector associated with that topic; otherwise a new topic is added.
         \return integer code that has the same meaning as the return code of system message SYS_PORT_CONNECT_ACK.
         */
        int addSubscription(IMQTTInputPort* port, const std::string& topic);
        
        /** \brief Remove a given port from all subscriptions.
         */
        void removeSubscription(IMQTTInputPort* port);
        
        /** \brief Send data to a given topic.
         \param data Pointer to the data to be sent.
         \param size The number of bytes of the data.
         \param topic The topic to send to.
         \return true if successful.
         */
        bool sendData(void *data, int size, const std::string& topic);
        
        /** \brief Start the MQTT client (thread).
         
         Start the client / thread if it is not running already. Only one is allowed to run at any moment.
         \return True if successful; false otherwise.
         */
        bool start();
        
        /** Stop the MQTT client. */
        void stop();
        
        
    private:
        /////////////////
        // Callbacks
        /////////////////
        
        /** Called when the connection with the server is lost. */
        static void on_connection_lost(void *context, char *cause);
        
        /** Called whenever a message is received. */
        static int on_message_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message *message);
        
        /** Called whenever a sent message has been delivered. */
        static void on_message_delivered(void *context, MQTTAsync_token token);
        
        /** Called when the connection with the server is established successfully. */
        static void onConnect(void* context, MQTTAsync_successData* response);
        
        /** Called when a connection attempt failed. */
        static void onConnectFailure(void* context, MQTTAsync_failureData* response);
        
        /** Called when subscription succeeds. */
        static void onSubscribe(void* context, MQTTAsync_successData* response);
        
        /** Called when subscription fails. */
        static void onSubscribeFailure(void* context, MQTTAsync_failureData* response);
        
        /** Called when the re-connection with the server is established successfully. */
        static void onReconnect(void* context, MQTTAsync_successData* response);
        
        /** Called when a re-connection attempt failed. */
        static void onReconnectFailure(void* context, MQTTAsync_failureData* response);
        
        /** Called when resubscription fails. */
        static void onReSubscribeFailure(void* context, MQTTAsync_failureData* response);
        
        /** Connection is permanently lost. Need to stop!!! */
        void onPermanentConnectionLost() {
            stop();
            // Notify that a critical error has happened
            if (m_node) {
                m_node->onPermanentCommunicationLost(COMM_MQTT);
            } else {
                // Throw an error
                throw std::runtime_error("Permanent connection lost for protocol MQTT");
            }
        }
        
        /** Called when the disconnection with the server is successful. */
        static void onDisconnect(void* context, MQTTAsync_successData* response);
    };

    
    //////////////////////////////////////////////////////////////////////
    // Definitions of MQTT ports
    //////////////////////////////////////////////////////////////////////
    
    /** The GC/SMN port in MQTT; it's just an input port. */
    class MQTTGCPort: public IMQTTInputPort {
        NodeBase* m_node;       // The node object to which the GC port will push events
        OBNSimMsg::SMN2N m_smn_msg; ///< The internal ProtoBuf message for parsing incoming SMN2N messages
        
    public:
        MQTTGCPort(NodeBase* pnode): m_node(pnode) {
            assert(pnode);
        }
        
        /** \brief Set the associated node object. */
//        void setNodeObject(NodeBase *pnode) {
//            assert(pnode);
//            m_node = pnode;
//        }
        
        virtual void parse_message(void* msg, int msglen) override;
    };
    
    
    /** \brief Base class for an openBuildNet input port in MQTT, contains name, mode, etc.
     */
    class MQTTInputPortBase: public PortBase, public IMQTTInputPort {
    protected:
        MQTTClient* m_mqtt_client;  ///< The MQTT Client object that manages the communication of this port
        
        /** Close the port. */
        virtual void close() override {
            // Remove itself from the managing client
            m_mqtt_client->removeSubscription(this);
        }
        
        /** Open the port given a full network name.
         In MQTT this does nothing.
         The port only starts working when an output is connected to it via connect_from_port().
         */
        virtual bool open(const std::string& full_name) {
            return true;
        }
        
        virtual std::pair<int, std::string> connect_from_port(const std::string& source) override;
    public:
        MQTTInputPortBase(const std::string& t_name, MQTTClient* t_client): PortBase(t_name), m_mqtt_client(t_client) {
            assert(t_client);
        }
        //virtual ~MQTTPortBase() { }
        
        virtual std::string fullPortName() const override {
            return isValid()?m_node->fullPortName(m_name):"";
        }
    };
    
    /** \brief Base class for an openBuildNet (strictly) output port in MQTT.
     A strictly output port does not accept any input.
     */
    class MQTTOutputPortBase: public OutputPortBase {
    protected:
        MQTTClient* m_mqtt_client;  ///< The MQTT Client object that manages the communication of this port
        
        /** Close the port. For an MQTT output port, this does nothing. */
        virtual void close() override {
        }
        
        /** Open the port given a full network name.
         For an MQTT output port, this does nothing. */
        virtual bool open(const std::string& full_name) override {
            return true;
        }
        
        std::string m_topicName{};    ///< The MQTT topic of this port
    public:
        MQTTOutputPortBase(const std::string& t_name, MQTTClient* t_client): OutputPortBase(t_name), m_mqtt_client(t_client)
        {
            assert(t_client);
        }
        //virtual ~MQTTOutputPortBase() { }
        
        /** \brief Returns the full path of the output port.
         
         In MQTT, this should return the topic of the output port, which can be used to connect it to input ports.
         */
        virtual std::string fullPortName() const {
            return isValid()?m_node->fullPortName(m_name):"";
        }
        
        /** Returns the topic of this port in MQTT. */
        const std::string& portTopicName() {
            if (m_topicName.empty() && isValid()) {
                // Update the topic name
                m_topicName = fullPortName();
            }
            return m_topicName;
        }
    };
            
            
    /** \brief Template class for an input port with specific type.
     
     This template class defines an input port with a specific fixed type (e.g. scalar, vector, matrix).
     Specializations are used to define the classes for each type.
     The template has the following signature:
     template <FORMAT, DATATYPE, STRICT> class InputPort;
     where:
     - FORMAT specifies the message format and is one of: OBN_PB for ProtoBuf for a fixed type, OBN_PB_USER for any ProtoBuf message format (which will be specified by DATATYPE), or OBN_BIN for raw binary data (user-defined format).
     - DATATYPE specifies the type of data, depending on FORMAT
     + If FORMAT is OBN_PB, DATATYPE can be:
     . bool, int32_t, int64_t, uint32_t, uint64_t, double, float: for scalars.
     . obn_vector<t> where t is one of the above types: a variable-length vector of elements of such type.
     . obn_vector_fixed<t, N> where t is one of the above type and N is a constant positive integer: a fixed-length vector of elements of such type. The data is statically allocated, hence more efficient. Incoming data will be checked and an error will be raised if its length is different than N.
     . obn_matrix<t> similar to obn_vector<t> but for a 2-D matrix.
     . obn_matrix_fixed<t, M, N> similar to obn_vector_fixed<t, N> but for a matrix of fixed numbers of rows (M) and columns (N).
     + If FORMAT is OBN_PB_USER then DATATYPE must be a ProtoBuf class generated by protoc. This may not be checked at compile time, but certain necessary methods of a ProtoBuf class for encoding and decoding data must be present. The data read from this port will be an object of this ProtoBuf class. The user is responsible for extracting values from the object.
     + If FORMAT is OBN_BIN then DATATYPE is irrelevant because the data read from this input port will be a binary string.
     - STRICT is a boolean value: if it is false (default), the input port is nonstrict, which means that any new incoming data will immediately replace the current datum (even if it has not been accessed); if it is true, the input port is strict, i.e. new incoming messages will not replace past messages but be queued to be accessed later.
     */
    template <typename F, typename D, const bool S=false>
    class MQTTInput;
    
    
    /** \brief Template class for an MQTT output port with specific type.
     
     This template class defines an MQTT output port with a specific fixed type (e.g. scalar, vector, matrix).
     Specializations are used to define the classes for each type.
     The template has the following signature:
     template <FORMAT, DATATYPE> class MQTTOutput;
     where:
     - FORMAT specifies the message format and is one of: OBN_PB for ProtoBuf for a fixed type, OBN_PB_USER for any ProtoBuf message format (which will be specified by DATATYPE), or OBN_BIN for raw binary data (user-defined format).
     - DATATYPE specifies the type of data, depending on FORMAT
     + If FORMAT is OBN_PB, DATATYPE can be:
     . bool, int32_t, int64_t, uint32_t, uint64_t, double, float: for scalars.
     . obn_vector<t> where t is one of the above types: a variable-length vector of elements of such type.
     . obn_vector_fixed<t, N> where t is one of the above type and N is a constant positive integer: a fixed-length vector of elements of such type. The data is statically allocated, hence more efficient.
     . obn_matrix<t> similar to obn_vector<t> but for a 2-D matrix.
     . obn_matrix_fixed<t, M, N> similar to obn_vector_fixed<t, N> but for a matrix of fixed numbers of rows (M) and columns (N).
     + If FORMAT is OBN_PB_USER then DATATYPE must be a ProtoBuf class generated by protoc. This may not be checked at compile time, but certain necessary methods of a ProtoBuf class for encoding and decoding data must be present. The data assigned to this port will be an object of this ProtoBuf class. The user is responsible for populating the object with appropriate values.
     + If FORMAT is OBN_BIN then DATATYPE is irrelevant because the data written to this output port will be a binary string.
     */
    template <typename F, typename D>
    class MQTTOutput;
    
    /** Implementation of MQTTInput for fixed data type encoded with ProtoBuf (OBN_PB), non-strict reading. */
    template <typename D>
    class MQTTInput<OBN_PB, D, false>: public MQTTInputPortBase {
    private:
        typedef OBN_DATA_TYPE_CLASS<D> _obn_data_type_class;
        typedef typename _obn_data_type_class::PB_message_class _pb_message_class;
        
        typename _obn_data_type_class::input_data_container m_cur_value;    ///< The typed value stored in this port
        std::atomic_bool m_pending_value{false};    ///< If a new value is pending (hasn't been read)
        
        /** The ProtoBuf message object to receive the data.
         This should be a permanent variable (a class member) rather than a temporary variable (in a function)
         because some implementations directly use the data stored in this message, rather than copying the data over.
         If a temporary message variable is used, in those cases, the program may crash (invalid access error). */
        _pb_message_class m_PBMessage;
        
        std::mutex m_valueMutex;    ///< Mutex for accessing the value
        
    public:
        typedef typename _obn_data_type_class::input_data_type ValueType;
        
    public:
        virtual void parse_message(void* msg, int msglen) override {
            // This managed input port does not generate events in the main thread
            // It simply saves the value in the message to the value
            try {
                // Parse the ProtoBuf message
                if (msg == nullptr || msglen < 0 || !m_PBMessage.ParseFromArray(msg, msglen)) {
                    // Error while parsing the raw message
                    throw OBNnode::inputport_error(this, OBNnode::inputport_error::ERR_RAWMSG);
                }
                
                // Read from the ProtoBuf message to the value
                std::unique_lock<std::mutex> mylock(m_valueMutex);
                bool result = OBN_DATA_TYPE_CLASS<D>::readPBMessage(m_cur_value, m_PBMessage);
                mylock.unlock();
                
                if (result) {
                    m_pending_value = true;
                } else {
                    // Error while reading the value, e.g. sizes don't match
                    throw OBNnode::inputport_error(this, OBNnode::inputport_error::ERR_READVALUE);
                }
                
            } catch (...) {
                // Catch everything and pass it to the main thread
                m_node->postExceptionEvent(std::current_exception());
            }
        }
        
    public:
        MQTTInput(const std::string& _name, MQTTClient* t_client): MQTTInputPortBase(_name, t_client) { }
        
        /** Get the current value of the port. If no message has been received, the value is undefined.
         The value is copied out, which may be inefficient for large data (e.g. a large vector or matrix).
         */
        ValueType operator() () {
            m_pending_value = false; // the value has been read
            std::lock_guard<std::mutex> mlock(m_valueMutex);
            return m_cur_value.v;
        }
        
        ValueType get() {
            m_pending_value = false; // the value has been read
            std::lock_guard<std::mutex> mlock(m_valueMutex);
            return m_cur_value.v;
        }
        
        typedef OBNnode::LockedAccess<typename _obn_data_type_class::input_data_container::data_type, std::mutex> LockedAccess;
        
        /** Returns a thread-safe direct access to the value of the port. */
        LockedAccess lock_and_get() {
            m_pending_value = false; // the value has been read
            return LockedAccess(&m_cur_value.v, &m_valueMutex);
        }
        
        /** Check if there is a pending input value (that hasn't been read). */
        bool isValuePending() const {
            return m_pending_value;
        }
    };
    
    
    /** Implementation of MQTTInput for custom ProtoBuf messages, non-strict reading. */
    template <typename PBCLS>
    class MQTTInput<OBN_PB_USER, PBCLS, false>: public MQTTInputPortBase {
        PBCLS m_cur_message;    ///< The current ProtoBuf data message stored in this port
        std::atomic_bool m_pending_value{false};    ///< If a new value is pending (hasn't been read)
        std::mutex m_valueMutex;    ///< Mutex for accessing the value
        
    public:
        virtual void parse_message(void* msg, int msglen) override {
            // This managed input port does not generate events in the main thread
            // It simply saves the value in the message to the value
            try {
                // Parse the ProtoBuf message into the internal variable
                bool result = false;
                if (msg != nullptr && msglen >= 0) {
                    std::lock_guard<std::mutex> mylock(m_valueMutex);
                    result = m_cur_message.ParseFromArray(msg, msglen);
                }
                
                if (result) {
                    m_pending_value = true;
                } else {
                    // Error while parsing the raw message
                    throw OBNnode::inputport_error(this, OBNnode::inputport_error::ERR_RAWMSG);
                }
            } catch (...) {
                // Catch everything and pass it to the main thread
                m_node->postExceptionEvent(std::current_exception());
            }
        }
        
        MQTTInput(const std::string& _name, MQTTClient* t_client): MQTTInputPortBase(_name, t_client) { }
        
        /** Returns a copy of the current message.
         To get direct access to the current message (without copying) see lock_and_get().
         */
        PBCLS get() {
            m_pending_value = false; // the value has been read
            std::lock_guard<std::mutex> mlock(m_valueMutex);
            return m_cur_message;
        }
        
        typedef OBNnode::LockedAccess<PBCLS, std::mutex> LockedAccess;
        
        /** Returns a thread-safe direct access to the value of the port. */
        LockedAccess lock_and_get() {
            m_pending_value = false;  // the value has been read
            return LockedAccess(&m_cur_message, &m_valueMutex);
        }
        
        /** Check if there is a pending input value (that hasn't been read). */
        bool isValuePending() const {
            return m_pending_value;
        }
    };


    /** Implementation of MQTTInput for binary data, non-strict reading. */
    template <typename D>
    class MQTTInput<OBN_BIN, D, false>: public MQTTInputPortBase {
        std::string m_cur_message;    ///< The current binary data message stored in this port
        std::atomic_bool m_pending_value{false};    ///< If a new value is pending (hasn't been read)
        std::mutex m_valueMutex;    ///< Mutex for accessing the value
        
        
    public:
        virtual void parse_message(void* msg, int msglen) override {
            // This managed input port does not generate events in the main thread
            // It simply saves the value in the message to the value
            
            m_pending_value = true;
            std::lock_guard<std::mutex> mylock(m_valueMutex);
            m_cur_message.assign(static_cast<char*>(msg), msglen);
        }
        
    public:
        MQTTInput(const std::string& _name, MQTTClient* t_client): MQTTInputPortBase(_name, t_client) { }
        
        /** Returns a copy of the current binary content, as a string. */
        std::string get() {
            m_pending_value = false;
            std::lock_guard<std::mutex> mylock(m_valueMutex);
            return m_cur_message;
        }
        
        typedef OBNnode::LockedAccess<std::string, std::mutex> LockedAccess;
        
        /** Returns a thread-safe direct access to the value of the port. */
        LockedAccess lock_and_get() {
            m_pending_value = false;  // the value has been read
            return LockedAccess(&m_cur_message, &m_valueMutex);
        }
        
        /** Check if there is a pending input value (that hasn't been read). */
        bool isValuePending() const {
            return m_pending_value;
        }
    };
    
    
    /** Implementation of MQTTOutput for fixed data type encoded with ProtoBuf (OBN_PB).
     This class of MQTTOutput is not thread-safe because usually it's accessed in the main thread only.
     */
    template <typename D>
    class MQTTOutput<OBN_PB, D>: public MQTTOutputPortBase {
        typedef OBN_DATA_TYPE_CLASS<D> _obn_data_type_class;
        
    public:
        typedef typename _obn_data_type_class::output_data_type ValueType;
        
    private:
        ValueType m_cur_value;    ///< The value stored in this port
        typename _obn_data_type_class::PB_message_class m_PBMessage;   ///< The ProtoBuf message object to format the data

        OBNsim::ResizableBuffer m_buffer;   ///< The buffer to store data
    public:
        MQTTOutput(const std::string& _name, MQTTClient* t_client): MQTTOutputPortBase(_name, t_client) { }
        
        /** Get the current (read-only) value of the port.
         The value is copied out, which may be inefficient for large data (e.g. a large vector or matrix).
         */
        ValueType operator() () const {
            return m_cur_value;
        }
        
        /** Directly access the value stored in this port; can change it (so it'll be marked as changed).
         If the value is a fixed-size Eigen vector/matrix and is going to be accessed many times, it will be a good idea to copy it to a local variable because the internal value variable in the port is not aligned for vectorization.
         Once all computations are done, the new value can be assigned to the port using either this operator or the assignment operator.
         */
        ValueType& operator* () {
            m_isChanged = true;
            return m_cur_value;
        }
        
        /** Assign new value to the port. */
        ValueType& operator= (ValueType && rhs) {
            m_cur_value = std::move(rhs);
            m_isChanged = true;
            return m_cur_value;
        }
        
        /** Assign new value to the port. */
        ValueType& operator= (const ValueType & rhs) {
            m_cur_value = rhs;
            m_isChanged = true;
            return m_cur_value;
        }
        
        
        /** Send data synchronously */
        virtual void sendSync() override {
            try {
                // Convert data to message
                OBN_DATA_TYPE_CLASS<D>::writePBMessage(m_cur_value, m_PBMessage);
                
                // Generate the binary content
                m_buffer.allocateData(m_PBMessage.ByteSize());
                if (!m_PBMessage.SerializeToArray(m_buffer.data(), m_buffer.size())) {
                    // Error while serializing the raw message
                    throw OBNnode::outputport_error(this, OBNnode::outputport_error::ERR_SENDMSG);
                }
                
                // Send the MQTT message
                if (!m_mqtt_client->sendData(m_buffer.data(), m_buffer.size(), portTopicName())) {
                    // Error while sending the message
                    throw OBNnode::outputport_error(this, OBNnode::outputport_error::ERR_SENDMSG);
                }
                m_isChanged = false;
            }
            catch (...) {
                // Catch everything and pass it to the main thread
                m_node->postExceptionEvent(std::current_exception());
            }
        }
    };


    /** Implementation of MQTTOutput for custom ProtoBuf data message (OBN_PB_USER).
     This class of MQTTOutput is not thread-safe because usually it's accessed in the main thread only.
     */
    template <typename PBCLS>
    class MQTTOutput<OBN_PB_USER, PBCLS>: public MQTTOutputPortBase {
        PBCLS m_cur_message;    ///< The ProtoBuf message stored in this port
        OBNsim::ResizableBuffer m_buffer;   ///< The buffer to store data
        
    public:
        MQTTOutput(const std::string& _name, MQTTClient* t_client): MQTTOutputPortBase(_name, t_client) { }
        
        /** Directly access the ProtoBuf message stored in this port; can change it (so it'll be marked as changed). */
        PBCLS& message() {
            m_isChanged = true;
            return m_cur_message;
        }
        
        /** Set the message content. */
        PBCLS& setMessage (const PBCLS& m) {
            m_isChanged = true;
            return (m_cur_message = m);
        }
        
        /** Assign a new message to the content of the port. */
        PBCLS& operator= (const PBCLS& m) {
            return setMessage(m);
        }
        
        /** Send data synchronously */
        virtual void sendSync() override {
            try {
                // Generate the binary content
                m_buffer.allocateData(m_cur_message.ByteSize());
                if (!m_cur_message.SerializeToArray(m_buffer.data(), m_buffer.size())) {
                    // Error while serializing the raw message
                    throw OBNnode::outputport_error(this, OBNnode::outputport_error::ERR_SENDMSG);
                }
                
                // Send the MQTT message
                if (!m_mqtt_client->sendData(m_buffer.data(), m_buffer.size(), portTopicName())) {
                    // Error while sending the message
                    throw OBNnode::outputport_error(this, OBNnode::outputport_error::ERR_SENDMSG);
                }
                m_isChanged = false;
            }
            catch (...) {
                // Catch everything and pass it to the main thread
                m_node->postExceptionEvent(std::current_exception());
            }
        }
    };
    
    
    /** Implementation of MQTTOutput for binary data message (OBN_BIN).
     This class of MQTTOutput is not thread-safe because usually it's accessed in the main thread only.
     */
    template <typename D>
    class MQTTOutput<OBN_BIN, D>: public MQTTOutputPortBase {
        std::string m_cur_message;    ///< The binary data message stored in this port
        
    public:
        MQTTOutput(const std::string& _name, MQTTClient* t_client): MQTTOutputPortBase(_name, t_client) { }
        
        /** Directly access the ProtoBuf message stored in this port; can change it (so it'll be marked as changed). */
        std::string& message() {
            m_isChanged = true;
            return m_cur_message;
        }
        
        /** Set the binary data content to a std::string */
        std::string& message(const std::string &s) {
            m_isChanged = true;
            return m_cur_message.assign(s);
        }
        
        /** Set the binary data content to n characters starting from a pointer. */
        std::string& message(const char* s, std::size_t n) {
            m_isChanged = true;
            return m_cur_message.assign(s, n);
        }
        
        /** Send data synchronously */
        virtual void sendSync() {
            try {
                // Send the MQTT message
                if (!m_mqtt_client->sendData(m_cur_message.data(), m_cur_message.size(), portTopicName())) {
                    // Error while sending the message
                    throw OBNnode::outputport_error(this, OBNnode::outputport_error::ERR_SENDMSG);
                }
                m_isChanged = false;
            }
            catch (...) {
                // Catch everything and pass it to the main thread
                m_node->postExceptionEvent(std::current_exception());
            }
        }
    };
}

#endif // OBNNODE_MQTTPORT_H
