/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief YARP port classes for the C++ node interface.
 *
 * Implement the basic interface of YARP port for a C++ node.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_YARPPORTBASE_H_
#define OBNNODE_YARPPORTBASE_H_

#include <cassert>
#include <string>
#include <obnnode_basic.h>
#include <yarp/os/all.h>

namespace OBNnode {
    class YarpNodeBase;
    
    /** \brief Send and receive binary data through YARP.
     
     This class subclasses yarp::os::Portable to package a binary message to send over YARP, or receive a binary message from YARP.
     It simply uses a header consisting of the message size in bytes, followed by the message's binary data.
     
     To reduce dynamic allocations, the message object will reuse allocated memory if possible.
     It manages the allocated size (_allocsize) and the actual size of the current message (_size).
     */
    class YARPMsgBin : public yarp::os::Portable {
    protected:
        OBNsim::ResizableBuffer m_buffer;   ///< The buffer to store data

    public:
        virtual bool write(yarp::os::ConnectionWriter& connection) {
            connection.appendInt(m_buffer.size());
            connection.appendExternalBlock(m_buffer.data(), m_buffer.size());
            return true;
        }
        virtual bool read(yarp::os::ConnectionReader& connection) {
            if (connection.isError()) {
                return false;
            }
            
            int newsize = connection.expectInt();
            if (newsize <= 0) {
                m_buffer.allocateData(0);
                return newsize<0?false:true;
            }
            
            m_buffer.allocateData(newsize);
            
            return connection.expectBlock(m_buffer.data(), m_buffer.size());
        }
        
        /** \brief Set the contents of the message by a byte string.
         
         \param msg Pointer to the binary data.
         \param len Length of the binary data, in bytes.
         */
        void setBinaryData(const void* msg, size_t len) {
            m_buffer.allocateData(len);
            memcpy(m_buffer.data(), msg, len);
        }
        
        /** \brief Set the contents of the message by a std::string.
         
         \param s The std::string to copy from.
         */
        void setBinaryData(const std::string &s) {
            m_buffer.allocateData(s.length());
            s.copy(m_buffer.data(), s.length());
        }
        
        /** \brief Get the pointer to the binary contents of the message. */
        const char* getBinaryData() const {
            return m_buffer.data();
        }
        
        /** \brief Return the size in bytes of the current binary data. */
        size_t getBinaryDataSize() const {
            return m_buffer.size();
        }
    };
    
    
    /** \brief Package a protobuf message to send to or receive from YARP.
     
     This class subclasses yarp::os::Portable to package a protobuf message to send over YARP, or a potentially different protobuf message to read from YARP.
     It defines methods to read and write messages, in protobuf format, with YARP connections.
     It simply uses a header consisting of the message size in bytes, followed by the message's binary data.
     
     To reduce dynamic allocations, the message object will reuse allocated memory if possible.
     It manages the allocated size (_allocsize) and the actual size of the current message (_size).
     
     TR is the type of the protobuf message to be read from YARP.
     TW is the type of the protobuf message to be sent to YARP.
     
     For input-only or output-only, we can use only the first template type because TR is set to TW by default.
     */
    template <typename TW, typename TR = TW>
    class YARPMsgPB : public YARPMsgBin {
    public:
        
        /** \brief Set the binary contents of the message from a ProtoBuf message object, to be sent over Yarp. */
        bool setMessage(const TW &msg) {
            m_buffer.allocateData(msg.ByteSize());
            return msg.SerializeToArray(m_buffer.data(), m_buffer.size());
        }
        
        /** \brief Get the ProtoBuf message object from the binary contents of the message received from Yarp. */
        bool getMessage(TR &msg) const {
            if (!m_buffer.data()) {
                return false;
            }
            return msg.ParseFromArray(m_buffer.data(), m_buffer.size());
        }
    };
    
    
    /** \brief Base class for an openBuildNet port, contains name, mode, etc.
     */
    class YarpPortBase: public PortBase {
    protected:
        /** Returns the actual Yarp port. */
        virtual yarp::os::Contactable& getYarpPort() = 0;
        virtual const yarp::os::Contactable& getYarpPort() const = 0;
        
        /** Close the port. */
        virtual void close() override {
            getYarpPort().close();
        }
        
        /** Open the port given a full network name. */
        virtual bool open(const std::string& full_name) {
            return getYarpPort().open(full_name[0]=='/'?full_name:('/'+full_name));
        }
        
        virtual std::pair<int, std::string> connect_from_port(const std::string& source) override;
    public:
        YarpPortBase(const std::string& t_name): PortBase(t_name) { }
        //virtual ~YarpPortBase() { }
        
        virtual std::string fullPortName() const override {
            return getYarpPort().getName();
        }
    };
    
    /** \brief Base class for an openBuildNet output port.
     */
    class YarpOutputPortBase: public OutputPortBase {
    protected:
        /** Returns the actual Yarp port. */
        virtual yarp::os::Contactable& getYarpPort() = 0;
        virtual const yarp::os::Contactable& getYarpPort() const = 0;
        
        /** Close the port. */
        virtual void close() override {
            getYarpPort().close();
        }
        
        /** Open the port given a full network name. */
        virtual bool open(const std::string& full_name) {
            return getYarpPort().open(full_name[0]=='/'?full_name:('/'+full_name));
        }
    public:
        YarpOutputPortBase(const std::string& t_name): OutputPortBase(t_name) { }
        //virtual ~YarpOutputPortBase() { }
        
        virtual std::string fullPortName() const override {
            return getYarpPort().getName();
        }
    };
}


#endif /* OBNNODE_YARPPORTBASE_H_ */