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
    public:
        virtual bool write(yarp::os::ConnectionWriter& connection) {
            connection.appendInt(_size);
            connection.appendExternalBlock(_data, _size);
            return true;
        }
        virtual bool read(yarp::os::ConnectionReader& connection) {
            if (connection.isError()) {
                return false;
            }
            
            int newsize = connection.expectInt();
            if (newsize <= 0) {
                _size = 0;
                return newsize<0?false:true;
            }
            
            allocateData(newsize);
            
            return connection.expectBlock(_data, _size);
        }
        
        /** \brief Set the contents of the message by a byte string.
         
         \param msg Pointer to the binary data.
         \param len Length of the binary data, in bytes.
         */
        void setBinaryData(const void* msg, size_t len) {
            allocateData(len);
            memcpy(_data, msg, len);
        }
        
        /** \brief Set the contents of the message by a std::string.
         
         \param s The std::string to copy from.
         */
        void setBinaryData(const std::string &s) {
            allocateData(s.length());
            s.copy(_data, s.length());
        }
        
        /** \brief Get the pointer to the binary contents of the message. */
        const char* getBinaryData() const {
            return _data;
        }
        
        /** \brief Return the size in bytes of the current binary data. */
        size_t getBinaryDataSize() const {
            return _size;
        }
        
        virtual ~YARPMsgBin() {
            if (_data) {
                delete [] _data;
            }
        }
        
    protected:
        /** The binary data of the message */
        char* _data = nullptr;
        
        /** The actual size of the message, not exceeding the allocated size. */
        size_t _size;
        
        /** The size of the allocated memory buffer (_data). */
        size_t _allocsize = 0;
        
        /** Allocate the memory block _data given the new size. It will reuse memory if possible. It will change _size. */
        void allocateData(size_t newsize) {
            assert(newsize >= 0);
            
            _size = newsize;
            
            if (_data) {
                // If _allocsize >= _size, we reuse the memory block
                if (_allocsize < _size) {
                    delete [] _data;
                }
            }
            else {
                _allocsize = 0;  // Make sure that _allocsize < _size
            }
            
            if (_allocsize < _size) {
                _allocsize = (_size & 0x0F)?(((_size >> 4)+1) << 4):_size;
                assert(_allocsize >= _size);
                _data = new char[_allocsize];
            }
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
            allocateData(msg.ByteSize());
            return msg.SerializeToArray(_data, _size);
        }
        
        /** \brief Get the ProtoBuf message object from the binary contents of the message received from Yarp. */
        bool getMessage(TR &msg) const {
            if (!_data) {
                return false;
            }
            return msg.ParseFromArray(_data, _size);
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