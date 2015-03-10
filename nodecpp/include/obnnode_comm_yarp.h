/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief YARP communication interface for a C++ node.
 *
 * Implement the communication interface with YARP for a C++ node.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNNODE_COMM_YARP_H
#define OBNNODE_COMM_YARP_H

#ifndef OBNSIM_COMM_YARP
#error To use this library the program must be compiled with YARP support.
#endif

#include <cassert>
#include <memory>
#include <thread>

#include <obnsim_msg.pb.h>
#include <yarp/os/all.h>

/** \file
 Because of the issue with YARP thread interfering with the standard thread and mutex, we will use only one incoming YARP port for each GC.
 All nodes must send their messages to that same port, and are identified by their IDs embedded in the messages.
 Therefore, both SMN2N and N2SMN messages must have the ID embedded.
 To send messages from the SMN to nodes, we use individual ports for the nodes to avoid sending too many duplicate messages.
 For example, if we have node1 and node2, we may have `_gc_/node1` to send SMN2N messages to node1, similarly for node2; but both node1 and node2 will send their N2SMN messages to the same port `_gc_` which is polled by the single communication thread.
 */


namespace OBNnode {
    namespace YARP {
        
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
        template <class TW, class TR = TW>
        class YARPMsg : public yarp::os::Portable {
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
                    return false;
                }
                
                allocateData(newsize);
                
                return connection.expectBlock(_data, _size);
            }
            
            /** \brief Set the contents of the message. */
            bool setMessage(const TW &msg) {
                allocateData(msg.ByteSize());
                return msg.SerializeToArray(_data, _size);
            }
            
            /** \brief Get the contents of the message to a protobuf object. */
            bool getMessage(TR &msg) const {
                if (!_data || _size <= 0) {
                    return false;
                }
                return msg.ParseFromArray(_data, _size);
            }
            
            virtual ~YARPMsg() {
                if (_data) {
                    delete [] _data;
                }
            }
        private:
            /** The binary data of the message */
            char* _data = nullptr;
            
            /** The actual size of the message, not exceeding the allocated size. */
            size_t _size;
            
            /** The size of the allocated memory buffer (_data). */
            size_t _allocsize = 0;
            
            /** Allocate the memory block _data given the new size. It will reuse memory if possible. It will change _size. */
            void allocateData(size_t newsize) {
                assert(newsize > 0);
                
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
        
        /** A YARP message for communicating with the SMN. */
        typedef YARPMsg<OBNSimMsg::N2SMN, OBNSimMsg::SMN2N> smnMsg;
        
        /** A buffered YARP port for communication with the SMN. */
        typedef yarp::os::BufferedPort<smnMsg> smnPort;
    }
}


#endif // OBNNODE_COMM_YARP_H
