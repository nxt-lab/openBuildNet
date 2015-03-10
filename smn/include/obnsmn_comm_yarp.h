/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief YARP communication interface.
 *
 * Implement the communication interface with YARP.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSIM_COMM_YARP_H
#define OBNSIM_COMM_YARP_H

#ifndef OBNSIM_COMM_YARP
#error To use this library the program must be compiled with YARP support.
#endif

#include <cassert>
#include <memory>
#include <thread>

#include <yarp/os/all.h>
#include <obnsmn_node.h>
#include <obnsmn_gc.h>

/** \file
 Because of the issue with YARP thread interfering with the standard thread and mutex, we will use only one incoming YARP port for each GC.
 All nodes must send their messages to that same port, and are identified by their IDs embedded in the messages.
 Therefore, both SMN2N and N2SMN messages must have the ID embedded.
 To send messages from the SMN to nodes, we use individual ports for the nodes to avoid sending too many duplicate messages.
 For example, if we have node1 and node2, we may have `_gc_/node1` to send SMN2N messages to node1, similarly for node2; but both node1 and node2 will send their N2SMN messages to the same port `_gc_` which is polled by the single communication thread.
 */


namespace OBNsmn {
    namespace YARP {
        
        /** \brief Package an SMN2N message to send over YARP.
         
         This class subclasses yarp::os::Portable to package an SMN2N message to send over YARP, or an N2SMN message to read from YARP.
         It defines methods to read N2SMN messages and write SMN2N messages, in protobuf format, with YARP connections.
         It simply uses a header consisting of the message size in bytes, followed by the message's binary data.
         
         To reduce dynamic allocations, the message object will reuse allocated memory if possible.
         It manages the allocated size (_allocsize) and the actual size of the current message (_size).
         */
        class YARPMsg : public yarp::os::Portable {
        public:
            virtual bool write(yarp::os::ConnectionWriter& connection) {
                connection.appendInt(_size);
                connection.appendExternalBlock(_data, _size);
                return true;
            }
            virtual bool read(yarp::os::ConnectionReader& connection);
            
            /** \brief Set the contents of the SMN2N message. */
            bool setMessage(const OBNSimMsg::SMN2N &msg) {
                allocateData(msg.ByteSize());
                return msg.SerializeToArray(_data, _size);
            }
            
            /** \brief Get the contents of the message to a N2SMN object. */
            bool getMessage(OBNSimMsg::N2SMN &msg) const {
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
            void allocateData(size_t newsize);
        };
        
        
        typedef yarp::os::BufferedPort<YARPMsg> YARPPort;
        
        class OBNNodeYARP: public OBNNode {
        public:
            
            /** \brief Construct an YARP node with a given port.
             
             Construct a YARP node with a given name, number of output groups, and a given YARP port.
             The node object will use the port as-is: it will not create / delete / open / close / connect the port.
             Therefore, the port object must be initialized, and then deleted, properly somewhere else.
             
             The port is only used for sending messages out.
             */
            OBNNodeYARP(const std::string& _name, int _nOutputGrps, YARPPort* _port): OBNNode(_name, _nOutputGrps), port(_port) {
                assert(_port);
            }
            // virtual ~OBNNodeYARP() { std::cout << "OBNNodeYARP deleted." << std::endl; }
            
            /** \brief Asynchronously send a message to a node. */
            virtual bool sendMessage(int nodeID, OBNSimMsg::SMN2N &msg);
            
            /* \brief Synchronously send a message to a node.  */
            // virtual bool sendMessageSync(int nodeID, OBNSimMsg::SMN2N &msg);
            
        private:
            /** \brief YARP port object for sending messages to node. */
            YARPPort *port;
        };
        
        /** \brief Thread for the main incoming YARP port of a GC.
         
         YARP's thread support interferes with standard C++11 thread support, causing segmentation faults if they are mixed.
         To overcome this problem, currently I don't use callbacks of YARP but simply create my own thread for polling the main incoming YARP port of a GC and pushing events to its shared event queue.
         See the comments at the top of this file for more details.
         */
        class YARPPollingThread {
        public:
            
            /**
             Construct the YARP polling thread object, associated with a given GC and with a given port name.
             This object will create and manage the YARP port.
             The port will be opened when the thread starts, or manually with a method.
             The port will be closed when the thread finishes its execution, or when it's deleted.
             So make sure that either all connections are made after the port is created, or permanent connections can be used.
             \param _gc Pointer to a valid GC thread, with which this thread is associated.
             \param _port Name of the incoming YARP port.
             */
            YARPPollingThread(GCThread* _gc, std::string _port): pGC(_gc), portName(_port) {
            }
            
            virtual ~YARPPollingThread() {
                if (pThread) {
                    // Although we can detach the thread, it's not a good idea because the thread's main procedure uses members of this object; once this object is deleted, the thread may not be able to run anymore. So in this destructor, we need to finish the thread's execution before we can destroy the object.
                    
                    if (pThread->joinable()) pThread->join();
                    pThread->detach();
                    delete pThread;
                }
                
                // The port will close automatically when it's deleted.
            }
            
            /** \brief Return the port object. */
            const YARPPort& getPort() { return port; }
            
            /** \brief Open the port.
             \return True if successful.
             */
            bool openPort() { return port.open(portName); }
            
            /** \brief Close the port (stop activities). */
            void closePort() { port.interrupt(); port.close(); }
            
            /** \brief Start the thread.
             
             Start the thread if it is not running already. Only one thread is allowed to run at any moment.
             \return True if successful; false otherwise.
             */
            
            bool startThread() {
                if (pThread) return false;  // Already running
                if (!pGC) return false;    // pGC must point to a valid GC object
                
                if (port.isClosed()) {
                    if (!openPort()) return false;
                }
                
                port.setStrict();

                // at this point, port is opened
                pThread = new std::thread(&YARPPollingThread::ThreadMain, this);
                
                return true;
            }
            
            /** \brief Join the thread to current thread.
             
             Join the thread (if one is running) to the current thread, which will be blocked until the thread ends.
             This is useful for the main thread to wait for the thread to stop.
             \return True if successful; false otherwise.
             */
            bool joinThread() {
                if (!pThread) return false;
                
                pThread->join();
                // We can now safely destroy _gcthread
                delete pThread;
                pThread = nullptr;
                return true;
            }
            
        private:
            /** The GC object with which this communication thread is associated. */
            GCThread *pGC;
            
            /** Name of the incoming YARP port. */
            std::string portName;
            
            /** The YARP port object for receiving messages from nodes.
             To make sure that no messages are dropped, we must call setStrict().
             */
            YARPPort port;
            
            /** The communication thread */
            std::thread * pThread = nullptr;
            
            /** This function is the entry point for the thread. Do not call it directly. */
            void ThreadMain() {
                // This thread simply polls the main GC port
                OBNSimMsg::N2SMN msg;
                
                while (!pGC->simple_thread_terminate) {
                    YARPMsg *b = port.read(false);
                    if (b) {
                        // data received in *b
                        if (!b->getMessage(msg)) {
                            // We should report an error here
                            std::cout << "Critical error: error while parsing input message from a node." << std::endl;
                        }
                        
                        pGC->pushNodeEvent(msg, 0);
                    }
                }
                
                // Close the port after finishing
                port.close();
            }
        };
    }
}


#endif // OBNSIM_COMM_YARP_H
