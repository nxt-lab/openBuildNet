/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Iterator classes for node update list.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBN_SIM_SMN__obnsim_iterators__
#define OBN_SIM_SMN__obnsim_iterators__

#include <obnsmn_gc.h>

namespace OBNsmn {
     /** \brief Implements iterators for the list of regular updates.
     
     It iterates gc_update_list.
     */
    class RegUpdateListIterator {
        const GCThread* gc;
        GCThread::NodeUpdateInfoList::const_iterator it;
        size_t pos;
        size_t maxsize;
    public:
        RegUpdateListIterator(const GCThread *pGC);
        
        RegUpdateListIterator& operator++();
        bool atEnd() const { return pos >= maxsize; }
        int getID() const { return it->nodeID; }
        outputmask_t getMask() const { return gc->_nodes[it->nodeID]->getNextUpdateMask(); }
    };
    
    /** \brief Implements iterators for the list of irregular updates.
     
     It iterates gc_update_list.
     */
    class IrregUpdateListIterator {
        //const GCThread* gc;
        GCThread::NodeUpdateInfoList::const_iterator it;
        size_t pos;
        size_t maxsize;
    public:
        IrregUpdateListIterator(const GCThread *pGC);
        IrregUpdateListIterator& operator++();
        bool atEnd() const { return pos >= maxsize; }
        int getID() const { return it->nodeID; }
        outputmask_t getMask() const { return it->irMask; }
    };
}

#endif /* defined(OBN_SIM_SMN__obnsim_iterators__) */
