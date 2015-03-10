/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Implementation of iterator classes for node update list.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnsmn_iterators.h>

using namespace OBNsmn;

RegUpdateListIterator::RegUpdateListIterator(const GCThread *pGC): gc(pGC), pos(0), maxsize(pGC->gc_update_size) {
    it = gc->gc_update_list.begin();
    
    // Move to the first valid item
    while (pos < maxsize) {
        if (it->type == 2) {
            ++pos;
            ++it;
        } else {
            break;
        }
    }
}

RegUpdateListIterator& RegUpdateListIterator::operator++() {
    if (pos >= maxsize) return *this;
    while (++pos < maxsize) {
        if ((++it)->type != 2) break;
    }
    return *this;
}


IrregUpdateListIterator::IrregUpdateListIterator(const GCThread *pGC): pos(0), maxsize(pGC->gc_update_size) {
    it = pGC->gc_update_list.begin();
    
    // Move to the first valid item
    while (pos < maxsize) {
        if (it->type == 1) {
            ++pos;
            ++it;
        } else {
            break;
        }
    }
}
IrregUpdateListIterator& IrregUpdateListIterator::operator++() {
    if (pos >= maxsize) return *this;
    while (++pos < maxsize) {
        if ((++it)->type != 1) break;
    }
    return *this;
}
