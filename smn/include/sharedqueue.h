/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Thread-safe shared queue of pointers to objects.
 *
 * Header file to implement a thread-safe shared queue of pointers to objects.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 * \see https://juanchopanzacpp.wordpress.com/2013/02/26/concurrent-queue-c11/
 * \see http://codingways.blogspot.ch/2012/08/implementing-thread-safe-queue-in-c11.html
 * \see https://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
 */


#ifndef OBNSIM_SHAREDQUEUE_H_
#define OBNSIM_SHAREDQUEUE_H_

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>

/** \brief Template of thread-safe shared queue.
 
 This shared queue contains smart pointers to objects of type T.
 When pushing new objects, remember to dynamically create the objects, not a local scope object, e.g. using make_shared.
 Because this uses smart pointers, after popping out an object (actually a pointer to an object), there is no need to delete the object.
 \tparam T Type of the objects (the queue contains smart pointers to these objects, not the objects themselves).
 */
template <typename T>
class shared_queue
{
public:
    typedef typename std::shared_ptr <T> item_type; ///< Smart pointer type to the objects.

private :
    std::queue<item_type> mData;
    mutable std::mutex mMut;
    std::condition_variable &mEmptyCondition;   // condition variable to notify after pushing to queue
    
public:
    /**
     A condition_variable object must be given. After an item is pushed to the queue successfully, this condition variable will be notified. It is used by other threads to wait until the item is pushed.
     
     \param pc Points to a valid condition_variable, that will be used to wait for item being pushed into the queue.
     */
    shared_queue(std::condition_variable& pc): mEmptyCondition(pc) { }
    
    ///@{
    /** Push an object into the queue.
     
     \param pValue The object to be pushed, of type T and must be dynamically allocated.
     */
    void push(T& pValue) // The object of type T must be dynamically allocated
    {
        // block execution here, if other thread already locked mMute!
        std::unique_lock<std::mutex> mlock(mMut);
        // if we are here no other thread is owned/locked mMute. so we can modify the internal data
        //item_type v(&pValue);
        //mData.push(v);
        mData.emplace(&pValue);
        mlock.unlock();  // unlock before notifying to reduce contention
        mEmptyCondition.notify_all();
    }
    
    void push(T* pValue) // The object of type T must be dynamically allocated
    {
        // block execution here, if other thread already locked mMute!
        std::unique_lock<std::mutex> mlock(mMut);
        // if we are here no other thread is owned/locked mMute. so we can modify the internal data
        //item_type v(pValue);
        //mData.push(v);
        mData.emplace(pValue);
        mlock.unlock();  // unlock before notifying to reduce contention
        mEmptyCondition.notify_all();
    }

    void push(item_type v) // The object of type T must be dynamically allocated
    {
        // block execution here, if other thread already locked mMute!
        std::unique_lock<std::mutex> mlock(mMut);
        // if we are here no other thread is owned/locked mMute. so we can modify the internal data
        mData.push(v);
        mlock.unlock();  // unlock before notifying to reduce contention
        mEmptyCondition.notify_all();
    }
    ///@}
    
    /* Old code when the events are stored in a deque, not queue
    void push_front(T& pValue)
    {
        // block execution here, if other thread already locked mMute!
        std::unique_lock<std::mutex> mlock(mMut);
        // if we are here no other thread is owned/locked mMute. so we can modify the internal data
        item_type v(&pValue);
        mData.push_front(v);
        mlock.unlock();  // unlock before notifying to reduce contention
        mEmptyCondition.notify_all();
    }

    void push_front(T* pValue)
    {
        // block execution here, if other thread already locked mMute!
        std::unique_lock<std::mutex> mlock(mMut);
        // if we are here no other thread is owned/locked mMute. so we can modify the internal data
        item_type v(pValue);
        mData.push_front(v);
        mlock.unlock();  // unlock before notifying to reduce contention
        mEmptyCondition.notify_all();
    }
    
    void push_front(item_type v)
    {
        // block execution here, if other thread already locked mMute!
        std::unique_lock<std::mutex> mlock(mMut);
        // if we are here no other thread is owned/locked mMute. so we can modify the internal data
        mData.push_front(v);
        mlock.unlock();  // unlock before notifying to reduce contention
        mEmptyCondition.notify_all();
    }
    */
    
    /** \brief Wait until queue is non-empty and pop.
     
     This function waits, in a thread-safe and idling manner, until the queue is non-empty and pop the oldest element.
     Use try_pop if you don't want to wait.
     \return The oldest item.
     \see try_pop()
     */
    item_type wait_and_pop()
    {
        std::unique_lock<std::mutex> lock(mMut);
        // if we are here. mMute is locked and no other thread can access/modify the data!
        // wait() method first checks if mData is not empty, allowes execution to go on. else :
        // unlocks the mMut and waits for signla.
        // because mMute is released other threads have a chance to Push new data into queue
        // ... in notify this condition variable!
        while (mData.empty()) {
            mEmptyCondition.wait(lock);
        }
        
        // if we are are here, mData is not empty and mMut is locked !
        item_type val = mData.front();  // copy the shared_ptr to val, which is still valid even when the element is popped
        mData.pop();
        return val;
    }
    
    /** \brief Try to pop if non-empty.
    
     If the queue is non-empty, pop the oldest element; otherwise, return nil pointer.
     \return The oldest item, or nil pointer if the queue is empty.
     */
    item_type try_pop()
    {
        std::lock_guard<std::mutex> lock(mMut);

        return try_pop_with_lock();
    }
    
    /** \brief Try to pop when the caller HAS the lock on the queue access.
     
     If the queue is non-empty, pop the oldest element; otherwise, return nil pointer.
     Use this method ONLY IF the caller has the lock on the queue access; otherwise serious faults may happen.
     \return The oldest item, or nil pointer if the queue is empty.
     \see getMutex()
     */
    item_type try_pop_with_lock()
    {
        if (mData.empty())
            return item_type();  // nil
        item_type val = mData.front();
        mData.pop();
        return val;
    }
    
    bool empty() const ///< Check if the queue is empty.
    {
        std::lock_guard<std::mutex> lock(mMut);
        return mData.empty();
    }
    
    /** \brief Check if the queue is empty when the caller HAS the lock on the queue access.
     
     Use this method ONLY IF the caller has the lock on the queue access; otherwise serious faults may happen.
     \see getMutex()
     */
    bool empty_with_lock() const
    {
        return mData.empty();
    }

    /** \brief Return the mutex used to lock/unlock access to this queue. */
    std::mutex& getMutex() const { return mMut; }
};

#endif /* OBNSIM_SHAREDQUEUE_H_ */
