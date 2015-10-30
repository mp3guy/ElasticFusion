/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef THREADMUTEXOBJECT_H_
#define THREADMUTEXOBJECT_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

template <class T>
class ThreadMutexObject
{
    public:
        ThreadMutexObject()
        {}

        ThreadMutexObject(T initialValue)
         : object(initialValue),
           lastCopy(initialValue)
        {}

        void assign(T newValue)
        {
            mutex.lock();

            object = lastCopy = newValue;

            mutex.unlock();
        }

        std::mutex & getMutex()
        {
            return mutex;
        }

        T & getReference()
        {
            return object;
        }

        void assignAndNotifyAll(T newValue)
        {
            mutex.lock();

            object = newValue;

            signal.notify_all();

            mutex.unlock();
        }
        
        void notifyAll()
        {
            mutex.lock();

            signal.notify_all();

            mutex.unlock();
        }

        T getValue()
        {
            mutex.lock();

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        T waitForSignal()
        {
            mutex.lock();

            signal.wait(mutex);

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        T getValueWait(int wait = 33000)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(wait));

            mutex.lock();

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        T & getReferenceWait(int wait = 33000)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(wait));

            mutex.lock();

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        void operator++(int)
        {
            mutex.lock();

            object++;

            mutex.unlock();
        }

    private:
        T object;
        T lastCopy;
        std::mutex mutex;
        std::condition_variable_any signal;
};

#endif /* THREADMUTEXOBJECT_H_ */
