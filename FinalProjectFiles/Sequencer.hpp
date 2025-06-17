/*******************************************************************************
 * Author - Steve Rizor
 * Edited by - Venetia Furtado
 * Final Project:  Aircraft Detection using Automatic Dependent 
 * Surveillance–Broadcast (ADSB) Data
 * This code has been resued from previous Exercises of the course.
 * ECEN 5613 - Spring 2025
 * University of Colorado Boulder
 * Revised 04/29/2025
*
********************************************************************************/

#pragma once

#include <cstdint>
#include <functional>
#include <thread>
#include <vector>
#include <semaphore>
#include <atomic>
#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <syslog.h>
#include <ctime>
#include <chrono>
#include <algorithm> // For std::min and std::max

using namespace std;

//#define DEBUG
void trace(const char* func)
{
#ifdef DEBUG
    cout << "Reached " << func << endl;
#endif
}


// The service class contains the service function and service parameters
// (priority, affinity, etc). It spawns a thread to run the service, configures
// the thread as required, and executes the service whenever it gets released.
class Service
{
public:
    template<typename T>
    Service(T&& doService, uint8_t affinity, uint8_t priority, uint32_t period, const char* threadInfo) :
        _doService(doService), _affinity(affinity), _priority(priority),_period(period),_semaphore(0), _isRunning(true), _threadInfo(threadInfo)
    {
        trace(__func__);
        // Start the service thread, which will begin running the given function immediately
        _service = jthread(&Service::_provideService, this);
    }
    // Overload operator< for sorting within pair (optional)
    bool operator<(const Service &other) const
    {
        return _period < other._period; // Sort by `value` in CustomClass
    }

    uint32_t getPeriod()
    {
        return _period;
    }
 
    void stop()
    {
        // change state to "not running" using an atomic variable
        // (heads up: what if the service is waiting on the semaphore when this happens?)
        _isRunning = false;
        this->release(); //(releases the blocked service in _provideService which is blocked on the binary semaphore (_semaphore.acquire))
    }
 
    void release()
    {
        // todo(completed): release the service using the semaphore
        _semaphore.release(); // Release the semaphore (increment by 1)
    }

    void printStats()
    {
        std::cout<<"\n***Printing stats for "<<_threadInfo<<"***"<<std::endl;
        std::cout<<"WCET: "<<_maxRunTime<<"ms"<<std::endl;
        std::cout<<"Max runtime: "<<_maxRunTime<<"ms"<<std::endl;
        std::cout<<"Min runtime: "<<_minRunTime<<"ms"<<std::endl;
        std::cout<<"Avg runtime: "<<((double)_sumRunTime)/_runCount<<"ms"<<std::endl;
        std::cout<<"Execution Time Jitter: "<<_maxRunTime-_minRunTime<<"ms"<<std::endl;
    }
 
private:
    function<void(void)> _doService;
    jthread _service;
    uint8_t _affinity;
    uint8_t _priority;
    uint32_t _period;
    binary_semaphore _semaphore;  // Start with no permit
    atomic<bool> _isRunning;
    string _threadInfo;

    long int _maxRunTime = 0;
    long int _minRunTime = UINT32_MAX;
    long int _wcet = 0;
    long int _sumRunTime;
    uint32_t _runCount = 0;


    void _initializeService()
    {
        // (heads up: the thread is already running and we're in its context right now)
        pthread_t native_handle = _service.native_handle();

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(_affinity, &cpuset);
        if (pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Failed to set CPU affinity\n";
        }
        else
        {
            //std::cout << "Affinity set success!\n";
        }

        struct sched_param param;
        param.sched_priority = _priority;
    
        if (pthread_setschedparam(native_handle, SCHED_FIFO, &param) != 0) {
            std::cerr << "Failed to set thread priority\n";
        }
        else
        {
            //std::cout << "Thread priority set success!\n";
        }

        pthread_setname_np(pthread_self(), _threadInfo.c_str());
    }

    void _provideService()
    {
        trace(__func__);

        _initializeService();
        // todo(completed): call _doService() on releases (sem acquire) while the atomic running variable is true

        while(_isRunning == true)
        {
            _semaphore.acquire(); // Lock the semaphore (block until semaphore is not 1 and decrement from 1 to 0)

            // check if i am still allowed to run?
            if (_isRunning == false)
            {
                break;
            }
#if 1
            auto now_start = std::chrono::system_clock::now();
            auto now_ms_start = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now_start.time_since_epoch())
                                    .count();
            //syslog(LOG_INFO, "Starting service %s at time %ld ms", _threadInfo.c_str(), now_ms_start);
#endif
            _doService();
#if 1
            auto now_end = std::chrono::system_clock::now();
            auto now_ms_end = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now_end.time_since_epoch())
                                  .count();
            //syslog(LOG_INFO, "Stopping service %s at time %ld ms", _threadInfo.c_str(), now_ms_end);
#endif
            auto runtime = now_ms_end - now_ms_start;
            _minRunTime = std::min(_minRunTime, runtime);
            _maxRunTime = std::max(_maxRunTime, runtime);

            _sumRunTime += runtime;
            _runCount++;

            // edge case: zero out sum if count is going to wrap around to prevent error in avg calc
            if (_runCount == UINT32_MAX)
            {
                _sumRunTime = 0;
            }
        }
    }
};


bool sorter(const std::unique_ptr<Service> &a, const std::unique_ptr<Service> &b)
{
    if (a->getPeriod() < b->getPeriod())
    {
        return true;
    }
    return false;
}
 
// The sequencer class contains the services set and manages
// starting/stopping the services. While the services are running,
// the sequencer releases each service at the requisite timepoint.
class Sequencer
{
public:
    template<typename... Args>
    void addService(Args&&... args)
    {
        // Add the new service to the services list,
        // constructing it in-place with the given args
        trace(__func__);

        _services.emplace_back(make_unique<Service>(forward<Args>(args)...));
    }

    void startServices()
    {
        // todo(completed): start timer(s), release services
        trace(__func__);

        timePoint++;

        for (size_t i = 0; i < _services.size(); i++) 
        {
            if (timePoint % _services[i]->getPeriod() == 0)
            {
                _services[i]->release();  // Dereferencing unique_ptr
            }
        }
    }

    void stopServices()
    {
        // todo(completed): stop timer(s), stop services
        trace(__func__);
        for (size_t i = 0; i < _services.size(); ++i) 
        {
            _services[i]->stop();  // Dereferencing unique_ptr
        }
        stopTimer();
    }

    void printStatistics()
    {
        // todo(completed): stop timer(s), stop services
        trace(__func__);
        for (size_t i = 0; i < _services.size(); ++i) 
        {
            _services[i]->printStats();  // Dereferencing unique_ptr
        }
    }

    void initTimer(void (*callbackFunc)(int))
    {
        struct sigevent sev{};
        struct itimerspec its{};
        //timer_t timerid;

        sev.sigev_notify = SIGEV_SIGNAL;  // Notify using a signal
        sev.sigev_signo = SIGALRM;        // Send SIGALRM when the timer expires
        signal(SIGALRM, callbackFunc);  // Register handler for SIGALRM

        timer_create(CLOCK_REALTIME, &sev, &timerid);

        uint32_t interval_ms = 1;
        its.it_value.tv_sec = interval_ms / 1000;
        its.it_value.tv_nsec = (interval_ms % 1000) * 1000000;
        its.it_interval = its.it_value; // set interval to same interval_ms

        if (timer_settime(timerid, 0, &its, nullptr) == -1) {
            std::cerr << "Failed to start timer!\n";
        }
    }

    void stopTimer()
    {
        struct itimerspec its{};
        if (timer_settime(timerid, 0, &its, nullptr) == -1)
        {
            std::cerr << "Failed to stop timer!\n";
        }
    }

    void sortService()
    {
        std::sort(_services.begin(),_services.end(), sorter); // Default sorting by first, then second

        for(auto& s: _services)
        {
            cout<<"\nPeriod="<<s->getPeriod()<<endl;
        }    
    }
private:
    vector<unique_ptr<Service>> _services;
    uint32_t timePoint = 0;
    timer_t timerid;
};
