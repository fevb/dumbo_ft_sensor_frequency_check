/*
 *  dumbo_ft_sensor_frequency_check.cpp
 *
 *  Created on: Dec 3, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <sys/mman.h>
#include <dumbo_force_torque_sensor/ForceTorqueSensor.h>
#include <vector>
#include <boost/thread.hpp>
#include <pthread.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <numeric>
#include <unistd.h>

bool quit = false;

boost::mutex mutex;
std::vector<double> loop_period(100,0.0);
std::vector<double> jitter(100, 0.0);


void quitRequested(int sig)
{
    std::cout << "shutting down!!!" << std::endl;
    quit = true;
}

void *printStats(void *)
{
    while(!quit)
    {
        mutex.lock();
        double avg_loop_period = std::accumulate(loop_period.begin(), loop_period.end(), 0.0)/loop_period.size();
        double avg_jitter = std::accumulate(jitter.begin(), jitter.end(), 0.0)/jitter.size();

        std::cout << "avg loop period (usec): " << avg_loop_period*1e+6 << std::endl;
        std::cout << "avg jitter (usec): " << avg_jitter*1e+6 << std::endl;

        mutex.unlock();

        usleep(1e+6);
    }
    int rv;
    return((void*) rv);
}

int main(int argc, char**argv)
{
    // Keep the kernel from swapping us out
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
      perror("mlockall");
      return -1;
    }

    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);


    double frequency = 1000.0; // Hz
    double period = (1/frequency)*(1e9); // period in nanoseconds

    // create thread for printing out stats
    int rv;
    pthread_t print_thread;
    pthread_attr_t print_thread_attr;
    pthread_attr_init(&print_thread_attr);

    if((rv = pthread_create(&print_thread, &print_thread_attr, printStats, NULL)) != 0)
    {
        std::cout << "Couldn't create print thread, exiting" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Set to realtime scheduler for this thread
    struct sched_param thread_param;
    int policy = SCHED_FIFO;
    thread_param.sched_priority = sched_get_priority_max(policy);
    pthread_setschedparam(pthread_self(), policy, &thread_param);


    // initialize the force torque sensor
    ForceTorqueSensor ft_sensor;

    ft_sensor.init("FT9910", "left");

    std::vector<double> force(3, 0.0);
    std::vector<double> torque(3, 0.0);

    int count = 0;

    // for logging the loop frequency
    struct timespec n;
    clock_gettime(CLOCK_MONOTONIC, &n);
    double current_loop_time = (double(n.tv_nsec)/1e+9)+n.tv_sec;
    double last_loop_time = (double(n.tv_nsec)/1e+9)+n.tv_sec;

    // control tick
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);

    // Snap to the nearest second
    tick.tv_sec = tick.tv_sec;
    tick.tv_nsec = (tick.tv_nsec / period + 1) * period;
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

    while(!quit)
    {

        ft_sensor.getFT(force, torque);

        // increase the tick
        tick.tv_nsec += period;

        while(tick.tv_nsec > 1e+9)
        {
            tick.tv_nsec -= 1e+9;
            tick.tv_sec++;
        }


        clock_gettime(CLOCK_MONOTONIC, &n);
        current_loop_time = (double(n.tv_nsec)/1e+9)+n.tv_sec;
        double current_period = current_loop_time - last_loop_time;
        last_loop_time = current_loop_time;

        // sleep until end of period
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

        struct timespec after;
        clock_gettime(CLOCK_REALTIME, &after);

        // calculate jitter
        double current_jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec-tick.tv_nsec)/1e+9);

        if(mutex.try_lock())
        {
            if(count++ > loop_period.size())
            {
                count = 0;
            }

            loop_period[count] = current_period;
            jitter[count] = current_jitter;

            mutex.unlock();
        }



    }

    pthread_join(print_thread, (void **)&rv);



}
