/*
 * Copyright (C) 2013 Derek Hageman <Derek.C.Hageman@gmail.com> 
 * 
 * This file is part of Fulcrum.
 *
 * Fulcrum is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * Fulcrum is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with Fulcrum.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _FULCRUM_H
#define _FULCRUM_H


/* Despite hours of banging my head against the wall, I have not found a
 * way to convince LD that external symbols are thumb capable (even if I
 * actually define a dummy copy in a separate NOLOAD section).  So it was 
 * always emitting BLX instructions to even addresses (which of course usage
 * faults).  This makes for more code but at least it loads the right (odd)
 * address. */
#define _API_CALL    __attribute__ ((long_call))

#include <api.h>
#include <common/netapi.h>
#include <common/synchronization.h>


/**
 * Start a new thread.  When a thread exits interrupts are automatically
 * enabled, so cleanup can be done with them disabled and the exit handler
 * will reenable them when the thread is released.
 * 
 * @param routine       the thread routine to execute
 * @param arg           the argument to the thread routine
 * @param stackTop      a pointer to the top (last in memory) of the thread's stack
 */
void thread_start(void (*routine)(void *), void *arg, void *stackTop) _API_CALL;

/**
 * Start a new thread and mark it as joinable.  The thread will not be released
 * when it finishes executing until a join routine is called on it.
 * 
 * @param routine       the thread routine to execute
 * @param arg           the argument to the thread routine
 * @param stackTop      a pointer to the top (last in memory) of the thread's stack
 * @return              the ID of the newly created thread
 */
int thread_start_join(void (*routine)(void *), void *arg, void *stackTop) 
    _API_CALL;

/**
 * Attempt a join on the given thread ID.  If successful the thread is released.
 * 
 * @param id            the thread ID
 * @return              true if the thread was joined
 */
bool thread_try_join(int id) _API_CALL;

/**
 * Wait for the given thread ID to finish and join it.
 * 
 * @param id            the thread ID
 */
void thread_join(int id) _API_CALL;

/**
 * Attempt a join on the given thread ID.  If successful the thread is released.
 * 
 * @param id            the thread ID
 * @param timeout       the maximum number of milliseconds to wait
 * @return              true if the thread was joined
 */
bool thread_wait_join(int id, uint32_t timeout) _API_CALL;

/**
 * Yield to the scheduler.
 */
void yield(void) _API_CALL;

/**
 * Get fractional seconds (16 lower order bits are the fractional second).
 * 
 * @return the fractional second counter
 */
uint32_t time_fractional(void) _API_CALL;

/**
 * Get total seconds.
 * 
 * @return the seconds (uptime) counter
 */
uint32_t time_seconds(void) _API_CALL;

/**
 * Get the number of seconds elapsed since a given start time.
 * 
 * @param start     the second start time, from time_seconds()
 * @return          the total seconds elapsed
 */
uint32_t time_seconds_elapsed(uint32_t start) _API_CALL;

/**
 * Get the number of fractional seconds elapsed since a given start time.
 * 
 * @param start     the start time, from time_fractional()
 * @return          the total fractional seconds elapsed
 */
uint32_t time_fractional_elapsed(uint32_t start) _API_CALL;

/**
 * Sleep for a given number of milliseconds.  The time slept is not precise,
 * but will not be less than the requested amount.
 * 
 * @param ms        the number of milliseconds to sleep
 */
void msleep(uint32_t ms) _API_CALL;

/**
 * Set the board LED state.  This only changed the GPIO bits, so if it is
 * being controlled by an alternate function, this will have no effect.
 * 
 * @param on        true to turn on the LED
 */
void set_led(bool on) _API_CALL;

/**
 * Set the handler to be called whenever the network layer is reset (due
 * to communications failures, usually).  This will be called from the kernel
 * stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void handler_network_reset(NetworkReset_Handler h) _API_CALL;

/**
 * Set the handler to be called whenever a TCP socket has the remote end closed.
 * This will be called from the kernel stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void handler_tcp_closed(TCPClosed_Handler h) _API_CALL;

/**
 * Set the handler to be called for EXTI10-15 interrupts.  This is used
 * internally by the network layer, so expect spurious calls.  This will be 
 * called from the kernel stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void handler_exti10_15(EXTI10_15_Handler h) _API_CALL;

/**
 * Set the handler to be called for HTTP GET requests.  This will be 
 * called from the kernel stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void handler_http_get(HTTP_GET h) _API_CALL;

/**
 * Set the handler to be called for HTTP POST requests.  This will be 
 * called from the kernel stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void handler_http_post(HTTP_POST h) _API_CALL;

#endif
