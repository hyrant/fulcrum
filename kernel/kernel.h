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

#ifndef _KERNEL_H
#define _KERNEL_H

#include <stdint.h>
#include <stdbool.h>

#include "netapi.h"
#include "api.h"

/**
 * Disable processor interrupts.
 */
inline void disableInterrupts(void)
{
    asm volatile ("CPSID i");
}
/**
 * Enable processor interrupts.
 */
inline void enableInterrupts(void)
{ 
    asm volatile ("CPSIE i");
}
/**
 * Enable processor interrupts with a memory clobber.  This is usually
 * just a formality, but may as well tell the compiler what we're doing.
 */
inline void enableInterruptsClobber(void)
{ 
    asm volatile ("CPSIE i" : : : "memory");
}



#ifndef MAXIMUM_THREADS
/**
 * The maximum number of executing or pending-join threads.
 */
#define MAXIMUM_THREADS   16
#endif

#ifndef TICK_RATE
/**
 * The number of ticks and context switches per second.  This must result
 * in the main thread being serviced at least three times a second.  That is
 * this value divided by the number of threads must be greater than three. 
 * Failure to do that will cause the watchdog to trip and reset under
 * maximum load.
 */
#define TICK_RATE   256
#endif

#ifndef MAXIMUM_MULTIPART_BOUNDARY
/**
 * The maximum length of a multipart boundary in a POST request.  This should
 * be 70 per RFC 1341, but I don't think any browsers actually use that, so it
 * could be reduced.  This also determines the size of the receive buffer.
 */
#define MAXIMUM_MULTIPART_BOUNDARY 70
#endif

/**
 * The current system tick counter.  Increments continuously from system start
 * up.
 */
extern volatile uint32_t systick;

/**
 * The current system uptime in seconds.  This is preserved across 
 * (non-restarting) system faults.
 */
extern volatile uint32_t uptime;

/**
 * A counter that is decremented with every systick interrupt.  If it
 * reaches zero, user code is stopped with the fault.
 */
extern volatile uint32_t softwareWatchdog;
#define SOFTWARE_WATCHDOG_MAGIC_MIN 0xDE000013
#define SOFTWARE_WATCHDOG_MAGIC_MAX (0xDE000013+TICK_RATE*3)
#define SOFTWARE_WATCHDOG_MAGIC_DISABLED 0xFFDEADFF

#pragma pack(push,4)
typedef struct {
    uint32_t sp;
    uint32_t r[13];
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} DebugRegisters;
#pragma pack(pop)

typedef struct {
    DebugRegisters registers;
    struct {
        uint32_t SHCSR;
        uint32_t CFSR;
        uint32_t HFSR;
        uint32_t DFSR;
        uint32_t MMFAR;
        uint32_t BFAR;
        uint32_t AFSR;
    } data;
} FaultInformation;

typedef enum {
    /** System initializing or running with no user code available. */
    RunState_Initialize = 0,
    /** System initializing with a reset of all saved settings queued. */
    RunState_ResetInitialize,
    
    /** System in setup mode. */
    RunState_Setup,
    
    /** Normal operation with user code currently executing (kernel using 
     * PSP) */
    RunState_UserExecuting,
    /** User code manually halted, so no automatic restart pending. */
    RunState_UserHalted,
    
    /** User code halted due to too many faults in the socket layer. */
    RunState_FailureHalted_SocketIO,
    /** User code halted due to a software watchdog timeout. */
    RunState_FailureHalted_SoftwareWatchdog,
    
    /** User code not started because we came up from a low power reset. */
    RunState_ResetHalted_LowPower,
    /** User code not started because we came up from an independent 
     * watchdog reset. */
    RunState_ResetHalted_WindowWatchDog,
    /** User code not started because we came up from an window 
     * watchdog reset. */
    RunState_ResetHalted_IndependentWatchDog,
    /** User code not started because we came up from software reset. */
    RunState_ResetHalted_SoftwareReset,
    
    /** User code halted after a NMI fault. */
    RunState_Fault_NMI,
    /** User code halted after a hard fault. */
    RunState_Fault_Hard,
    /** User code halted after a memory fault. */
    RunState_Fault_Memory,
    /** User code halted after a bus fault. */
    RunState_Fault_Bus,
    /** User code halted after a usage fault. */
    RunState_Fault_Usage,
} SystemRunState;

typedef struct {
    /**
     * The current system state code.
     */
    SystemRunState state;
    union {
        /** The data for the system while the user code is stopped due to
         * a fault. */
        struct {
            uint16_t retryTime;
            
            union {
                FaultInformation fault;
            } debug;
        } userFaulted;
        
        /** The data for the system while in normal operation. */
        struct {
            /* Put this here so we don't use the memory elsewhere */
            char postBuffer[70];
            uint8_t postBufferLength;
            
            uint8_t networkFaultCounter;
            uint32_t lastNetworkFault;
            
            NetworkReset_Handler networkResetHandler;
            TCPClosed_Handler tcpClosedHandler;
            EXTI10_15_Handler extiHandler;
            HTTP_GET httpGET;
            HTTP_POST httpPOST;
        } normal;
        
        /** The data for the system while in setup mode. */
        struct {            
            struct in_addr ip;
            union {
                struct {
                    struct in_addr netmask;
                    struct in_addr gateway;
                    struct in_addr dns;
                } fixed;
                struct {
                    uint32_t leaseTimeout;
                } dhcp;
            } ipData;
            
            uint8_t bssid[6];
            char ssid[33];
            
            enum {
                Setup_Security_Open = 0,
                Setup_Security_WPA2,
                Setup_Security_WPA,
                Setup_Security_WEP64,
                Setup_Security_WEP128,
            } securityMode;
            union {
                char passphrase[33];
                uint8_t wepKey[13];
            } securityData;
            
            bool simpleConfigDone;
            uint32_t connectionAttemptStart;
        } setup;
    } detailed;
} SystemStatus;
/**
 * The current shared system state.
 */
extern SystemStatus systemStatus;

/**
 * Test if interrupts are enabled
 * 
 * @return true if interrupts are enabled
 */
bool Kernel_interruptsEnabled(void);

/**
 * Launch the user code.
 */
void Kernel_startUserCode(void);

/**
 * Halt the user code execution.
 */
void Kernel_haltUserCode(void);

/**
 * Reset the system entirely.  This just reboots the microcontroller, it does
 * not erase the flash.
 */
void Kernel_rebootSystem(void);

/**
 * Indicate that the user code has performed a software level fault
 * (e.x. it's messed with the network layer in bad way).
 * 
 * @param fault     the fault triggered
 */
void Kernel_softwareUserFault(SystemRunState fault);

/**
 * Calculate the number of elapsed ticks since the given start time.
 * 
 * @param start     the start tick
 * @return          the number of elapsed ticks
 */
uint32_t Kernel_elapsed(uint32_t start);

/**
 * Yield to the scheduler.
 */
void Thread_yield(void);

/**
 * Test if the currently executing thread is the main kernel thread.
 * 
 * @return true if the current thread is the kernel thread
 */
bool Thread_isKernel(void);

/**
 * Start a new thread.  When a thread exits interrupts are automatically
 * enabled, so cleanup can be done with them disabled and the exit handler
 * will reenable them when the thread is released.
 * 
 * @param routine       the thread routine to execute
 * @param arg           the argument to the thread routine
 * @param stackTop      a pointer to the top (last in memory) of the thread's stack
 */
void Thread_start(void (*routine)(void *), void *arg, void *stackTop);

/**
 * Start a new thread and mark it as joinable.  The thread will not be released
 * when it finishes executing until a join routine is called on it.
 * 
 * @param routine       the thread routine to execute
 * @param arg           the argument to the thread routine
 * @param stackTop      a pointer to the top (last in memory) of the thread's stack
 * @return              the ID of the newly created thread
 */
int Thread_startJoinable(void (*routine)(void *), void *arg, void *stackTop);

/**
 * Attempt a join on the given thread ID.  If successful the thread is released.
 * 
 * @param id            the thread ID
 * @return              true if the thread was joined
 */
bool Thread_tryJoin(int id);

/**
 * Wait for the given thread ID to finish and join it.
 * 
 * @param id            the thread ID
 */
void Thread_join(int id);

/**
 * Attempt a join on the given thread ID.  If successful the thread is released.
 * 
 * @param id            the thread ID
 * @param timeout       the maximum number of milliseconds
 * @return              true if the thread was joined
 */
bool Thread_joinWait(int id, uint32_t timeout);

/**
 * Yield to the scheduler only if the current thread is not the kernel thread.
 */
void Thread_userYield(void);

/**
 * Fill debug info for the given thread.
 * 
 * @param idx           the thread index (>= 1 and < MAXIMUM_THREADS)
 * @param reg           the output debug registers
 */
void Thread_debugInfo(int idx, DebugRegisters *reg);

/**
 * Test if a given thread is running.
 * 
 * @param idx           the thread index
 * @return              true if the thread is running
 */
bool Thread_isRunning(int idx);

/**
 * Return the total available flash size in KiB.
 * 
 * @return the flash size in KiB
 */
uint32_t Kernel_totalFlashSize(void);

/**
 * Get fractional seconds (16 lower order bits are the fractional second).
 * 
 * @return the fractional second counter
 */
uint32_t Kernel_fractionalSeconds(void);

/**
 * Get total seconds.
 * 
 * @return the seconds (uptime) counter
 */
uint32_t Kernel_seconds(void);

/**
 * Get the number of seconds elapsed since a given start time.
 * 
 * @param start     the second start time, from Kernel_seconds()
 * @return          the total seconds elapsed
 */
uint32_t Kernel_secondsElapsed(uint32_t start);

/**
 * Get the number of fractional seconds elapsed since a given star time.
 * 
 * @param start     the start time, from Kernel_fractionalSeconds()
 * @return          the total fractional seconds elapsed
 */
uint32_t Kernel_fractionalElapsed(uint32_t start);

/**
 * Sleep for a given number of milliseconds.  The time slept is not precise,
 * but will not be less than the requested amount.
 * 
 * @param ms        the number of milliseconds to sleep
 */
void Kernel_msleep(uint32_t ms);

/**
 * Set the handler to be called whenever the network layer is reset (due
 * to communications failures, usually).  This will be called from the kernel
 * stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void Kernel_installNetworkResetHandler(NetworkReset_Handler h);

/**
 * Set the handler to be called for EXTI10-15 interrupts.  This is used
 * internally by the network layer, so expect spurious calls.  This will be 
 * called from the kernel stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void Kernel_installEXTIHandler(EXTI10_15_Handler h);

/**
 * Set the handler to be called for HTTP GET requests.  This will be 
 * called from the kernel stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void Kernel_installGETHandler(HTTP_GET h);

/**
 * Set the handler to be called for HTTP POST requests.  This will be 
 * called from the kernel stack, so space is limited.
 * 
 * @param h     the handler to call
 */
void Kernel_installPOSTHandler(HTTP_POST h);

#endif
