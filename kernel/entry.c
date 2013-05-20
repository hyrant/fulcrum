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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f1/rcc.h>

#include "kernel.h"
#include "init.h"
#include "transport.h"
#include "control.h"
#include "server.h"

volatile uint32_t systick = 0;
volatile uint32_t uptime = 0;
volatile uint32_t softwareWatchdog = 0;
SystemStatus systemStatus;

extern unsigned _data_loadaddr, _ram, _data, _edata, _ebss, _stack, 
                _kernelbegin, _userbegin;

/* 
 * Some usage taken from http://www.coactionos.com/embedded-design/36-context-switching-on-the-cortex-m3.html 
 */

/* Simply test if interrupts are currently enabled */
bool Kernel_interruptsEnabled(void)
{
    uint32_t status;
    asm volatile (
        " MRS %0, PRIMASK \n" 
        : "=r" (status) );
    return (status & 1) == 0;
}

static uint32_t wrapU32(uint32_t start, uint32_t now)
{
    if (now < start)
        return 0xFFFFFFFF - start + now;
    return now - start;
}

uint32_t Kernel_seconds(void)
{
    return uptime;
}

/* Calculate the number of elapsed system ticks from a given start, accounting
 * for wrap around */
uint32_t Kernel_elapsed(uint32_t start)
{
    return wrapU32(start, systick);
}

uint32_t Kernel_secondsElapsed(uint32_t start)
{
    return wrapU32(start, uptime);
}

uint32_t Kernel_fractionalElapsed(uint32_t start)
{
    return wrapU32(start, Kernel_fractionalSeconds());
}

void Kernel_msleep(uint32_t total)
{
    uint32_t start = systick;
    total *= TICK_RATE;
    total += TICK_RATE/2;
    total /= 1000;
    do {
        Thread_userYield();
    } while (Kernel_elapsed(start) <= total);
}

void Kernel_installNetworkResetHandler(NetworkReset_Handler h)
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts(); 
    if (systemStatus.state == RunState_UserExecuting)
        systemStatus.detailed.normal.networkResetHandler = h;
    if (interruptable)
        enableInterrupts();
}
void Kernel_installTCPClosedHandler(TCPClosed_Handler h)
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts(); 
    if (systemStatus.state == RunState_UserExecuting)
        systemStatus.detailed.normal.tcpClosedHandler = h;
    if (interruptable)
        enableInterrupts();
}
void Kernel_installEXTIHandler(EXTI10_15_Handler h)
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts(); 
    if (systemStatus.state == RunState_UserExecuting)
        systemStatus.detailed.normal.extiHandler = h;
    if (interruptable)
        enableInterrupts();
}
void Kernel_installGETHandler(HTTP_GET h)
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts(); 
    if (systemStatus.state == RunState_UserExecuting)
        systemStatus.detailed.normal.httpGET = h;
    if (interruptable)
        enableInterrupts();
}
void Kernel_installPOSTHandler(HTTP_POST h)
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts(); 
    if (systemStatus.state == RunState_UserExecuting)
        systemStatus.detailed.normal.httpPOST = h;
    if (interruptable)
        enableInterrupts();
}
void Kernel_rebootSystem(void)
{
    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
}

/* Called when the vector table has changed.  Just make sure it's reached the
 * Device memory (since order is not preserved relative to Normal memory,
 * which is what our SRAM is). */
static inline void vectorTableChanged(void)
{
    asm volatile ("DSB");
}


/* Thread is active */
#define THREAD_FLAG_ACTIVE          0x01

/* Thread is using the MSP */
#define THREAD_FLAG_USE_MSP         0x02

/* Thread has an extra part of a time slice from a yield (so don't change on 
 * the systick) */
#define THREAD_FLAG_EARLY_TIMESLICE 0x04

/* Thread will not be released until it is joined */
#define THREAD_REQUIRE_JOIN         0x08

#pragma pack(push,4)
typedef struct {
    void *sp;       /* The SP at the last context switch */
    uint32_t flags;
} ThreadEntry;
#pragma pack(pop)

static ThreadEntry threads[MAXIMUM_THREADS];
static ThreadEntry *currentThread;

static void selectNextThread(void)
{
    while (true) {
        ++currentThread;
        if (currentThread >= &threads[MAXIMUM_THREADS]) {
            currentThread = &threads[0];
            return;
        }
        if (!(currentThread->flags & THREAD_FLAG_ACTIVE))
            continue;
        return;
    }
}

void Thread_yield(void)
{
    Transport_process();
    
    asm volatile (
        "    STR %0, [%1] \n"
        :
        : "r" (SCB_ICSR_PENDSVSET), "r" (&SCB_ICSR)
        : "memory" );
    /* Memory clobber isn't "really" needed, but may as well tell the compiler
     * that things have probably changed. */
}

bool Thread_isKernel(void)
{
    return (currentThread == &threads[0]);
}

void Thread_userYield(void) 
{
    if (Thread_isKernel())
        return;
    asm volatile (
        "    STR %0, [%1] \n"
        :
        : "r" (SCB_ICSR_PENDSVSET), "r" (&SCB_ICSR)
        : "memory" );
}

#pragma pack(push,4)
typedef struct {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
} HardwareStackFrame;
#pragma pack(pop)

/* Unflag the thread as executing then yield (which should never return 
 * control. */
static void threadExit(void) 
{
    disableInterrupts();
    currentThread->flags &= ~THREAD_FLAG_ACTIVE;
    enableInterrupts();
    Thread_yield();
    while (true) { }
}

static int internalThreadAdd(void (*routine)(void *), void *arg, 
                             void *stackTop, uint32_t flags) 
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts();
    for (int i=1; i<MAXIMUM_THREADS; i++) {
        if (threads[i].flags & THREAD_FLAG_ACTIVE)
            continue;
        if (threads[i].flags & THREAD_REQUIRE_JOIN)
            continue;
            
        HardwareStackFrame *frame = (HardwareStackFrame *)(
                ((uint32_t)stackTop & 0xFFFFFFFC) - /* Word align */
                sizeof(HardwareStackFrame));
        frame->r0 = (uint32_t)arg;
        frame->r1 = 0;
        frame->r2 = 0;
        frame->r3 = 0;
        frame->r12 = 0;
        frame->pc = (uint32_t)routine;
        frame->lr = (uint32_t)threadExit;
        frame->psr = 0x01000000;
        threads[i].flags = THREAD_FLAG_ACTIVE|flags;
        threads[i].sp = stackTop - 
            sizeof(HardwareStackFrame) - 
            8 * 4;  /* r4-r11 */

        if (interruptable)
            enableInterrupts();
        return i;
    }
    
    if (interruptable)
        enableInterrupts();
    return 0;
}

void Thread_start(void (*routine)(void *), void *arg, void *stackTop) 
{
    internalThreadAdd(routine, arg, stackTop, 0);
}
int Thread_startJoinable(void (*routine)(void *), void *arg, void *stackTop) 
{
    return internalThreadAdd(routine, arg, stackTop, THREAD_REQUIRE_JOIN);
}

bool Thread_tryJoin(int id) 
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts();
    if (threads[id].flags & THREAD_FLAG_ACTIVE) {
        if (interruptable)
            enableInterrupts();
        return false;
    }
    threads[id].flags &= ~THREAD_REQUIRE_JOIN;
    if (interruptable)
        enableInterrupts();
    return true;
}
void Thread_join(int id) 
{
    while (!Thread_tryJoin(id)) {
        Thread_yield();
    }
}
bool Thread_joinWait(int id, uint32_t timeout) 
{
    uint32_t start = systick;
    timeout *= 1000;
    timeout /= TICK_RATE;
    while (!Thread_tryJoin(id)) {
        if (Kernel_elapsed(start) > timeout)
            return false;
        Thread_yield();
    }
    return true;
}

void Kernel_startUserCode(void) 
{
    disableInterrupts();
    if (systemStatus.state == RunState_UserExecuting) {
        enableInterrupts();
        return;
    }
    
    void *entryAddress;
    void *entryStack;
    asm volatile (
        /* Set PSP to the current stack */
        "   MRS r0, msp \n"
        "   MSR psp, r0 \n"
        
        /* Entry SP */
        "   LDR %[entryStack], [%[vectorTable]] \n"
        /* Load entry handler */
        "   LDR %[entryAddress], [%[vectorTable], #4] \n"
        
        /* Adjust it up by the reserved size if it's that far from 1024 byte
         * boundary, this allows programs compiled with libopencm3 to correctly
         * return from main() (it resets the MSP on entry) if they set their
         * stack pointer back from the end of memory. */
        "   MOVW r0, #0x3FF \n"
        "   AND r0, %[entryStack] \n"
        "   ADD r0, %[stackReserved] \n"
        "   CMP r0, #1024 \n"
        "   IT EQ \n"
        "   ADDEQ %[entryStack], %[stackReserved] \n"
         
        /* Set MSP to where the new stack will be after the initial push */
        "   SUB r0, %[entryStack], %[stackReserved] \n"
        "   MSR msp, r0 \n"
        
        /* Switch to PSP for the kernel */
        "   MOVS r0, #0x2 \n"
        "   MSR control, r0 \n"
        : [entryStack] "=&r" (entryStack),
            [entryAddress] "=&r" (entryAddress)
        : [vectorTable] "r" (&_userbegin),
            [stackReserved] "I" (sizeof(HardwareStackFrame) + 4 * 8)
        : "r0");
        
    threads[0].flags &= ~THREAD_FLAG_USE_MSP;
        
    SCB_VTOR = (uint32_t)(&_userbegin);
    internalThreadAdd(entryAddress, NULL, entryStack, THREAD_FLAG_USE_MSP);
    
    systemStatus.state = RunState_UserExecuting;
    
    memset(&systemStatus.detailed.normal, 0, 
        sizeof(systemStatus.detailed.normal));
    
    vectorTableChanged();
    enableInterrupts();
}

void Kernel_softwareUserFault(SystemRunState fault)
{
    if (systemStatus.state != RunState_UserExecuting)
        return;
    Kernel_haltUserCode();
    
    memset(&systemStatus.detailed.userFaulted, 0, 
                sizeof(systemStatus.detailed.userFaulted));
    systemStatus.detailed.userFaulted.retryTime = 300;
    systemStatus.state = fault;
}

static void initializeData(void)
{
    volatile unsigned *src, *dest;
    for (src = &_data_loadaddr, dest = &_data; dest < &_edata; src++, dest++)
		*dest = *src;
	while (dest < &_ebss)
		*dest++ = 0;
}

static void anomalousReset(SystemRunState state)
{
    systemStatus.state = state;
    systemStatus.detailed.userFaulted.retryTime = 300;
}

static inline void initializeResetState(void)
{
    uint32_t resetFlags = RCC_CSR;
    RCC_CSR |= RCC_CSR_RMVF;
    if (resetFlags & RCC_CSR_LPWRRSTF) {
        anomalousReset(RunState_ResetHalted_LowPower);
        return;
    } else if (resetFlags & RCC_CSR_WWDGRSTF) {
        anomalousReset(RunState_ResetHalted_WindowWatchDog);
        return;
    } else if (resetFlags & RCC_CSR_IWDGRSTF) {
        anomalousReset(RunState_ResetHalted_IndependentWatchDog);
        return;
    } else if (resetFlags & RCC_CSR_SFTRSTF) {
        anomalousReset(RunState_ResetHalted_SoftwareReset);
        return;
    }
}

static void enterKernelLoop(void)
{
    currentThread = &threads[0];
    currentThread->flags = THREAD_FLAG_EARLY_TIMESLICE | THREAD_FLAG_ACTIVE | 
        THREAD_FLAG_USE_MSP;
    
    asm volatile ("MSR msp, %0 \n" : : "r" (&_stack) );
    enableInterrupts();
    asm volatile ( " B Control_main \n" );
}

void run(void)
{
    disableInterrupts();
    initializeData();
    initializeResetState();
    if (Kernel_initializePeripherals() && 
            systemStatus.state == RunState_Initialize) {
        systemStatus.state = RunState_ResetInitialize;
    }
    
    enterKernelLoop();
}

void runFaultEntry(void)
{
    Kernel_initializePeripherals();
    
    enterKernelLoop();
}

static void softwareWatchdogTripped(void);
void *_contextSwitchInner(void *stack, uint32_t mode) 
{
    currentThread->sp = stack;
    
    if (mode == 0) {
        /* Do this here, because the outer function is naked */
        if (((++systick) % TICK_RATE) == 0) {
            ++uptime;
        }
        if (systemStatus.state == RunState_UserExecuting) {
            uint32_t wdt = --softwareWatchdog;
            if (wdt == SOFTWARE_WATCHDOG_MAGIC_DISABLED-1) {
                softwareWatchdog = SOFTWARE_WATCHDOG_MAGIC_DISABLED;
            } else if (wdt <= SOFTWARE_WATCHDOG_MAGIC_MIN ||
                    wdt >= SOFTWARE_WATCHDOG_MAGIC_MAX) {
                softwareWatchdogTripped();
            }
        }
        
        if (currentThread->flags & THREAD_FLAG_EARLY_TIMESLICE) {
            /* Skip the switch if this is an extended time slice */
            currentThread->flags &= ~THREAD_FLAG_EARLY_TIMESLICE;
            return currentThread;
        } 
        
        selectNextThread();
    } else if (mode == 1) {
        /* Not on an extended time slice anymore */
        currentThread->flags &= ~THREAD_FLAG_EARLY_TIMESLICE;    
        selectNextThread();
        currentThread->flags |= THREAD_FLAG_EARLY_TIMESLICE;
    }

    return currentThread;
}

void Thread_debugInfo(int idx, DebugRegisters *reg)
{
    disableInterrupts();
        
    ThreadEntry *thread = &threads[idx];
    if (!(thread->flags & THREAD_FLAG_ACTIVE)) {
        enableInterrupts();
        memset(reg, 0, sizeof(DebugRegisters));
        return;
    }
    
    asm volatile (
        /* Thread SP */
        "   STR %[threadSP], [%[target]], #4 \n"
        /* r4-r11 */
        "   ADD %[target], #16 \n"
        "   MOVS r1, #8 \n"
        "1: \n"
        "   LDMFD %[threadSP]!, {r0} \n"
        "   STR r0, [%[target]], #4 \n"
        "   SUBS r1, #1 \n"
        "   BNE 1b \n"
        /* r0-r3 */
        "   SUB %[target], #48 \n"
        "   MOVS r1, #4 \n"
        "2: \n"
        "   LDMFD %[threadSP]!, {r0} \n"
        "   STR r0, [%[target]], #4 \n"
        "   SUBS r1, #1 \n"
        "   BNE 2b \n"
        /* r12, lr, pc, psr */
        "   ADD %[target], #32 \n"
        "   MOVS r1, #4 \n"
        "3: \n"
        "   LDMFD %[threadSP]!, {r0} \n"
        "   STR r0, [%[target]], #4 \n"
        "   SUBS r1, #1 \n"
        "   BNE 3b \n"
        :
        : [threadSP] "r" (thread->sp),
            [target] "r" (reg)
        : "r0", "r1", "memory" );
    
    enableInterrupts();
}

bool Thread_isRunning(int idx)
{
    return (threads[idx].flags & THREAD_FLAG_ACTIVE);
}

void _contextSwitchBegin(void) __attribute__((naked));
void _contextSwitchBegin(void)
{
    asm volatile ( "CPSID i \n" );
    asm volatile (
        /* Check current thread */
        "   TST %[threadFlags], %[flagUseMSP] \n"
        "   BEQ 1f \n"
        
        /* Currently using MSP */
        "   MRS r0, msp \n"
        "   STMFD r0!, {r4-r11} \n"
        "   MSR msp, r0 \n"
        "   BX lr \n"
        
        /* Currently using PSP */
        "1: \n"
        "   MRS r0, psp \n"
        "   STMFD r0!, {r4-r11} \n"
        "   MSR psp, r0 \n"
        "   BX lr \n"
        
        /* R0 is loaded with the active stack now */
        : 
        : [threadFlags] "r" (currentThread->flags),
            [flagUseMSP] "I" (THREAD_FLAG_USE_MSP)
        : "r0" );
}

void _contextSwitchExecute(void) __attribute__((naked));
void _contextSwitchExecute(void)
{
    asm volatile (
        /* r0 is address of the current thread structure (returned from 
         * inner) */
         
        /* Load SP and flags */
        "   LDR r1, [r0], #4 \n"
        "   LDR r0, [r0] \n"

        /* Load high registers */
        "   LDMFD r1!, {r4-r11} \n"
        
        /* Check current thread */
        "   TST r0, %[flagUseMSP] \n"
        "   BEQ 1f \n"
        
        /* Return to MSP */
        "   MSR msp, r1 \n"
        "   MOV r0, #0xFFFFFFF9 \n"
        "   B 2f \n"
        
        /* Return to PSP */
        "1: \n"
        "   MSR psp, r1 \n"
        "   MOV r0, #0xFFFFFFFD \n"
        
        /* r0 loaded with magic return value now */
        "2: \n"
        "   CPSIE i \n"
        "   BX r0 \n"
        :
        : [flagUseMSP] "I" (THREAD_FLAG_USE_MSP)
        : "r0", "r1", "memory" );
}

static void pendsv_handler(void) __attribute__((naked));
static void pendsv_handler(void)
{
    asm volatile (
        "   BL _contextSwitchBegin \n"
        "   MOVS r1, #1 \n"
        "   BL _contextSwitchInner \n"
        "   B _contextSwitchExecute \n"
        :
        : 
        : "r0", "r1", "lr" );
}

static void systick_handler(void) __attribute__((naked));
static void systick_handler(void)
{
    asm volatile (
        "   BL _contextSwitchBegin \n"
        "   MOVS r1, #0 \n"
        "   BL _contextSwitchInner \n"
        "   B _contextSwitchExecute \n"
        :
        :
        : "r0", "r1", "lr" );
}

void _faultInitializeStack(void) __attribute__((naked));
void _faultInitializeStack(void)
{
    asm volatile (
        "   CPSID i \n"
        "   CPSID f \n" );
    asm volatile (
        /* Reset MSP to a sane value */
        "   MOV sp, %0 \n"
        
        /* Make sure the fauling stack didn't smash the kernel stack */
        "   CMP r0, %0\n"
        "   BLE 3f \n"
        
        /* Check if the fauling stack has enough room to read the exception
         * entry frame before it. */
        "   MOVW r1, %[addRAMSize] \n"
        "   ADD %0, r1 \n"
        "   CMP r0, %0 \n"
        "   BGT 3f \n"
        
        /* Now done with passed in registers, so recycle them */
        
        /* Stacks are sane so store the debug info */
        "   SUB sp, %[debugSize] \n"
        /* Original SP */
        "   STR r0, [sp], #4 \n"
        /* r0-r3 */
        "   MOVS r1, #4 \n"
        "1: \n"
        "   LDMFD r0!, {%0} \n"
        "   STR %0, [sp], #4 \n"
        "   SUBS r1, #1 \n"
        "   BNE 1b \n"
        /* r4-r11 */
        "   STMIA sp, {r4-r11} \n"
        "   ADD sp, #32 \n"
        /* r12, lr, pc, psr */
        "   MOVS r1, #4 \n"
        "2: \n"
        "   LDMFD r0!, {%0} \n"
        "   STR %0, [sp], #4 \n"
        "   SUBS r1, #1 \n"
        "   BNE 2b \n"
        /* Restore SP */
        "   SUB sp, %[debugSize] \n"
        "   B 5f\n"
        
        /* Faulting SP wasn't sane, so just write zeros to debug */ 
        "3: \n"
        /* Fill with zeros */
        "   MOVS %0, #0 \n"
        "   MOV r1, %[fillRegisters] \n"
        "4: \n"
        "   PUSH {%0} \n"
        "   SUBS r1, #1 \n"
        "   BNE 4b \n"
        /* Original SP */
        "   PUSH {r0} \n"
        
        "5: \n"
        /* Set r0 to point to the start of the stacked info */
        "   MOV r0, sp \n"
        "   BX lr\n"
        
        :
        : "r" ((uint32_t)&_stack), 
            [fillRegisters] "I" (sizeof(DebugRegisters) / 4 - 1),
            [debugSize] "I" (sizeof(DebugRegisters)),
            [addRAMSize] "I" ((RAM_SIZE - 1024) - 64)
        : "r0", "r1", "memory" );
}

void _faultRecover(void) __attribute__((naked));
void _faultRecover(void)
{
    asm volatile (
        /* Pop temporary fault debug data off */
        "   ADD sp, %[debugSize] \n"
        
        /* PSR */
        "   MOV r0, #0x01000000 \n"
        "   PUSH {r0} \n"
        
        /* Target PC */
        "   PUSH {%[targetAddress]} \n"
        
        /* Push zero on for r0-r3, r12, lr */
        "   MOVS r1, #6 \n"
        "   MOVS r0, #0 \n"
        "1: \n"
        "   PUSH {r0} \n"
        "   SUBS r1, #1 \n"
        "   BNE 1b \n"
        
        /* Return to MSP */
        "   MOV r0, #0xFFFFFFF9 \n"
        "   CPSIE f \n"
        "   BX r0 \n"
        :
        : [targetAddress] "r" ((void *)runFaultEntry),
            [debugSize] "I" (sizeof(DebugRegisters))
        : "r0", "r1"  );
}

void _faultHandlerInner(DebugRegisters *reg, SystemRunState state);

#define faultHandlerGeneric(name, code) \
static void name(void) __attribute__((naked)); \
static void name(void) \
{ \
    asm volatile ( \
        /* Use the magic return value to put the right stack pointer in r0 */ \
        "   TST LR, #0x4 \n" \
        "   ITE EQ \n" \
        "   MRSEQ r0, msp \n" \
        "   MRSNE r0, psp \n" \
        "   BL _faultInitializeStack \n" \
        /* r0 set to register pointer by initialize, set r1 to the type */ \
        "   MOV r1, %[faultType] \n" \
        "   BL _faultHandlerInner \n" \
        "   B _faultRecover \n" \
        : \
        : [faultType] "I" (code) \
        : "r0", "r1", "sp"); \
}

faultHandlerGeneric(fault_nmi, RunState_Fault_NMI)
faultHandlerGeneric(fault_hard, RunState_Fault_Hard)
faultHandlerGeneric(fault_memory, RunState_Fault_Memory)
faultHandlerGeneric(fault_bus, RunState_Fault_Bus)
faultHandlerGeneric(fault_usage, RunState_Fault_Usage)

static void null_handler(void) { }

static void exti_handler(void)
{
    Transport_irqEXTI();
    
    if (systemStatus.state == RunState_UserExecuting &&
            systemStatus.detailed.normal.extiHandler)
        (systemStatus.detailed.normal.extiHandler)();
}

__attribute__ ((section(".vectors")))
void (*const vector_table[]) (void) = {
    (void*)&_stack,
    run,                /* Reset */
    fault_nmi,          /* NMI */
    fault_hard,         /* Hard fault */
    fault_memory,       /* Memory fault */
    fault_bus,          /* Bus fault */
    fault_usage,        /* Usage fault */
    0, 0, 0, 0,         /* Reserved */
    null_handler,       /* SVCall */
    0, 0,               /* Reserved */
    pendsv_handler,     /* PendSV */
    systick_handler,	/* Systick */

    null_handler,       /* WWD */
    null_handler,       /* PVD */
    null_handler,       /* Tamper */
    null_handler,       /* RTC */
    null_handler,       /* Flash */
    null_handler,       /* RCC */
    null_handler,       /* EXTI 0 */
    null_handler,       /* EXTI 1 */
    null_handler,       /* EXTI 2 */
    null_handler,       /* EXTI 3 */
    null_handler,       /* EXTI 4 */
    null_handler,       /* DMA Channel 1 */
    Transport_irqRxDMA, /* DMA Channel 2 */
    null_handler,       /* DMA Channel 3 */
    null_handler,       /* DMA Channel 4 */
    null_handler,       /* DMA Channel 5 */
    null_handler,       /* DMA Channel 6 */
    null_handler,       /* DMA Channel 7 */
    null_handler,       /* ADC 1, 2 */
    null_handler,       /* USB HP/CAN Tx */
    null_handler,       /* USB LP/CAN Rx */
    null_handler,       /* CAN Rx1 */
    null_handler,       /* CAN SCE */
    null_handler,       /* EXTI 5-9 */
    null_handler,       /* TIM1 BRK */
    null_handler,       /* TIM1 UP */
    null_handler,       /* TIM1 Trigger common */
    null_handler,       /* TIM1 CC */
    null_handler,       /* TIM2 */
    null_handler,       /* TIM3 */
    null_handler,       /* TIM4 */
    null_handler,       /* I2C1 EV */
    null_handler,       /* I2C1 ER */
    null_handler,       /* I2C2 EV */
    null_handler,       /* I2C2 ER */
    null_handler,       /* SPI1 */
    null_handler,       /* SPI2 */
    null_handler,       /* USART1 */
    null_handler,       /* USART2 */
    null_handler,       /* USART3 */
    exti_handler,       /* EXTI 10-15 */
#if 0
    null_handler,       /* RTC Alarm */
    null_handler,       /* USB wakeup */
    null_handler,       /* TIM8 BRK */
    null_handler,       /* TIM8 UP */
    null_handler,       /* TIM8 Trigger common */
    null_handler,       /* TIM8 CC */
    null_handler,       /* ADC 3 */
    null_handler,       /* FSMC */
    null_handler,       /* SDIO */
    null_handler,       /* TIM5 */
    null_handler,       /* SPI3 */
    null_handler,       /* UART4 */
    null_handler,       /* UART5 */
    null_handler,       /* TIM6 */
    null_handler,       /* TIM7 */
    null_handler,       /* DMA2 Channel 1 */
    null_handler,       /* DMA2 Channel 2 */
    null_handler,       /* DMA2 Channel 3 */
    null_handler,       /* DMA2 Channel 4,5 */
    null_handler,       /* DMA2 Channel 5 */
    null_handler,       /* Ethernet */
    null_handler,       /* Thernet wakeup */
    null_handler,       /* CAN2 Tx */
    null_handler,       /* CAN2 Rx0 */
    null_handler,       /* CAN2 Rx1 */
    null_handler,       /* CAN2 SCE */
    null_handler,       /* USB OTG FS */
#endif
};

void _faultHandlerInner(DebugRegisters *reg, SystemRunState state)
{
    {
        uint32_t saveUptime = uptime;
        initializeData();
        memcpy(&systemStatus.detailed.userFaulted.debug.fault.registers, reg, 
            sizeof(DebugRegisters));
        uptime = saveUptime;
    }
    systemStatus.detailed.userFaulted.retryTime = 300;
    systemStatus.state = state;
    
    systemStatus.detailed.userFaulted.debug.fault.data.SHCSR = SCB_SHCSR;
    systemStatus.detailed.userFaulted.debug.fault.data.CFSR = SCB_CFSR;
    systemStatus.detailed.userFaulted.debug.fault.data.HFSR = SCB_HFSR;
    systemStatus.detailed.userFaulted.debug.fault.data.DFSR = SCB_DFSR;
    systemStatus.detailed.userFaulted.debug.fault.data.MMFAR = SCB_MMFAR;
    systemStatus.detailed.userFaulted.debug.fault.data.BFAR = SCB_BFAR;
    systemStatus.detailed.userFaulted.debug.fault.data.AFSR = SCB_AFSR;
    
    SCB_VTOR = (uint32_t)(&vector_table[0]);
    vectorTableChanged();
}

static void softwareWatchdogTripped(void)
{
    disableInterrupts();
    Kernel_disableUserPeripherals();
        
    threads[0].flags |= THREAD_FLAG_USE_MSP;
    for (int i=1; i<MAXIMUM_THREADS; i++)
        threads[i].flags = 0;
        
    SCB_VTOR = (uint32_t)(&vector_table[0]);
    
    memset(&systemStatus.detailed.userFaulted, 0, 
                sizeof(systemStatus.detailed.userFaulted));
    systemStatus.detailed.userFaulted.retryTime = 300;
    systemStatus.state = RunState_FailureHalted_SoftwareWatchdog;
    
    vectorTableChanged();
    enableInterrupts();
}

void Kernel_haltUserCode(void) 
{
    Transport_waitIdle();
    
    if (systemStatus.state != RunState_UserExecuting) {
        enableInterrupts();
        return;
    }

    asm volatile (
        /* Set MSP to the current stack */
        "   MOV r0, sp \n"
        "   MSR msp, r0 \n"
        
        /* Switch to MSP for the kernel */
        "   MOVS r0, #0x0 \n"
        "   MSR control, r0 \n"
        :
        :
        : "r0");
        
    Kernel_disableUserPeripherals();
        
    threads[0].flags |= THREAD_FLAG_USE_MSP;
    for (int i=1; i<MAXIMUM_THREADS; i++)
        threads[i].flags = 0;
        
    SCB_VTOR = (uint32_t)(&vector_table[0]);
    
    systemStatus.state = RunState_UserHalted;
    
    vectorTableChanged();
    enableInterrupts();
    
    /* Do this after interrupts are enabled again, since it requires using
     * the CC3000 */
    Server_closeUserSockets();
}
