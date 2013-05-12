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

#include "synchronization.h"
#include "kernel.h"

void semaphore_init(Semaphore *s, uint32_t n)
{
    s->s = n;
}

void semaphore_V(Semaphore *s, uint32_t n) __attribute__((naked));
void semaphore_V(Semaphore *s, uint32_t n) 
{
    (void)s; (void)n;
    asm volatile (
        /* Just keep trying until we can add to it, we can only ever fail due
         * to concurrent access, so we can try again right away */
        "1: \n"
        "   LDREX r2, [r0] \n"
        "   ADD r2, r1 \n"
        "   STREX r3, r2, [r0] \n"
        "   CMP r3, #0 \n"
        "   BNE 1b \n"
        "   BX lr \n"
        :
        :
        : "r0", "r1", "r2", "r3", "memory" );
}

void semaphore_P(Semaphore *s, uint32_t n) __attribute__((naked));
void semaphore_P(Semaphore *s, uint32_t n) 
{
    (void)s; (void)n;
    asm volatile (
        /* Try acquire */
        "1: \n"
        "   LDREX r2, [r0] \n"
        "   SUBS r2, r1 \n"
        "   BLT 2f \n"
        "   STREX r3, r2, [r0] \n"
        "   CMP r3, #0 \n"
        /* Don't sleep on a write contest, just try again */
        "   BNE 1b \n"
        "   BX lr \n"
        
        /* Can't acquire, so wait */
        "2: \n"
        "   CLREX \n"
        "   PUSH {r0, r1, lr} \n"
        "   BL Thread_yield \n"
        "   POP {r0, r1, lr} \n"
        "   B 1b \n"
        :
        :
        : "r0", "r1", "r2", "r3", "memory");
}

void mutex_init(Mutex *m) 
{ 
    semaphore_init(&m->sem, 1); 
}
void mutex_lock(Mutex *m)
{
    semaphore_P(&m->sem, 1);
}
void mutex_unlock(Mutex *m)
{
    semaphore_V(&m->sem, 1);
}

/* Condition variables are just implemented by waiting for a counter to change.
 * This works because spurious wakes are tolerated, but not sleeps.  It also
 * works with a 32-bit one because it's not possible to miss a service with
 * the number of threads we can possibly support versus the maximum execution
 * rate of the processor.  That is, we're guaranteed to be serviced from the
 * round-robin scheduler before the counter could possibly wrap around back 
 * to the same value even if all other threads are doing nothing but calling
 * wake in a loop. */

void cv_init(ConditionVariable *cv) 
{
    cv->data = 0;
}

/* We don't actually need exclusive access on the wait because the worst case
 * is that we sleep an extra schedule spin, which is fine. */
void cv_wait(ConditionVariable *cv, 
                            Mutex *m ) __attribute__((naked));
void cv_wait(ConditionVariable *cv, Mutex *m) 
{
    (void)cv; (void)m;
    asm volatile(
        "   PUSH {r4, lr} \n"
        /* Read initial counter */
        "   LDR r4, [r0] \n"
        
        /* Loop until we get a change in the value */
        "1: \n"
        "   LDR r2, [r0] \n"
        "   CMP r2, r4 \n"
        "   IT NE \n"
        "   POPNE {r4, pc} \n"
        
        /* Release the mutex, yield, then re-acquire and try again */
        "   PUSH {r0} \n"
        "   PUSH {r1} \n"
        "   MOV r0, r1 \n"
        "   BL mutex_unlock \n"
        "   BL Thread_yield \n"
        "   POP {r0} \n"
        "   BL mutex_lock \n"
        "   MOV r1, r0 \n"
        "   POP {r0} \n"
        "   B 1b \n"
        
        :
        :
        : "r0", "r1", "r2", "r4" );
}

void cv_wake(ConditionVariable *cv) __attribute__((naked));
void cv_wake(ConditionVariable *cv)
{
    (void)cv;
    asm volatile(
        /* Just spin if we fail the write back, since that just means
         * a write contest, so no sleeping needed */
        "1: \n"
        "   LDREX r1, [r0] \n"
        "   ADD r1, #1 \n"
        "   STREX r2, r1, [r0] \n"
        "   CMP r2, #0 \n"
        "   BNE 1b \n"
        "   BX lr \n"
        :
        :
        : "r0", "r1", "r2", "memory" );
}
