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

#ifndef _SYNCHRONIZATION_H
#define _SYNCHRONIZATION_H

#include <api.h>

#ifndef _API_CALL
#define _API_CALL
#endif

void semaphore_init(Semaphore *s, uint32_t n) _API_CALL;
void semaphore_V(Semaphore *s, uint32_t n) _API_CALL;
void semaphore_P(Semaphore *s, uint32_t n) _API_CALL;

void mutex_init(Mutex *m) _API_CALL;
void mutex_lock(Mutex *m) _API_CALL;
void mutex_unlock(Mutex *m) _API_CALL;

void cv_init(ConditionVariable *cv) _API_CALL;
void cv_wait(ConditionVariable *cv, Mutex *m) _API_CALL;
void cv_wake(ConditionVariable *cv) _API_CALL;

#endif
