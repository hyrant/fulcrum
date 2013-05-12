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

#ifndef _API_H
#define _API_H

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push,4)

typedef struct {
    uint32_t s;
} Semaphore;
#define SEMAPHORE_INITIALIZER(n)    { .s = n }

typedef struct {
    Semaphore sem;
} Mutex;
#define MUTEX_INITIALIZER           { .sem = SEMAPHORE_INITIALIZER(1) }

typedef struct {
    uint32_t data;
} ConditionVariable;
#define CONDITION_VARIABLE_INITIALIZER  { .data = 0 }

#pragma pack(pop)

/** The type of the callback issued on a network system reset */
typedef void (*NetworkReset_Handler)(void);

/** The type of the callback issued when a TCP socket is remotely terminated */
typedef void (*TCPClosed_Handler)(void);

/** The type for the callback on the EXTI 10-15 handler */
typedef void (*EXTI10_15_Handler)(void);


/** The types of handling provided for a hooked HTTP request */
typedef enum {
    /** The request is not handled by user code (usually results in a 404) */
    HTTP_Unhandled = 0,
    /** The request is handled by the user code. */
    HTTP_Handled,
    /** Same as HTTP_Handled, but the stock header and footer are not
     * output.  The user code should provide all headers (including the
     * "200 OK") the trailing CRLF and the body content. */
    HTTP_Handled_Direct,
    /** The request is handled by the user code but requires authentication
     * if available. */
    HTTP_AuthorizationRequired,
    /** Same as HTTP_AuthorizationRequired, but the stock header and footer 
     * are not output.  The user code should provide all headers (including the
     * "200 OK") the trailing CRLF and the body content. */
    HTTP_AuthorizationRequired_Direct,
    /** The request is handled by the user code in raw mode. */
    HTTP_Raw,
} HTTP_Status;


/**
 * The handler called to send data to the given socket.
 * @param fd    the output socket
 */
typedef void (*HTTP_SendData)(int fd);

/**
 * The handler called when in raw mode.  This is called every time there is
 * data read.  When finished it should return false and the kernel will
 * close the socket.  This will be called with a zero length buffer and a
 * socket of -1 if the connection was closed.
 * 
 * @param data      the incoming data buffer
 * @param n         the size of the incoming data
 * @param fd        the client socket
 * @return          false once handling is completed
 */
typedef bool (*HTTP_RawHandler)(const void *data, uint32_t n, int fd);

/** The callbacks for a GET request. */
typedef union {
    HTTP_SendData data;
    HTTP_RawHandler raw;
} HTTP_GETCallback;

/** 
 * The type of the handler called after the initial line is received for
 * a GET request. 
 * 
 * @param path      the path requested
 * @param callback  the callback to use if the status is not unhandled
 * @return          the handler status
 */
typedef HTTP_Status (*HTTP_GET)(const char *path, HTTP_GETCallback *callback);

/**
 * Called when a field begins in a post request.
 * 
 * @param name      the field name
 */
typedef void (*HTTP_POSTFieldBegin)(const char *name);

/**
 * Called as data is received during a post request.
 * 
 * @param data      the incoming data buffer
 * @param n         the size of the incoming data
 */
typedef void (*HTTP_POSTFieldData)(const void *data, uint32_t n);

/**
 * Called when the post request is completed.  If the connection has already
 * been closed then the socket will be -1.
 * 
 * @param fd        the socket
 */
typedef void (*HTTP_POSTCompleted)(int fd);

/**
 * The callbacks for a POST request.
 */
typedef union {
    struct {
        HTTP_POSTFieldBegin begin;
        HTTP_POSTFieldData field;
        HTTP_POSTCompleted completed;
    } data;
    HTTP_RawHandler raw;
} HTTP_POSTCallback;

/** 
 * The type of the handler called after the initial line is received for
 * a POST request. 
 * 
 * @param path      the path requested
 * @param callback  the callback to use if the status is not unhandled
 * @return          the handler status
 */
typedef HTTP_Status (*HTTP_POST)(const char *path, HTTP_POSTCallback *callback);

#endif
