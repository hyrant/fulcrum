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

#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include <api.h>
#include "kernel.h"

#ifndef LISTEN_PORT
#define LISTEN_PORT 80
#endif

#ifndef PAGE_TITLE
#define PAGE_TITLE  "Fulcrum"
#endif

/**
 * Test if there is valid user code.
 * 
 * @return true if there is valid user code
 */
bool Control_haveUserCode(void);

/**
 * Get the length of the user code.
 * 
 * @return the user code length in bytes
 */
uint32_t Control_userCodeLength(void);

/**
 * Test if there is an authorization requirement set.
 * 
 * @return true if authorization is required
 */
bool Control_requireAuthorization(void);

/**
 * Test if the given encoded authorization matches.
 * 
 * @param encoded   the base64 encoded authorization string
 * @return          true if the authorization is accepted
 */
bool Control_checkAuthorization(const char *encoded);

/**
 * Test if the network is operational.
 * 
 * @return true if the network is up
 */
bool Control_networkUp(void);

/**
 * The main control loop.  Does not return.
 */
void Control_main(void);

typedef enum {
    /** System currently offline and initializing the network layer. */
    Offline_Spinup = 0,
    /** System currently offline waiting for the network to come online. */
    Offline_WaitingForInitialize,
    /** System currently offline because too many user faults where pending,
     * the network will be restarted when control returns to the kernel. */
    Offline_TooManyUserFaultsPending,
    /** System currently offline and awaiting a restart when control returns
     * to the kernel. */
    Offline_RestartPending,
    
    /** System entering setup mode. */
    Setup_ModeEnter,
    /** System waiting for the network layer to come online so the parameters
     * for setup can be set (e.x. start simpleconfig) */
    Setup_ModeInitializing,
    /** System in processing for setup mode.  Currently handling UART data and
     * waiting for a simple config done message. */
    Setup_Mode,
    /** Currently applying parameters (and restarting the network) */
    Setup_Apply,
    /** Simple config message received, connection attempt will begin
     * when control returns. */
    Setup_SimpleConfigDone,
    /** Setup done and waiting for a connection before finalizing and entering
     * normal operations. */
    Setup_Connecting,
    
    /** System idle and waiting for connections. */
    Idle,
    
    /** In the initialization state for a GET request (reading the URL) */
    Initialize_GET,
    /** In the initialization state for a POST request (reading the URL) */
    Initialize_POST,
    
    /** Index GET request, awaiting authorization if needed. */
    GET_Index,
    /** Thread debug GET request, awaiting authorization if needed. */
    GET_Thread0,
    GET_ThreadN = GET_Thread0 + MAXIMUM_THREADS,
    /** Kernel RAM dump GET request, awaiting authorization if needed. */
    GET_RAM_Kernel,
    /** User RAM dump GET request, awaiting authorization if needed. */
    GET_RAM_User,
    /** Global RAM dump GET request, awaiting authorization if needed. */
    GET_RAM_All,
    /** Kernel ROM dump GET request, awaiting authorization if needed. */
    GET_ROM_Kernel,
    /** User ROM dump GET request, awaiting authorization if needed. */
    GET_ROM_User,
    /** Global ROM dump GET request, awaiting authorization if needed. */
    GET_ROM_All,
    /** Stop user code POST request, awaiting authorization if needed. */
    POST_StopUser,
    /** Start user code POST request, awaiting authorization if needed. */
    POST_StartUser,
    /** Issue a low level system reset, awaiting authorization if needed. */
    POST_RebootSystem,
    /** User code upload post request in progress. */
    POST_UserCodeUpload,
    
    /** In the middle of a bulk memory send (initiated from one of the RAM/ROM
     * requests above). */
    Upload_Memory,
    
    /** Have a GET request that was handled by the user code callback, but
     * required authorization, so waiting for that. */
    User_GET_AuthorizationRequired,
    /** Have a GET request that was handled by the user code callback, but
     * required authorization, so waiting for that.  This disables standard
     * headers output */
    User_GET_AuthorizationRequired_Direct,
    
    /** Handling a full raw mode user code GET request. */
    User_GET_Raw,
    /** Handling a full raw mode user code POST request. */
    User_POST_Raw,
    
    /** Handling a generalized POST request from the user code callback. */
    User_POST,
} ControlState;
extern ControlState controlState;

typedef struct {
    uint8_t length;
    /* Extra for the CRLF-- at the start */
    char data[MAXIMUM_MULTIPART_BOUNDARY+5];
} ControlParseBuffer;

typedef struct {
    enum {
        /** Currently processing the headers at the start of the POST request,
         * waiting for one we need to care about. */
        POST_Header_Generic,
        /** Got the Content-Type header. */
        POST_Header_ContentType,
        /** Content type indicated multipart so currently reading the
         * boundary marker into the global buffer. */
        POST_Header_ContentType_MultipartBoundary,
        /** Got the Content-Length header, so currently reading the length. */
        POST_Header_ContentLength,
        /** Got the Transfer-Encoding header, so currently reading the
         * type (specifically chunked or not). */
        POST_Header_TransferEncoding,        
        /** Got the authorization header, so currently reading the type. */
        POST_Header_Authorization,
        /** Got Basic authorization so currently reading the string. */
        POST_Header_Authorization_Basic,
        
        /** In the body of a URL encoded request, reading the field name. */
        POST_URL_Field,
        /** In the body of a URL encoded request, reading the field data. */
        POST_URL_Data,
        
        /** In the body of a multipart request, waiting for the initial 
         * boundary marker. */
        POST_Multipart_InitialBoundary,
        /** Got a boundary marker, so waiting for the end of it (either a
         * CRLF or a '--' marking the end of the entire request. */
        POST_Multipart_Boundary_Finish,        
        /** Reading internal headers after a boundary marker. */
        POST_Multipart_Header,
        /** Got a Content-Disposition internal header, so waiting for
         * the name field. */
        POST_Multipart_Header_ContentDisposition,
        /** Got the name field in the header, so reading the name content. */
        POST_Multipart_Header_ContentDisposition_Name,
        /** Finishing off the end of a header. */
        POST_Multipart_Header_Finish,
        /** In the body of the multipart request, sending data along and
         * waiting for another boundary marker. */
        POST_Multipart_Body,
    } state;
    
    union {
        struct {
            /** Authorization required to actually process. */
            bool authorizationRequired;
            /** Valid authorization received. */
            bool authorized;
            /** Multipart encoded incoming. */
            bool multipart;
        } initialize;
        struct {
            /** Have had a name field in the headers. */
            bool hadName;
        } multipart;
    } data;
    
    /** Output the standard text headers before calling the process function. */
    bool outputStandardHeaders;
    /** Using chunked length encoding. */
    bool chunked;
    /** The number of bytes remaining available in the total request or
     * the current segment for chunked encoding. */
    uint32_t remainingLength;
} ControlPOSTData;

typedef enum {
    /** Waiting for the authorization header. */
    SimpleAuth_WaitHeader,
    /*** Waiting for the type of authorization (Basic) */
    SimpleAuth_WaitType,
    /** Matching the authorization data to what we require. */
    SimpleAuth_Matching,
} ControlSimpleAuthorization;

typedef union {
    struct {
        ControlParseBuffer buffer;
        ControlPOSTData post;
    } base;
    
    struct {
        ControlParseBuffer buffer;
        ControlSimpleAuthorization authorization;
    } simpleAuthorization;
    
    struct {
        ControlParseBuffer buffer;
        HTTP_GETCallback callback;
        ControlSimpleAuthorization authorization;
    } userGET;
    
    struct {
        ControlParseBuffer buffer;
        ControlPOSTData post;
        HTTP_POSTCallback callback;
    } userPOST;
    
    struct {
        ControlParseBuffer buffer;
        ControlPOSTData post;
        uint32_t address;
        uint8_t nPending;
        uint8_t pending[4];
    } userCodeUpload;
    
    struct {
        uint32_t address;
        uint32_t remaining;
    } staticSend;
} ControlData;
extern ControlData controlData;

extern uint32_t controlStateStartTime;

/**
 * Indicate that a fault in networking has occurred.  This will trigger
 * a re-opening of the server socket and may stop the user code if
 * too many faults occur.
 */
void Control_networkFault(void);

/** 
 * The handler for the user code upload POST request field name.
 * 
 * @param name  the field name
 */
void Control_writeUserCodeField(const char *name);

/**
 * The handler for the user code upload POST request data.
 * 
 * @param data  the data to add
 * @param n     the length
 */
void Control_writeUserCodeData(const void *data, uint32_t n);

/**
 * The completion handler for the user code upload POST request.
 * 
 * @param fd    the client socket
 */
void Control_writeUserCodeCompleted(int fd);

/**
 * Set the encoded authorization string.
 * 
 * @param encoded   the encoded string
 */
void Control_setAuthorization(const char *encoded);

/** @see nvmem_write(NetAPINVMemFileID, const void *, uint32_t, uint32_t) */
int User_nvmem_write(NetAPINVMemFileID file, const void *buffer, 
                     uint32_t length, uint32_t seek);
/** @see nvmem_create(NetAPINVMemFileID, uint32_t) */
int User_nvmem_create(NetAPINVMemFileID file, uint32_t length);

#endif
