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
#include <string.h>

#include "kernel.h"
#include "control.h"
#include "server.h"
#include "util.h"

static char *startOfNextParameter(char *begin)
{
    for (; *begin == ' ' || *begin == '\t'; ++begin) { }
    if (*begin == ';') {
        ++begin;
        for (; *begin == ' ' || *begin == '\t'; ++begin) { }
    }
    return begin;
}

static void beginPOSTData(void)
{
    if (controlData.base.post.data.initialize.authorizationRequired && 
            !controlData.base.post.data.initialize.authorized) {
        Server_unauthorized();
        return;
    }
    
    if (controlData.base.post.data.initialize.multipart) {
        controlData.base.post.state = POST_Multipart_InitialBoundary;
    } else {
        controlData.base.post.state = POST_URL_Field;
    }
    
    if (!controlData.base.post.chunked && 
            controlData.base.post.remainingLength == 0) {
        controlData.base.post.remainingLength = 0xFFFFFFFF;
    }
}
static void readInitialBoundary(uint32_t lengthAdd)
{
    if (lengthAdd + systemStatus.detailed.normal.postBufferLength > 
            sizeof(systemStatus.detailed.normal.postBuffer)) {
        lengthAdd = sizeof(systemStatus.detailed.normal.postBuffer) -
            systemStatus.detailed.normal.postBufferLength;
    }
    memcpy(&systemStatus.detailed.normal.postBuffer[
        systemStatus.detailed.normal.postBufferLength],
        controlData.base.buffer.data, lengthAdd);
    systemStatus.detailed.normal.postBufferLength += lengthAdd;
}

static void postRequestFinish(HTTP_POSTCompleted completed)
{
    if (controlData.base.post.outputStandardHeaders) {
        Server_standardTextBegin();
    }
    (completed)(httpClientSocket);
    if (controlData.base.post.outputStandardHeaders) {
        Server_standardTextFooters();
    }
    Server_connectionCompleted();
}

static int postDataBlockBegin(HTTP_POSTCompleted completed)
{
    while (true) {
        if (controlData.base.post.remainingLength > 
                controlData.base.buffer.length)
            return controlData.base.buffer.length;
        
        if (controlData.base.post.chunked) {
            char *begin = &controlData.base.buffer.data[
                controlData.base.post.remainingLength];
            char *ptr = begin;
            char *end = &controlData.base.buffer.data[
                controlData.base.buffer.length];
            for (; ptr < end && (*ptr == '\r' || *ptr == '\n'); 
                    ++ptr) { }
            char *lineEnd = Server_findLineEnd(ptr, end);
            if (lineEnd == NULL) {
                if (httpClientSocket == -1) {
                    postRequestFinish(completed);
                    return 0;
                }
            } else {            
                uint32_t len = UTIL_parseHexadecimal(ptr);
                if (len != 0) {
                    Util_shiftConsumeBuffer(begin, lineEnd, end - begin);
                    controlData.base.buffer.length -= lineEnd - begin;
                    controlData.base.post.remainingLength += len;
                    continue;
                }
            }
        }
        
        if (controlData.base.post.remainingLength == 0) {
            postRequestFinish(completed);
            return 0;
        }
        
        return controlData.base.post.remainingLength;
    }
}
static bool postConsumeData(char *end, HTTP_POSTCompleted completed)
{
    Server_consumeBuffer(end);
    
    uint32_t n = end - &controlData.base.buffer.data[0];
    if (n >= controlData.base.post.remainingLength) {
        controlData.base.post.remainingLength = 0;
        /* Chunked data just needs to get another chunk length */
        if (controlData.base.post.chunked)
            return true;
            
        postRequestFinish(completed);
        return false;
    }
    
    controlData.base.post.remainingLength -= n;
    return true;
}

static bool beginMultipartDataSegment(char *end, HTTP_POSTFieldBegin begin,
                                      HTTP_POSTCompleted completed)
{
    if (!controlData.base.post.data.multipart.hadName)
        (begin)("");
    controlData.base.post.data.multipart.hadName = false;
    controlData.base.post.state = POST_Multipart_Body;
    return postConsumeData(end, completed);
}

static bool multpartBoundaryMatch(char *begin, char *end,
                                  HTTP_POSTFieldData data,
                                  HTTP_POSTCompleted completed)
{
    if ((end-begin) < systemStatus.detailed.normal.postBufferLength)
        return false;
    for (char *compare=systemStatus.detailed.normal.postBuffer,
            *endCompare=compare+systemStatus.detailed.normal.postBufferLength;
            compare != endCompare; ++compare, ++begin) {
        if (*compare != *begin) {
            /* We know this can't be a CR/LF so we can send it along too */
            ++begin;
            
            (data)(controlData.base.buffer.data, begin - 
                &controlData.base.buffer.data[0]);
            return postConsumeData(begin, completed);
        }
    }
    
    controlData.base.post.state = POST_Multipart_Boundary_Finish;
    return postConsumeData(begin, completed);
}

static char *nextLineBreak(char *begin, int available)
{
    for (; available > 0; --available, ++begin) {
        if (*begin == '\r' || *begin == '\n')
            return begin;
    }
    return NULL;
}

static bool genericPOSTProcess(HTTP_POSTFieldBegin begin, 
                               HTTP_POSTFieldData data,
                               HTTP_POSTCompleted completed)
{
    char *ptr;
    switch (controlData.base.post.state) {
    case POST_Header_Generic:
        if ((ptr=Server_bufferStartsWith("Content-Type:")) != NULL) {
            Server_consumeBuffer(ptr);
            controlData.base.post.state = POST_Header_ContentType;
            return true;
        } else if ((ptr=Server_bufferStartsWith("Content-Length:")) != NULL) {
            Server_consumeBuffer(ptr);
            controlData.base.post.state = POST_Header_ContentLength;
            return true;
        } else if ((ptr=Server_bufferStartsWith("Transfer-Encoding:")) != NULL) {
            Server_consumeBuffer(ptr);
            controlData.base.post.state = POST_Header_TransferEncoding;
            return true;
        } else if ((ptr=Server_bufferStartsWith("Authorization:")) != NULL) {
            Server_consumeBuffer(ptr);
            controlData.base.post.state = POST_Header_Authorization;
            return true;
        } else if (Server_headersEnded()) {
            beginPOSTData();
            if (httpClientSocket == -1) {
                (completed)(-1);
                return false;
            }
            return true;
        } 
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        Server_consumeBuffer(ptr);
        return true;
        
    case POST_Header_ContentType:
        if ((ptr=Server_bufferStartsWith("multipart/")) != NULL) {
            char *fieldCheck = Server_endOfCurrentField();
            fieldCheck = startOfNextParameter(fieldCheck);
            fieldCheck = Util_bufferStartsWith("boundary=", fieldCheck);
            if (fieldCheck != NULL) {            
                Server_consumeBuffer(fieldCheck);
                controlData.base.post.data.initialize.multipart = true;
                controlData.base.post.state = 
                    POST_Header_ContentType_MultipartBoundary;
                
                switch (systemStatus.state) {
                case RunState_Setup:
                    systemStatus.state = RunState_Initialize;
                    break;
                
                case RunState_FailureHalted_SocketIO:
                case RunState_FailureHalted_SoftwareWatchdog:    
                case RunState_ResetHalted_LowPower:
                case RunState_ResetHalted_WindowWatchDog:
                case RunState_ResetHalted_IndependentWatchDog:
                case RunState_ResetHalted_SoftwareReset:    
                case RunState_Fault_NMI:
                case RunState_Fault_Hard:
                case RunState_Fault_Memory:
                case RunState_Fault_Bus:
                case RunState_Fault_Usage:
                    systemStatus.state = RunState_UserHalted;
                    break;
                default:
                    break;
                }
                systemStatus.detailed.normal.postBufferLength = 0;
                
                return true;
            }
        }
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        controlData.base.post.state = POST_Header_Generic;
        Server_consumeBuffer(ptr);
        return true;
    
    case POST_Header_ContentType_MultipartBoundary:
        if ((ptr = Util_findCharacter(controlData.base.buffer.data, 
                '\r','\n',0)) == NULL) {
            readInitialBoundary(controlData.base.buffer.length);
            memset(&controlData.base.buffer, 0, 
                sizeof(controlData.base.buffer));
            return false;
        } else {
            char *addLast = ptr;
            for (; addLast >= &controlData.base.buffer.data[0] &&
                    (*addLast == '\n' || *addLast == '\r'); --addLast) { }
            ++addLast;
            readInitialBoundary(addLast - &controlData.base.buffer.data[0]);
                        
            controlData.base.post.state = POST_Header_Generic;
            
            if (*ptr == '\n') {
                ++ptr;
                if (*ptr == '\r')
                    ++ptr;
            } else {
                ++ptr;
                if (*ptr == '\n')
                    ++ptr;
            }
            Server_consumeBuffer(ptr);
            return true;
        }
        
    case POST_Header_Authorization:
        if ((ptr=Server_bufferStartsWith("Basic")) != NULL) {
            Server_consumeBuffer(ptr);
            controlData.base.post.state = POST_Header_Authorization_Basic;
            return true;
        }
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        Server_consumeBuffer(ptr);
        controlData.base.post.state = POST_Header_Generic;
        return true;    
    case POST_Header_Authorization_Basic:
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;

        controlData.base.post.data.initialize.authorized = 
            Control_checkAuthorization(controlData.base.buffer.data);
        
        Server_consumeBuffer(ptr);
        controlData.base.post.state = POST_Header_Generic;
        return true;
        
    case POST_Header_ContentLength:
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        
        controlData.base.post.remainingLength = 
            Util_parseDecimal(controlData.base.buffer.data);
        Server_consumeBuffer(ptr);
        controlData.base.post.state = POST_Header_Generic;
        return true;
        
    case POST_Header_TransferEncoding:
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        if (Server_bufferMatches("chunked"))
            controlData.base.post.chunked = true;
        Server_consumeBuffer(ptr);
        controlData.base.post.state = POST_Header_Generic;
        return true;
        
    
    case POST_URL_Field: {
        int available = postDataBlockBegin(completed);
        if (available == 0)
            return false;
        if ((ptr = Util_findCharacter(controlData.base.buffer.data, 
                '=',0,0)) == NULL)
            return false;
        if (ptr >= &controlData.base.buffer.data[available])
            return false;
        for (char *replace=controlData.base.buffer.data; 
                replace<ptr; ++replace) {
            if (*replace == '+')
                *replace = ' ';
        }
        int len = (ptr - &controlData.base.buffer.data[0]);
        ++ptr;
        
        controlData.base.buffer.data[Util_replaceURLEncoding(
            controlData.base.buffer.data, &len)] = 0;
        (begin)(controlData.base.buffer.data);        
        controlData.base.post.state = POST_URL_Data;
        
        return postConsumeData(ptr, completed);
    }
    
    case POST_URL_Data: {
        int available = postDataBlockBegin(completed);
        if (available == 0)
            return false;
        ptr = controlData.base.buffer.data;
        int len;
        for (len=0; len<available; ++len, ++ptr) {
            if (*ptr == '&')
                break;
            if (*ptr == '+')
                *ptr = ' ';
        }
        
        bool fieldEnded = false;
        if (*ptr == '&') {
            controlData.base.post.state = POST_URL_Field;
            fieldEnded = true;
            ++ptr;
        }
        
        if (len != 0) {
            int output = Util_replaceURLEncoding(
                controlData.base.buffer.data, &len);
            (data)(controlData.base.buffer.data, output);
            
            if (!fieldEnded) {
                ptr = &controlData.base.buffer.data[
                    controlData.base.buffer.length - len];
            }
        }
        
        return postConsumeData(ptr, completed);
    }
    
    case POST_Multipart_InitialBoundary: {
        int available = postDataBlockBegin(completed);
        if (available < 3)
            return false;
        if ((ptr = Util_findCharacter(controlData.base.buffer.data, 
                '-',0,0)) == NULL) {
            return postConsumeData(&controlData.base.buffer.data[available], 
                completed);
        }
        if (ptr != &controlData.base.buffer.data[0]) {
            return postConsumeData(ptr, completed);
        }
        ++ptr;
        if (*ptr != '-') {
            return postConsumeData(ptr+1, completed);
        }
        ++ptr;
        if (available < systemStatus.detailed.normal.postBufferLength+2)
            return false;
        
        char *compare = systemStatus.detailed.normal.postBuffer;
        for (int i=0; i<systemStatus.detailed.normal.postBufferLength;
                ++i, ++ptr, ++compare) {
            if (*compare != *ptr)
                return postConsumeData(ptr, completed);
        }
        
        controlData.base.post.state = POST_Multipart_Boundary_Finish;
        controlData.base.post.data.multipart.hadName = false;
        return postConsumeData(ptr, completed);
    }
    
    case POST_Multipart_Boundary_Finish: {
        int available = postDataBlockBegin(completed);
        if (available < 2)
            return false;
        char check = controlData.base.buffer.data[0];
        if (check == '\r') {
            controlData.base.post.state = POST_Multipart_Header;
            if (controlData.base.buffer.data[1] == '\n') {
                return postConsumeData(&controlData.base.buffer.data[2], 
                    completed);
            }
            return postConsumeData(&controlData.base.buffer.data[1], 
                completed);
        } else if (check == '\n') {
            controlData.base.post.state = POST_Multipart_Header;
            if (controlData.base.buffer.data[1] == '\r') {
                return postConsumeData(&controlData.base.buffer.data[2], 
                    completed);
            }
            return postConsumeData(&controlData.base.buffer.data[1], 
                completed);
        } else if (check == '-' && controlData.base.buffer.data[1] == '-') {
            postRequestFinish(completed);
            return false;
        } else {
            return postConsumeData(&controlData.base.buffer.data[1], 
                completed);
        }
    }
    
    case POST_Multipart_Header: {
        int available = postDataBlockBegin(completed);
        if (available == 0)
            return false;
        if ((ptr=Server_bufferStartsWith("Content-Disposition:")) != NULL &&
                ptr < &controlData.base.buffer.data[available]) {
            controlData.base.post.state = 
                POST_Multipart_Header_ContentDisposition;
            return postConsumeData(ptr, completed);
        }
        
        if (controlData.base.buffer.data[0] == '\r') {
            if (controlData.base.buffer.data[1] == '\n') {
                return beginMultipartDataSegment(
                    &controlData.base.buffer.data[2], begin, completed);
            } else {
                return beginMultipartDataSegment(
                    &controlData.base.buffer.data[1], begin, completed);
            }
        } else if (controlData.base.buffer.data[0] == '\n') {
            if (controlData.base.buffer.data[1] == '\r') {
                return beginMultipartDataSegment(
                    &controlData.base.buffer.data[2], begin, completed);
            } else {
                return beginMultipartDataSegment(
                    &controlData.base.buffer.data[1], begin, completed);
            }
        }
        
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        if (ptr >= &controlData.base.buffer.data[available])
            return false;            
        return postConsumeData(ptr, completed);
    }
    case POST_Multipart_Header_ContentDisposition: {
        int available = postDataBlockBegin(completed);
        if (available < 6)
            return false;
        for (ptr = controlData.base.buffer.data; available >= 6;
                ++ptr, --available) {
            char *check = Util_bufferStartsWith("name=\"", ptr);
            if (check == NULL)
                continue;
            controlData.base.post.state = 
                POST_Multipart_Header_ContentDisposition_Name;
            ptr = check;
            break;
        }
        return postConsumeData(ptr, completed);
    }
    case POST_Multipart_Header_ContentDisposition_Name: {
        int available = postDataBlockBegin(completed);
        if (available <= 1)
            return false;
        
        if ((ptr = Util_findCharacter(controlData.base.buffer.data, 
                '\"',0,0)) == NULL)
            return false;
        if (ptr >= &controlData.base.buffer.data[available])
            return false;
        available = ptr - &controlData.base.buffer.data[0];
        available = Util_replaceURLEncoding(controlData.base.buffer.data, 
            &available);
        controlData.base.buffer.data[available] = 0;
        (begin)(controlData.base.buffer.data);
        controlData.base.post.data.multipart.hadName = true;
        
        controlData.base.post.state = POST_Multipart_Header_Finish;
        return postConsumeData(ptr+1, completed);
    }
    case POST_Multipart_Header_Finish: {
        int available = postDataBlockBegin(completed);
        if (available == 0)
            return false;        
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        if (ptr >= &controlData.base.buffer.data[available])
            return false;
        controlData.base.post.state = POST_Multipart_Header;
        return postConsumeData(ptr, completed);
    }
    
    case POST_Multipart_Body: {
        int available = postDataBlockBegin(completed);
        if (available == 0)
            return false;
            
        if ((ptr = nextLineBreak(controlData.base.buffer.data, 
                available)) == NULL) {
            (data)(controlData.base.buffer.data, available);
            return postConsumeData(&controlData.base.buffer.data[available], 
                completed);
        }
        if (ptr != &controlData.base.buffer.data[0]) {
            (data)(controlData.base.buffer.data, ptr - 
                controlData.base.buffer.data);
            return postConsumeData(ptr, completed);
        }
        if (available < 2)
            return false;
        int nMatched = 1;
        if (controlData.base.buffer.data[0] == '\r') {
            if (controlData.base.buffer.data[1] == '\n') {
                ++nMatched;
            }
        } else {
            if (controlData.base.buffer.data[1] == '\r') {
                ++nMatched;
            }
        }
        if (available < nMatched+2)
            return false;
        if (controlData.base.buffer.data[nMatched] == '-' &&
                controlData.base.buffer.data[nMatched+1] == '-') {
            return multpartBoundaryMatch(
                &controlData.base.buffer.data[nMatched+2],
                &controlData.base.buffer.data[available], data, completed);
        }
        
        /* Not actually the start, so we can send everything until the next
         * CR/LF, or everything we have if there isn't one */
        ptr = nextLineBreak(&controlData.base.buffer.data[nMatched], 
            available - nMatched);
        if (ptr == NULL)
            ptr = &controlData.base.buffer.data[available];        
        (data)(controlData.base.buffer.data, ptr - 
            controlData.base.buffer.data);
        return postConsumeData(ptr, completed);
    }
    }
    
    return false;
}

static void genericPOST(HTTP_POSTFieldBegin begin, 
                        HTTP_POSTFieldData data,
                        HTTP_POSTCompleted completed)
{
    ServerReadStatus status = Server_read();
    if (status == ServerRead_Nothing)
        return;
    if (status == ServerRead_Error) {
        (completed)(-1);
        return;
    }
    while (genericPOSTProcess(begin, data, completed)) { }
}

void Server_userPOST(void)
{    
    genericPOST(controlData.userPOST.callback.data.begin,
        controlData.userPOST.callback.data.field,
        controlData.userPOST.callback.data.completed);
}

void Server_uploadPOST(void)
{
    genericPOST(Control_writeUserCodeField, Control_writeUserCodeData,
        Control_writeUserCodeCompleted);
}
