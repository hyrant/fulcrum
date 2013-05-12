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

extern unsigned _ram, _kernelbegin, _data_loadaddr, _edata, _data, _userbegin;

char *Server_bufferStartsWith(const char *token) 
{
    return Util_bufferStartsWith(token, controlData.base.buffer.data);
}

char *Server_findLineEnd(char *begin, char *end)
{
    if (httpClientSocket != -1 && (end-begin) < 2)
        return NULL;
    for (char *check=begin; check<end-1; ++check) {
        if (*check == '\r') {
            *check = 0;
            ++check;
            if (*check == '\n')
                ++check;
            return check;
        } else if (*check == '\n') {
            *check = 0;
            ++check;
            if (*check == '\r')
                ++check;
            return check;
        }
    }
    return NULL;
}

char *Server_bufferLineEnd(void)
{
    return Server_findLineEnd(controlData.base.buffer.data,
        &controlData.base.buffer.data[controlData.base.buffer.length]);
}

char *Server_endOfCurrentField(void)
{
    char *ptr = controlData.base.buffer.data;
    for (; *ptr; ++ptr) {
        if (*ptr == ' ' || *ptr == '\t' || *ptr == '\r' || *ptr == '\n')
            break;
    }
    return ptr;
}

static bool bufferAfterIs(const char *compare, const char *check)
{
    for (; ; ++check, ++compare) {
        if (!(*compare))
            return !(*check);
        if (!Util_caseInsensitiveCompare(*compare, *check))
            return false;
    }
}

bool Server_bufferMatches(const char *compare)
{    
    return bufferAfterIs(compare, controlData.base.buffer.data);
}

void Server_consumeBuffer(char *end)
{
    controlData.base.buffer.length = Util_shiftConsumeBuffer(
        controlData.base.buffer.data, end, controlData.base.buffer.length);
}

bool Server_headersEnded(void)
{
    if (controlData.base.buffer.data[0] == '\r') {
        if (controlData.base.buffer.data[1] == '\n') {
            Server_consumeBuffer(&controlData.base.buffer.data[2]);
        } else {
            Server_consumeBuffer(&controlData.base.buffer.data[1]);
        }
        return true;
    } else if (controlData.base.buffer.data[0] == '\n') {
        if (controlData.base.buffer.data[1] == '\r') {
            Server_consumeBuffer(&controlData.base.buffer.data[2]);
        } else {
            Server_consumeBuffer(&controlData.base.buffer.data[1]);
        }
        return true;
    }
    return false;
}

static bool clientAuthorizationPassed(ControlSimpleAuthorization *auth)
{
    if (!Control_requireAuthorization())
        return true;
        
    char *ptr;
    while (true) {
        if (controlData.base.buffer.length == 0)
            return false;
            
        switch (*auth) {
        case SimpleAuth_WaitHeader:            
            if ((ptr = Server_bufferStartsWith("Authorization:")) != NULL) {
                Server_consumeBuffer(ptr);
                *auth = SimpleAuth_WaitType;
                break;
            } else if (Server_headersEnded()) {
                Server_unauthorized();
                return false;
            }
            
            if ((ptr = Server_bufferLineEnd()) == NULL)
                return false;
            Server_consumeBuffer(ptr);
            break;
            
        case SimpleAuth_WaitType:
            if ((ptr = Server_bufferStartsWith("Basic")) != NULL) {
                Server_consumeBuffer(ptr);
                *auth = SimpleAuth_Matching;
                break;
            }
            
            if ((ptr = Server_bufferLineEnd()) == NULL)
                return false;
            *auth = SimpleAuth_WaitHeader;
            Server_consumeBuffer(ptr);
            break;
            
        case SimpleAuth_Matching:
            if ((ptr = Server_bufferLineEnd()) == NULL)
                return false;
            if (!Control_checkAuthorization(controlData.base.buffer.data)) {
                Server_unauthorized();
                return false;
            } else {
                return true;
            }
        }
    }
}

bool genericReadProcess(void)
{
    char *ptr;
    
    if (controlData.base.buffer.length == 0)
        return false;
    
    switch (controlState) {
    case Idle:
        if ((ptr = Server_bufferStartsWith("GET")) != NULL) {
            Server_consumeBuffer(ptr);
            controlState = Initialize_GET;
            return true;
        } else if ((ptr = Server_bufferStartsWith("POST")) != NULL) {
            Server_consumeBuffer(ptr);
            controlState = Initialize_POST;
            return true;
        } else if (Util_findCharacter(controlData.base.buffer.data, 
                '\r', '\n', 0) != NULL) {
            Server_badRequest();
            return false;
        }
        return false;
        
    case Initialize_GET:
    case Initialize_POST:
    {
        if ((ptr = Server_bufferLineEnd()) == NULL)
            return false;
        char *tmp = Server_endOfCurrentField();
        
        int n = (tmp - &controlData.base.buffer.data[0]);
        
        /* Have room for a newline, so this is safe always */
        controlData.base.buffer.data[
            Util_replaceURLEncoding(controlData.base.buffer.data, &n)] = 0;
        
        if ((tmp=Server_bufferStartsWith("/fulcrum")) != NULL) {
            if (bufferAfterIs(tmp, "/ram_kernel.bin")) {
                controlState = GET_RAM_Kernel;
            } else if (bufferAfterIs(tmp, "/ram_user.bin") &&
                    Control_haveUserCode()) {
                controlState = GET_RAM_User;
            } else if (bufferAfterIs(tmp, "/ram_all.bin")) {
                controlState = GET_RAM_All;
            } else if (bufferAfterIs(tmp, "/rom_kernel.bin")) {
                controlState = GET_ROM_Kernel;
            } else if (bufferAfterIs(tmp, "/rom_user.bin") &&
                    Control_haveUserCode()) {
                controlState = GET_ROM_User;
            } else if (bufferAfterIs(tmp, "/rom_all.bin")) {
                controlState = GET_ROM_All;
            } else if (bufferAfterIs(tmp, "/user_halt") &&
                    systemStatus.state == RunState_UserExecuting) {
                controlState = POST_StopUser;
            } else if (bufferAfterIs(tmp, "/user_start") &&
                    Control_haveUserCode()) {
                controlState = POST_StartUser;
            } else if (bufferAfterIs(tmp, "/system_reboot")) {
                controlState = POST_RebootSystem;
            } else if (bufferAfterIs(tmp, "/user_upload") &&
                    controlState == Initialize_POST) {
                Server_consumeBuffer(ptr);
                controlState = POST_UserCodeUpload;
                return false;
            } else if ((tmp = Util_bufferStartsWith("/thread_", tmp)) != NULL &&
                    (n=Util_parseDecimal(tmp)) > 0 && n < MAXIMUM_THREADS) {
                controlState = GET_Thread0 + n;
            } else {
                controlState = GET_Index;
            }
            
            Server_consumeBuffer(ptr);
            return true;
        } else if (systemStatus.state == RunState_UserExecuting) {
            if (controlState == Initialize_GET) {
                if (systemStatus.detailed.normal.httpGET) {
                    HTTP_Status status = 
                        (systemStatus.detailed.normal.httpGET)(
                        controlData.base.buffer.data,
                        &controlData.userGET.callback);
                    
                    switch (status) {
                    case HTTP_Unhandled:
                        break;
                    case HTTP_Handled:
                        Server_standardTextBegin();
                        (controlData.userGET.callback.data)(httpClientSocket);
                        Server_standardTextFooters();
                        Server_connectionCompleted();
                        return false;
                    case HTTP_Handled_Direct:
                        (controlData.userGET.callback.data)(httpClientSocket);
                        Server_connectionCompleted();
                        return false;
                    case HTTP_AuthorizationRequired:
                        controlState = User_GET_AuthorizationRequired;
                        Server_consumeBuffer(ptr);
                        return true;
                    case HTTP_AuthorizationRequired_Direct:
                        controlState = User_GET_AuthorizationRequired_Direct;
                        Server_consumeBuffer(ptr);
                        return true;
                    case HTTP_Raw:
                        Server_consumeBuffer(ptr);
                        if (!(controlData.userGET.callback.raw)(
                                controlData.userGET.buffer.data,
                                controlData.userGET.buffer.length,
                                httpClientSocket)) {
                            Server_connectionCompleted();
                        } else {
                            controlData.base.buffer.length = 0;
                            controlStateStartTime = systick;
                            controlState = User_GET_Raw;
                        }
                        return false;
                    }
                }
            } else {
                if (systemStatus.detailed.normal.httpPOST) {
                    HTTP_Status status = 
                        (systemStatus.detailed.normal.httpPOST)(
                        controlData.base.buffer.data,
                        &controlData.userPOST.callback);
                    
                    switch (status) {
                    case HTTP_Unhandled:
                        break;
                    case HTTP_Handled:
                        Server_consumeBuffer(ptr);
                        controlState = User_POST;
                        controlData.userPOST.post.outputStandardHeaders = true;
                        return false;
                    case HTTP_Handled_Direct:
                        Server_consumeBuffer(ptr);
                        controlState = User_POST;
                        return false;
                    case HTTP_AuthorizationRequired:
                        Server_consumeBuffer(ptr);
                        controlState = User_POST;
                        controlData.userPOST.post.outputStandardHeaders = true;
                        controlData.userPOST.post.data.initialize.
                            authorizationRequired = true;
                        return false;
                    case HTTP_AuthorizationRequired_Direct:
                        Server_consumeBuffer(ptr);
                        controlState = User_POST;
                        controlData.userPOST.post.data.initialize.
                            authorizationRequired = true;
                        return false;
                    case HTTP_Raw:
                        Server_consumeBuffer(ptr);
                        if (!(controlData.userPOST.callback.raw)(
                                controlData.userPOST.buffer.data,
                                controlData.userPOST.buffer.length,
                                httpClientSocket)) {
                            Server_connectionCompleted();
                        } else {
                            controlData.base.buffer.length = 0;
                            controlStateStartTime = systick;
                            controlState = User_POST_Raw;
                        }
                        return false;
                    }
                }
            }
        }
        
        if (Server_bufferMatches("/") || 
                Server_bufferStartsWith("/index") != NULL) {
            
            /* Have to reset this if we get here because the user call
             * may have stomped it. */
            controlData.simpleAuthorization.authorization = 
                SimpleAuth_WaitHeader;
            
            controlState = GET_Index;
            Server_consumeBuffer(ptr);
            return true;
        }
        
        Server_notFound();
        return false;
    }

    case GET_Index:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_mainPage();
        return false;
    case GET_RAM_Kernel:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_memoryAsFile("ram_kernel.bin", (uint32_t)(&_ram), 1024);
        return false;
    case GET_RAM_User:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_memoryAsFile("ram_user.bin", (uint32_t)(&_ram) + 1024, 
            RAM_SIZE - 1024);
        return false;
    case GET_RAM_All:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_memoryAsFile("ram_all.bin", (uint32_t)(&_ram), RAM_SIZE);
        return false;
    case GET_ROM_Kernel:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_memoryAsFile("rom_kernel.bin", (uint32_t)(&_kernelbegin), 
                ((uint32_t)(&_data_loadaddr) + 
                ((uint32_t)(&_edata) - (uint32_t)(&_data))) -
                (uint32_t)(&_kernelbegin));
        return false;
    case GET_ROM_User:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_memoryAsFile("rom_user.bin", (uint32_t)(&_userbegin),
                    Control_userCodeLength());
        return false;
    case GET_ROM_All:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_memoryAsFile("rom_all.bin", (uint32_t)(&_kernelbegin),
                Kernel_totalFlashSize() * 1024);
        return false;        
    case POST_StopUser:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Kernel_haltUserCode();
        Server_reloadStringResponse("User code stopped.");     
        return false;
    case POST_StartUser:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_reloadStringResponse("Starting user code.");
        Kernel_startUserCode();
        return false;
    case POST_RebootSystem:
        if (!clientAuthorizationPassed(&
                controlData.simpleAuthorization.authorization))
            return false;
        Server_reloadStringResponse("Rebooting system.");
        Kernel_rebootSystem();
        return false;
        
    case User_GET_AuthorizationRequired:
        if (!clientAuthorizationPassed(&controlData.userGET.authorization))
            return false;
        Server_standardTextBegin();
        (controlData.userGET.callback.data)(httpClientSocket);
        Server_standardTextFooters();
        Server_connectionCompleted();
        return false;
    case User_GET_AuthorizationRequired_Direct:
        if (!clientAuthorizationPassed(&controlData.userGET.authorization))
            return false;
        (controlData.userGET.callback.data)(httpClientSocket);
        Server_connectionCompleted();
        return false;

    default:
        if (controlState > GET_Thread0 && controlState < GET_ThreadN) {
            if (!clientAuthorizationPassed(&
                    controlData.simpleAuthorization.authorization))
                return false;
            Server_threadDebugInformation(controlState - GET_Thread0);
            return false;
        }
        return false;
    }
    return false;
}

void Server_genericRead(void)
{
    if (Server_read() != ServerRead_MoreData)
        return;
 
    while (genericReadProcess()) { }
}
