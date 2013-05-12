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
#include "netapi.h"
#include "server.h"
#include "util.h"
#include "transport.h"

static int serverSocket;
int httpClientSocket;

void Server_clearSockets(void)
{
    serverSocket = -1;
    httpClientSocket = -1;
}
void Server_closeSockets(void)
{
    if (httpClientSocket >= 0) {
        closesocket(httpClientSocket);
        httpClientSocket = -1;
    }
    if (serverSocket >= 0) {
        closesocket(serverSocket);
        serverSocket = -1;
    }
}
bool Server_socketOpen(void)
{
    return serverSocket >= 0;
}
bool Server_clientConnected(void)
{
    return httpClientSocket >= 0;
}

void Server_connectionCompleted(void)
{
    if (httpClientSocket != -1) {
        closesocket(httpClientSocket);
        httpClientSocket = -1;
        controlStateStartTime = systick;
    }
    controlState = Idle;
}


void Server_waitForConnection(void)
{
    if (serverSocket < 0) {
        serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (serverSocket < 0) {
            /* Not a fault if we've got no network config (DHCP pending, for
             * example) */
            struct in_addr addr;
            if (ioctl_network_status(&addr, NULL, NULL, NULL, NULL, 
                    NULL, NULL, 0) == 0 && addr.s_addr == 0) {
                Thread_yield();
                return;
            }
            
            Control_networkFault();
            return;
        }
        
        /* This is the default now */
        #if 0
        uint32_t opt = SOCK_ON;
        if (setsockopt(serverSocket, SOL_SOCKET, SOCKOPT_NONBLOCK,
                &opt, sizeof(opt)) != 0) {
            Control_networkFault();
            return;
        }
        #endif
        
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(LISTEN_PORT);
        addr.sin_addr = INADDR_ANY;
        if (bind(serverSocket, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            Control_networkFault();
            return;
        }
        
        if (listen(serverSocket, 5) != 0) {
            Control_networkFault();
            return;
        }
        
        controlStateStartTime = systick;
    }
    
    if ((httpClientSocket = accept(serverSocket, NULL, NULL)) < 0) {
        httpClientSocket = -1;
        
        /* Periodically re-open it so we catch cases where we've lost
         * synchronization with the device */
        if (Kernel_elapsed(controlStateStartTime) > 300 * TICK_RATE) {
            closesocket(serverSocket);
            serverSocket = -1;
        }
        
        Thread_yield();
        return;
    }
    
    memset(&controlData, 0, sizeof(controlData));
    controlStateStartTime = systick;
}

ServerReadStatus Server_read(void)
{
    if (httpClientSocket < 0) {
        Server_connectionCompleted();
        return ServerRead_Error;
    }
    if (Kernel_elapsed(controlStateStartTime) > 30 * TICK_RATE) {
        Server_connectionCompleted();
        return ServerRead_Error;
    }
    
    if (controlData.base.buffer.length >= 
            sizeof(controlData.base.buffer.data)-1) {
        /* Overflow, so just reset since by now we're only looking for
         * the end of the line anyway; anything else would have consumed
         * something and made this not full.  We leave the last character in
         * case it was a newline, marking the end of the current line. */
        Server_consumeBuffer(&controlData.base.buffer.data[
            sizeof(controlData.base.buffer.data)-3]);
    }
    int n = recv(httpClientSocket, 
        &controlData.base.buffer.data[controlData.base.buffer.length],
        (sizeof(controlData.base.buffer.data)-1) - 
        controlData.base.buffer.length);
    if (n == 0) {
        Thread_yield();
        return ServerRead_Nothing;
    }
    if (n < 0) {
        Server_connectionCompleted();
        return ServerRead_Error;
    }
    
    controlData.base.buffer.length += n;
    return ServerRead_MoreData;
}

/* Override this to prevent the user code from accedentally messing
 * with the HTTP server sockets. */
bool netapi_system_socket(int sockfd)
{
    if (Thread_isKernel())
        return false;
    return sockfd == serverSocket || sockfd == httpClientSocket;
}

void Server_closeUserSockets(void)
{
    for (int i=0; i<8; i++) {
        if (!Transport_ready())
            return;
        if (i == serverSocket || i == httpClientSocket)
            continue;
        closesocket(i);
    }
}
