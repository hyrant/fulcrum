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
#include <string.h>

#include <fulcrum.h>

#include "adc.h"

static int tcpServerSocket;
static int tcpConnectedSocket;
static int udpSocket;

/**
 * Write a string to a socket.
 * 
 * @param fd        the socket
 * @param str       the string
 */
static void writeString(int fd, const char *str)
{
    send(fd, str, strlen(str));
}

/**
 * Write a single number in decimal out to the given socket.
 * 
 * @param fd    the socket
 * @param n     the number
 */
static void writeDecimal(int fd, uint32_t n)
{
    if (n == 0) {
        send(fd, "0", 1);
        return;
    }
    char buffer[10];
    int length = 0;
    while (n) {
        buffer[length++] = '0' + (n%10);
        n /= 10;
    }
    for (char *a=buffer, *b=&buffer[length-1]; a<b; ++a, --b) {
        char tmp = *a;
        *a = *b;
        *b = tmp;
    }
    send(fd, buffer, length);
}

/**
 * Accept a pending connection, if any.
 */
static void acceptTCPConnection(void)
{
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    if ((tcpConnectedSocket = accept(tcpServerSocket, (struct sockaddr *)&addr, 
            &len)) < 0) {
        tcpConnectedSocket = -1;
        yield();
        return;
    }
    
    writeString(tcpConnectedSocket,
"Fulcrum example server\r\n"
"ADC Temperature: ");
    uint16_t adcTemp;
    uint16_t adcRef;
    ADC_get(&adcTemp, &adcRef);
    writeDecimal(tcpConnectedSocket, adcTemp);
    writeString(tcpConnectedSocket, ", Reference: ");
    writeDecimal(tcpConnectedSocket, adcRef);
    writeString(tcpConnectedSocket, "\r\nNow echoing all input\r\n");
}

/**
 * Handle incoming TCP data.
 */
static void tcpReceive()
{
    char buffer[64];
    int n = recv(tcpConnectedSocket, buffer, sizeof(buffer));
    if (n == 0) {
        yield();
        return;
    }
    if (n < 0) {
        closesocket(tcpConnectedSocket);
        tcpConnectedSocket = -1;
        return;
    }
    send(tcpConnectedSocket, buffer, n);
}

/**
 * Attempt to open the TCP server's listening socket.
 */
static void openTCPSocket(void)
{
    if ((tcpServerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {        
        tcpServerSocket = -1;
        yield();
        return;
    }
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(23);
    addr.sin_addr = INADDR_ANY;
    if (bind(tcpServerSocket, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        closesocket(tcpServerSocket);
        tcpServerSocket = -1;
        return;
    }
    
    if (listen(tcpServerSocket, 5) != 0) {
        closesocket(tcpServerSocket);
        tcpServerSocket = -1;
        return;
    }
}

/**
 * The thread for the TCP server.
 */
static void tcpServerThread(void *param)
{
    (void)param;
    while (true) {
        if (tcpServerSocket < 0)
            openTCPSocket();
        if (tcpConnectedSocket < 0) {
            acceptTCPConnection();
        } else {
            tcpReceive();
        }
    }
}

/**
 * Append a decimal number to a string buffer.
 * 
 * @param target    the target buffer
 * @param length    the length of the buffer
 * @param n         the number to add
 */
static void appendDecimal(char *target, int *length, uint32_t n)
{
    target += *length;
    if (n == 0) {
        *target = '0';
        ++(*length);
        return;
    }
    char *original = target;
    while (n) {
        *target = '0' + (n%10);
        ++target;
        n /= 10;
    }
    *length += target - original;
    for (--target; original<target; ++original, --target) {
        char tmp = *original;
        *original = *target;
        *target = tmp;
    }
}

/**
 * Handle an incoming UDP packet.
 */
static void udpReceive(void)
{
    char buffer[64];
    struct sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);
    int n = recvfrom(udpSocket, buffer, sizeof(buffer)-1, 
            (struct sockaddr *)&addr, &addrlen);
    if (n == 0) {
        yield();
        return;
    }
    if (n < 0) {
        closesocket(udpSocket);
        udpSocket = -1;
        return;
    }
    
    struct in_addr resolved;
    buffer[n] = 0;
    if (gethostbyname(buffer, &resolved) != 0) {
        if (n < 32) {
            strcat(buffer, ": resolve failed\r\n");
        } else {
            strcpy(buffer, "Resolve failed\r\n");
        }
        n = strlen(buffer);
    } else {
        n = 0;
        appendDecimal(buffer, &n, resolved.s_addr & 0xFF);
        buffer[n++] = '.';
        appendDecimal(buffer, &n, (resolved.s_addr>>8) & 0xFF);
        buffer[n++] = '.';
        appendDecimal(buffer, &n, (resolved.s_addr>>16) & 0xFF);        
        buffer[n++] = '.';
        appendDecimal(buffer, &n, (resolved.s_addr>>24) & 0xFF);
        buffer[n++] = '\r';
        buffer[n++] = '\n';
    }

    sendto(udpSocket, buffer, n, (const struct sockaddr *)&addr, addrlen);
}

/**
 * Attempt to open the UDP socket.
 */
static void openUDPSocket(void)
{
    if ((udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        udpSocket = -1;
        yield();
        return;
    }
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(2048);
    addr.sin_addr = INADDR_ANY;
    if (bind(udpSocket, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        closesocket(udpSocket);
        udpSocket = -1;
        return;
    }
}

/**
 * The thread for the UDP server
 */
static void udpServerThread(void *param)
{
    (void)param;
    
    while (true) {
        int sock = udpSocket;
        if (sock < 0) {
            openUDPSocket();
            continue;
        }
        
        fd_set rx;
        FD_ZERO(&rx);
        FD_SET(sock, &rx);
        
        int rc = select(&rx, NULL, NULL, 100);
        if (rc == 0) {
            yield();
            continue;
        }
        if (rc < 0) {
            closesocket(udpSocket);
            udpSocket = -1;
            continue;
        }
        
        udpReceive();
    }
}

/**
 * Handle a network reset by the kernel.
 */
static void handleNetworkReset(void)
{
    tcpServerSocket = -1;
    tcpConnectedSocket = -1;
    udpSocket = -1;
}

/**
 * Handle a remotely closed TCP socket.
 */
static void handleTCPClosed(void)
{
    if (tcpConnectedSocket == -1)
        return;
    closesocket(tcpConnectedSocket);
    tcpConnectedSocket = -1;
}

extern unsigned _stack;
void Server_init(void)
{
    handler_network_reset(handleNetworkReset);
    handler_tcp_closed(handleTCPClosed);
    handleNetworkReset();
    
    //thread_start(tcpServerThread, NULL, (uint8_t *)&_stack - 1024 - 256 * 2);
    thread_start(udpServerThread, NULL, (uint8_t *)&_stack - 1024 - 256 * 4);
}
