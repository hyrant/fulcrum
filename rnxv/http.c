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

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <fulcrum.h>

#include "http.h"
#include "serial.h"
#include "gpio.h"

/**
 * Shared data used during POST requests.  Interpretation depends on the
 * active callback.
 */
static union {
    struct {
        char buffer[16];
        enum {
            SerialField_Invalid,
            SerialField_Baud,
            SerialField_DataBits,
            SerialField_Parity,
            SerialField_StopBits,
            SerialField_FlowControl,
        } field;
    } serialUpdate;
    struct {
        char buffer[16];
    } tcpUpdate;
    struct {
        char buffer[16];
        uint8_t pin;
        enum {
            GPIO_Disable = 0,
            GPIO_On,
            GPIO_Off,
        } mode;
    } gpio;
} post;

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
 * Write the main web page out the given socket.
 * 
 * @param fd    the output socket
 */
static void serveMainPage(int fd)
{
    if (fd == -1)
        return;
        
    int baud;
    SerialParity parity;
    int dataBits;
    int stopBits;
    bool flowControl;
    Serial_getSettings(&baud, &parity, &dataBits, &stopBits, &flowControl);
        
    writeString(fd,
"Fulcrum RN-XV transparent serial pass through.<br>"
"<br>"
"<br>"
"<form action=\"/serial_update\" method=\"post\">"
    "Baud: <input type=\"number\" name=\"baud\" min=\"50\" max=\"460800\" value=\"");
        writeDecimal(fd, baud);
        writeString(fd, "\"><br>"
    "Parity: <select name=\"parity\">"
        "<option value=\"0\"");
        if (parity == Parity_None)
            writeString(fd, " selected");
        writeString(fd, ">None</option>"
        "<option value=\"1\"");
        if (parity == Parity_Even)
            writeString(fd, " selected");
        writeString(fd, ">Even</option>"
        "<option value=\"2\"");
         if (parity == Parity_Odd)
            writeString(fd, " selected");
        writeString(fd, ">Odd</option>"
    "</select><br>"
    "Data bits: <select name=\"data\">"
        "<option value=\"7\"");
        if (dataBits == 7)
            writeString(fd, " selected");
        writeString(fd, ">7</option>"
        "<option value=\"8\"");
        if (dataBits == 8)
            writeString(fd, " selected");
        writeString(fd, ">8</option>"
    "</select><br>"
    "Stop bits: <select name=\"stop\">"
        "<option value=\"1\"");
        if (stopBits == 1)
            writeString(fd, " selected");
        writeString(fd, ">1</option>"
        "<option value=\"2\"");
        if (stopBits == 2)
            writeString(fd, " selected");
        writeString(fd, ">2</option>"
    "</select><br>"
    "Flow control: <select name=\"flow\">"
        "<option value=\"0\"");
        if (!flowControl)
            writeString(fd, " selected");
        writeString(fd, ">None</option>"
        "<option value=\"1\"");
        if (flowControl)
            writeString(fd, " selected");
        writeString(fd, ">RTS/CTS</option>"
    "</select><br>"
    "<input type=\"submit\" value=\"Update\">"
"</form><br>"
"<br>"
"<br>"
"<form action=\"/tcp_update\" method=\"post\">"
    "TCP Port: <input type=\"number\" name=\"port\" min=\"1\" max=\"65535\" value=\"");
            writeDecimal(fd, Serial_getTCPPort());
        writeString(fd, "\">"
    "<input type=\"submit\" value=\"Change\">"
"</form><br><br>");
    for (int i=0; i<16; i++) {
        const char *name = GPIO_getName(i);
        if (name == NULL)
            continue;
        writeString(fd, "<br>"
"<form action=\"/gpio_");
            writeDecimal(fd, i);
            writeString(fd, "\" method=\"post\">GPIO");
        writeDecimal(fd, i);
        writeString(fd, " ");
        writeString(fd, name);
        writeString(fd, ": <select name=\"set\">"
            "<option value=\"0\">Disable</option>"
            "<option value=\"1\">On</option>"
            "<option value=\"2\">Off</option>"
    "<input type=\"submit\" value=\"Set\">"
"</form>");
    }
}

/* atoi uses a ton of RAM, so just do this quick and dirty */
static uint32_t parseDecimal(const char *in)
{
    uint32_t result = 0;
    for (; *in; ++in) {
        char add = *in;
        if (add < '0' || add > '9')
            break;
        result *= 10;
        result += (uint32_t)(add - '0');
    }
    return result;
}

static void serialUpdateFieldCompleted(void)
{
    if (!post.serialUpdate.buffer[0])
        return;
        
    switch (post.serialUpdate.field) {
    case SerialField_Invalid:
        break;
        
    case SerialField_Baud:
        Serial_setBaud(parseDecimal(post.serialUpdate.buffer));
        break;
    
    case SerialField_Parity: {
        switch (parseDecimal(post.serialUpdate.buffer)) {
        case 1: Serial_setParity(Parity_Even); break;
        case 2: Serial_setParity(Parity_Odd); break;
        default: Serial_setParity(Parity_None); break;
        }
        break;
    }
    
    case SerialField_DataBits:
        Serial_setDataBits(parseDecimal(post.serialUpdate.buffer));
        break;
    
    case SerialField_StopBits:
        Serial_setStopBits(parseDecimal(post.serialUpdate.buffer));
        break;
        
    case SerialField_FlowControl:
        Serial_setFlowControl(parseDecimal(post.serialUpdate.buffer) != 0);
        break;
    
    }
        
    post.serialUpdate.buffer[0] = 0;
}
static void serialUpdateFieldBegin(const char *name)
{
    serialUpdateFieldCompleted();
    
    if (!strcmp(name, "baud")) {
        post.serialUpdate.field = SerialField_Baud;
    } else if (!strcmp(name, "parity")) {
        post.serialUpdate.field = SerialField_Parity;
    } else if (!strcmp(name, "data")) {
        post.serialUpdate.field = SerialField_DataBits;
    } else if (!strcmp(name, "stop")) {
        post.serialUpdate.field = SerialField_StopBits;
    } else if (!strcmp(name, "flow")) {
        post.serialUpdate.field = SerialField_FlowControl;
    } else {
        post.serialUpdate.field = SerialField_Invalid;
    }
}
static void serialUpdateFieldData(const void *data, uint32_t n)
{
    if (post.serialUpdate.field == SerialField_Invalid)
        return;
    uint32_t len = strlen(post.serialUpdate.buffer);
    if (len >= sizeof(post.serialUpdate.buffer))
        return;
    if (n > (sizeof(post.serialUpdate.buffer)-1)-len)
        n = (sizeof(post.serialUpdate.buffer)-1)-len;
    memcpy(&post.serialUpdate.buffer[len], data, n);
    post.serialUpdate.buffer[len+n] = 0;
}
static void serialUpdateCompleted(int fd)
{
    serialUpdateFieldCompleted();
    Serial_saveSettings();
    
    serveMainPage(fd);
}


static void tcpUpdateFieldCompleted(void)
{
    if (!post.tcpUpdate.buffer[0])
        return;
    Serial_setTCPPort(parseDecimal(post.tcpUpdate.buffer));
}
static void tcpUpdateFieldBegin(const char *name)
{
    (void)name;
    tcpUpdateFieldCompleted();
}
static void tcpUpdateFieldData(const void *data, uint32_t n)
{
    uint32_t len = strlen(post.tcpUpdate.buffer);
    if (len >= sizeof(post.tcpUpdate.buffer))
        return;
    if (n > (sizeof(post.tcpUpdate.buffer)-1)-len)
        n = (sizeof(post.tcpUpdate.buffer)-1)-len;
    memcpy(&post.tcpUpdate.buffer[len], data, n);
    post.tcpUpdate.buffer[len+n] = 0;
}
static void tcpUpdateCompleted(int fd)
{
    tcpUpdateFieldCompleted();
    Serial_saveSettings();
    
    serveMainPage(fd);
}

static void gpioFieldCompleted(void)
{
    if (!post.gpio.buffer[0])
        return;
    post.gpio.mode = parseDecimal(post.gpio.buffer);
}
static void gpioFieldBegin(const char *name)
{
    (void)name;
    gpioFieldCompleted();
}
static void gpioFieldData(const void *data, uint32_t n)
{
    uint32_t len = strlen(post.gpio.buffer);
    if (len >= sizeof(post.gpio.buffer))
        return;
    if (n > (sizeof(post.gpio.buffer)-1)-len)
        n = (sizeof(post.gpio.buffer)-1)-len;
    memcpy(&post.gpio.buffer[len], data, n);
    post.gpio.buffer[len+n] = 0;
}
static void gpioCompleted(int fd)
{
    gpioFieldCompleted();
    
    switch (post.gpio.mode) {
    case GPIO_Disable:
        GPIO_setMode(1<<post.gpio.pin, false);
        break;
    case GPIO_On:
        GPIO_setMode(1<<post.gpio.pin, true);
        GPIO_setOutput(1<<post.gpio.pin, 1<<post.gpio.pin);
        break;
    case GPIO_Off:
        GPIO_setMode(1<<post.gpio.pin, true);
        GPIO_setOutput(1<<post.gpio.pin, 0);
        break;
    default:
        break;
    }
    
    serveMainPage(fd);
}

/**
 * Called for any unknown URL by the kernel, so provide the callbacks for
 * a new main page.
 */
static HTTP_Status GET_handler(const char *path, HTTP_GETCallback *callback)
{
    if (!strcmp(path, "/") || !strcmp(path, "/index.htm") || 
            !strcmp(path, "/index.html")) {
        callback->data = serveMainPage;
        return HTTP_Handled;
    }
    return HTTP_Unhandled;
}

/**
 * Called for any unknown POST request by the kernel, so provide handlers
 * for the update URLs we accept.
 */
static HTTP_Status POST_handler(const char *path, HTTP_POSTCallback *callback)
{
    if (!strcmp(path, "/serial_update")) {
        post.serialUpdate.buffer[0] = 0;
        post.serialUpdate.field = SerialField_Invalid;
        
        callback->data.begin = serialUpdateFieldBegin;
        callback->data.field = serialUpdateFieldData;
        callback->data.completed = serialUpdateCompleted;
        return HTTP_Handled;
    } else if (!strcmp(path, "/tcp_update")) {
        post.tcpUpdate.buffer[0] = 0;
        
        callback->data.begin = tcpUpdateFieldBegin;
        callback->data.field = tcpUpdateFieldData;
        callback->data.completed = tcpUpdateCompleted;
        return HTTP_Handled;
    } else if (!strncmp(path, "/gpio_", 6)) {
        post.gpio.buffer[0] = 0;
        post.gpio.pin = parseDecimal(path+6);
        if (post.gpio.pin >= 16)
            return HTTP_Unhandled;
        
        callback->data.begin = gpioFieldBegin;
        callback->data.field = gpioFieldData;
        callback->data.completed = gpioCompleted;
        return HTTP_Handled;
    }
    return HTTP_Unhandled;
}

void HTTP_init(void)
{
    handler_http_get(GET_handler);
    handler_http_post(POST_handler);
}

