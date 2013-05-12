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
#include "led.h"
#include "adc.h"

/**
 * Shared data used during POST requests.  Interpretation depends on the
 * active callback.
 */
static union {
    struct {
        char buffer[16];
        LED_MODE mode;
        uint16_t up;
        uint16_t down;
        enum {
            LEDFIELD_INVALID,
            LEDFIELD_MODE,
            LEDFIELD_UP,
            LEDFIELD_DOWN,
        } field;
    } ledUpdate;
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
        
    LED_MODE ledMode;
    uint16_t ledUp;
    uint16_t ledDown;
    LED_getMode(&ledMode, &ledUp, &ledDown);
        
    writeString(fd,
"Fulcrum example main page.<br>"
"Servers listening on TCP port 23 and UDP 2048.<br>"
"<br>"
"<br>"
"<form name=\"led\" action=\"/led_update\" method=\"post\">"
    "LED Mode: <select name=\"mode\">"
        "<option value=\"0\"");
        if (ledMode == LED_BLINK)
            writeString(fd, " selected");
        writeString(fd, ">Blink</option>"
        "<option value=\"1\"");
        if (ledMode == LED_FADE)
            writeString(fd, " selected");
        writeString(fd, ">Fade</option>"
    "</select><br>"
    "On time (milliseconds): <input type=\"number\" name=\"up\" min=\"1\" max=\"10000\" value=\"");
        writeDecimal(fd, ledUp);
        writeString(fd, "\"><br>"
    "Off time (milliseconds): <input type=\"number\" name=\"down\" min=\"1\" max=\"10000\" value=\"");
        writeDecimal(fd, ledDown);
        writeString(fd, "\"><br>"
    "<input type=\"submit\" value=\"Update\">"
"</form><br>"
"<br>"
"<br>"
"ADC Temperature: ");
    uint16_t adcTemp;
    uint16_t adcRef;
    ADC_get(&adcTemp, &adcRef);
    writeDecimal(fd, adcTemp);
    writeString(fd, ", Reference: ");
    writeDecimal(fd, adcRef);
}

/* atoi uses a ton of RAM, so just do this quick and dirty */
uint32_t parseDecimal(const char *in)
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

/**
 * A field in the LED update request has been completed so update the
 * shared data.
 */
static void ledFieldCompleted(void)
{
    if (!post.ledUpdate.buffer[0])
        return;
        
    switch (post.ledUpdate.field) {
    case LEDFIELD_INVALID:
        break;
    case LEDFIELD_MODE: {
        uint32_t i = parseDecimal(post.ledUpdate.buffer);
        if (i == 1) {
            post.ledUpdate.mode = LED_FADE;
        } else {
            post.ledUpdate.mode = LED_BLINK;
        }
        break;
    case LEDFIELD_UP:
        post.ledUpdate.up = (uint16_t)parseDecimal(post.ledUpdate.buffer);
        break;
    case LEDFIELD_DOWN:
        post.ledUpdate.down = (uint16_t)parseDecimal(post.ledUpdate.buffer);
        break;
    }
    }
        
    post.ledUpdate.buffer[0] = 0;
}

/**
 * A field is beginning in the LED update request.
 * 
 * @param name  the name of the incoming field
 */
static void ledUpdateFieldBegin(const char *name)
{
    ledFieldCompleted();
    
    if (!strcmp(name, "mode")) {
        post.ledUpdate.field = LEDFIELD_MODE;
    } else if (!strcmp(name, "up")) {
        post.ledUpdate.field = LEDFIELD_UP;
    } else if (!strcmp(name, "down")) {
        post.ledUpdate.field = LEDFIELD_DOWN;
    } else {
        post.ledUpdate.field = LEDFIELD_INVALID;
    }
}

/**
 * Data is being added to the active field in the LED update request.
 * 
 * @param data  the data to add
 * @param n     the length to add
 */
static void ledUpdateFieldData(const void *data, uint32_t n)
{
    if (post.ledUpdate.field == LEDFIELD_INVALID)
        return;
    uint32_t len = strlen(post.ledUpdate.buffer);
    if (len >= sizeof(post.ledUpdate.buffer))
        return;
    if (n > (sizeof(post.ledUpdate.buffer)-1)-len)
        n = (sizeof(post.ledUpdate.buffer)-1)-len;
    memcpy(&post.ledUpdate.buffer[len], data, n);
    post.ledUpdate.buffer[len+n] = 0;
}

/**
 * The LED update request has completed, so apply changes and serve the
 * main page again.
 * 
 * @param fd    the socket
 */
static void ledUpdateCompleted(int fd)
{
    ledFieldCompleted();
    LED_setMode(post.ledUpdate.mode, post.ledUpdate.up, post.ledUpdate.down);
    
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
    if (!strcmp(path, "/led_update")) {
        post.ledUpdate.buffer[0] = 0;
        post.ledUpdate.mode = LEDFIELD_INVALID;
        LED_getMode(&post.ledUpdate.mode, &post.ledUpdate.up, 
            &post.ledUpdate.down);
        
        callback->data.begin = ledUpdateFieldBegin;
        callback->data.field = ledUpdateFieldData;
        callback->data.completed = ledUpdateCompleted;
        return HTTP_Handled;
    }
    return HTTP_Unhandled;
}

void HTTP_init(void)
{
    handler_http_get(GET_handler);
    handler_http_post(POST_handler);
}

