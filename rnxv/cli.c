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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "cli.h"
#include "serial.h"
#include "gpio.h"

void CLI_enter(CLIContext *context)
{
    context->buffer[0] = 0;
    context->output[0] = 0;
    strcpy(context->output, "CMD\r\n");
}

static void discardingShift(char *target, size_t maximumLength,
                            const char *add, size_t addLength)
{
    if (addLength >= maximumLength-1) {
        add += addLength;
        add -= maximumLength-1;
        addLength = maximumLength-1;
    }
    size_t existing = strlen(target);
    if (existing + addLength >= maximumLength) {
        size_t discard = (existing + addLength) - maximumLength;
        memmove(target, &target[discard], existing - discard);
    }
    memcpy(&target[existing], add, addLength);
    target[existing+addLength] = 0;
}

void CLI_incoming(CLIContext *context, const char *add, uint32_t length)
{
    discardingShift(context->buffer, sizeof(context->buffer), add, length);
}

static void addToOutput(CLIContext *context, const char *add)
{
    discardingShift(context->output, sizeof(context->output), add, strlen(add));
}

static const char *bufferStartsWith(CLIContext *context, const char *begin)
{
    int len = strlen(begin);
    if (strncasecmp(context->buffer, begin, len))
        return NULL;
    return &context->buffer[len];
}

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
static uint32_t parseHex(const char *in, const char **out)
{
    uint32_t result = 0;
    for (; *in; ++in) {
        char add = *in;
        if (add >= '0' && add <= '9') {
            result *= 16;
            result += (uint32_t)(add - '0');
        } else if (add >= 'A' && add <= 'F') {
            result *= 16;
            result += (uint32_t)(add - 'A');
        } else if (add >= 'a' && add <= 'f') {
            result *= 16;
            result += (uint32_t)(add - 'a');
        } else {
            break;
        }
    }
    *out = in;
    return result;
}

bool CLI_process(CLIContext *context)
{
    while (true) {
        char *end = strpbrk(context->buffer, "\r\n");
        if (end == NULL)
            return true;            
        *end = 0;
        
        if (bufferStartsWith(context, "quit")) {
            addToOutput(context, "EXIT");
            return false;
        }
        
        const char *ptr;
        if (bufferStartsWith(context, "set uart mode") ||
                bufferStartsWith(context, "set uart tx") ||
                bufferStartsWith(context, "set sys iofunc")) {
            /* Ignored */
            addToOutput(context, "AOK\r\n");
        } else if ((ptr=bufferStartsWith(context, 
                "set uart instant ")) != NULL) {
            uint32_t baud = parseDecimal(ptr);
            if (baud > 0) {
                Serial_setBaud(baud);
                addToOutput(context, "AOK\r\n");
            } else {
                addToOutput(context, "ERROR\r\n");
            }
        } else if ((ptr=bufferStartsWith(context, "set sys mask ")) != NULL) {
            uint32_t mask = parseHex(ptr, &ptr);
            if (!(*ptr)) {
                addToOutput(context, "ERROR\r\n");
            } else if (*ptr == '0') {
                GPIO_setMode(mask, false);
                addToOutput(context, "AOK\r\n");
            } else {
                GPIO_setMode(mask, true);
                addToOutput(context, "AOK\r\n");
            }
        } else if ((ptr=bufferStartsWith(context, "set sys output ")) != NULL) {
            uint32_t output = parseHex(ptr, &ptr);
            if (!(*ptr)) {
                addToOutput(context, "ERROR\r\n");
            } else {
                uint32_t mask = parseHex(ptr, &ptr);
                GPIO_setOutput(mask, output);
                addToOutput(context, "AOK\r\n");
            }
        } else if (end != &context->buffer[0]) {
            addToOutput(context, "ERROR\r\n");
        }
        
        memmove(context->buffer, end+1, strlen(end+1)+1);
    }
}
