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

#include "util.h"


int Util_shiftConsumeBuffer(char *begin, char *end, int totalLength) 
{
    int shifted = (end - begin);
    for (char *final=begin+totalLength; end != final; 
            ++begin, ++end)
        *begin = *end;
    memset(begin, 0, shifted);
    return totalLength - shifted;
}

bool Util_caseInsensitiveCompare(char a, char b)
{
    if (a >= 'a' && a <= 'z')
        a -= 'a' - 'A';
    if (b >= 'a' && b <= 'z')
        b -= 'a' - 'A';
    return a == b;
}

char *Util_bufferStartsWith(const char *token, char *buffer) 
{
    for (; *token; ++buffer, ++token) {
        if (!(*buffer))
            return NULL;
        if (!Util_caseInsensitiveCompare(*token, *buffer))
            return NULL;
    }
    for (; *buffer && (*buffer == ' ' || *buffer == '\t'); ++buffer) { }
    return buffer;
}

char *Util_findCharacter(const char *buffer, char a, char b, char c)
{
    for (; *buffer; ++buffer) {
        if (*buffer == a || *buffer == b || *buffer == c)
            return (char *)buffer;
    }
    return NULL;
}

struct in_addr Util_parseIP(const char *ip)
{
    uint8_t component = 0;
    uint32_t addr = 0;
    for (; ; ++ip) {
        char add = *ip;
        if (add == '.' || !add) {
            addr <<= 8;
            addr |= component;
            component = 0;
            if (!add)
                break;
            continue;
        }
        if (add < '0' || add > '9') {
            addr = 0;
            break;
        }
        component *= 10;
        component += (uint8_t)(add - '0');
    }
    struct in_addr in;
    in.s_addr = htonl(addr);
    return in;
}

uint32_t Util_parseDecimal(const char *in)
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

static uint8_t hexDigit(const char digit)
{
    if (digit >= '0' && digit <= '9')
        return digit - '0';
    if (digit >= 'A' && digit <= 'F')
        return digit - 'A' + 0xA;
    if (digit >= 'a' && digit <= 'f')
        return digit - 'a' + 0xA;
    return 0;
}

int Util_replaceURLEncoding(char *buffer, int *length)
{
    char *original = buffer;
    int remaining;
    for (remaining=*length; remaining>0; ++buffer, --remaining) {
        if (*buffer != '%') {
            continue;
        }
        if (remaining < 3)
            break;
            
        char *target = buffer+1;
        *buffer = (char)((hexDigit(*target) << 4) | hexDigit(*(target+1)));
        
        for (int consume=remaining-3; consume>0; --consume, 
                ++target) {
            *target = *(target+2);
        }
        
        remaining -= 2;
    }
    *length = remaining;
    return buffer - original;
}

int Util_parseHex(const char *hex, uint8_t *dest, int length)
{
    int remaining = length;
    memset(dest, 0, length);
    for (; remaining > 0; --remaining, ++dest) {
        for (; *hex == ':' || *hex == '.'; ++hex) { }
        if (!(*hex))
            break;
        char a = *hex; ++hex;
        if (!(*hex) || (*hex == ':' || *hex == '.')) {
            *dest = hexDigit(a);
            continue;
        }
        char b = *hex; ++hex;
        *dest = (hexDigit(a) << 4) | hexDigit(b);
    }
    return length - remaining;
}

uint32_t UTIL_parseHexadecimal(const char *in)
{
    uint32_t result = 0;
    for (; *in; ++in) {
        result <<= 4;
        uint8_t add = hexDigit(*in);
        if (add == 0 && *in != '0')
            break;
        result |= (uint32_t)add;
    }
    return result;
}
