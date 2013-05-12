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

#ifndef _UTIL_H
#define _UTIL_H

#include <stdint.h>
#include <stdbool.h>

#include "netapi.h"

/**
 * Remove a segment of a buffer, shifting the remaining data left and
 * replacing the shifted data with zero.
 * <br>
 * This is effectively:
 * @code
 * memmove(begin, end, totalLength-(end-begin));
 * memset(end, 0, (begin+totalLength)-end)
 * return totalLength-(end-begin)
 * @endcode
 */
int Util_shiftConsumeBuffer(char *begin, char *end, int totalLength);

/**
 * Simple case insensitive compare of two characters.
 * 
 * @param a     the first character
 * @param b     the second character
 * @return      true if they are equal
 */
bool Util_caseInsensitiveCompare(char a, char b);

/**
 * Return the end of the first part of a buffer if it matches a token.  This
 * is case insensitive.  All spaces after the token are also excluded.
 * 
 * @param token     the token to search for
 * @param buffer    the buffer
 * @return          the first character after the token if the buffer matches
 */
char *Util_bufferStartsWith(const char *token, char *buffer);

/**
 * Find the first occurrence of the given characters.  This is analogous
 * to strpbrk, but implemented this way to avoid having to allocate the
 * strings.
 * 
 * @param buffer    the buffer to search
 * @param a         the first character to find
 * @param b         the second character to find or zero
 * @param c         the third character to find or zero
 */
char *Util_findCharacter(const char *buffer, char a, char b, char c);

/**
 * Parse an ip address.
 * 
 * @param ip        the input ip address
 * @return          the parsed IP address or zero if invalid
 */
struct in_addr Util_parseIP(const char *ip);

/**
 * Parse a decimal number.
 * 
 * @param in        the input string to parse
 * @return          the parsed number
 */
uint32_t Util_parseDecimal(const char *in);

/**
 * Parse a hexadecimal number.
 * 
 * @param in        the input string to parse
 * @return          the parsed number
 */
uint32_t UTIL_parseHexadecimal(const char *in);

/**
 * Replace URL (percent) encoding with the literal characters it represents.
 * This also calculates the updated length as well as how much of the buffer
 * was consumed.  The result may not be completely parsed if it ends
 * with an incomplete escape.
 * 
 * @param buffer    the input and output buffer
 * @param length    the input length and the output number of bytes remaining that where not parsed
 * @return          the length of the parsed buffer
 */
int Util_replaceURLEncoding(char *buffer, int *length);

/**
 * Parse a series of hexadecimal digits (e.x. a MAC address).  ":" and "."
 * are permitted to separate pairs of digits.
 * 
 * @param hex       the hex string to parse
 * @param dest      the output buffer
 * @param length    the length of the output buffer
 * @return          the number of digits parsed
 */
int Util_parseHex(const char *hex, uint8_t *dest, int length);

#endif
