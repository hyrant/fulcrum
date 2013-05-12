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

#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    char buffer[64];    
    char output[32];
} CLIContext;

/**
 * Enter CLI command mode on the given context.
 * 
 * @param context   the context
 */
void CLI_enter(CLIContext *context);

/**
 * Add incoming data to the context.
 * 
 * @param context   the context
 * @param add       the start of data to add
 * @param length    the length of data to add
 */
void CLI_incoming(CLIContext *context, const char *add, uint32_t length);

/**
 * Process the context.
 * 
 * @param context   the context
 * @return          false if the processing should end
 */
bool CLI_process(CLIContext *context);

#endif
