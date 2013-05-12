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

#ifndef _SERVER_H
#define _SERVER_H

#include <stdbool.h>
#include <api.h>

/** The currently connected client socket */
extern int httpClientSocket;

/**
 * Clear the socket handles.
 */
void Server_clearSockets(void);

/**
 * Close the sockets if they're open.
 */
void Server_closeSockets(void);

/**
 * Close all sockets except those used by the kernel.
 */
void Server_closeUserSockets(void);

/**
 * Handle the generic read state for the server.
 */
void Server_genericRead(void);

/**
 * Test if the server socket is open.
 * 
 * @return true if the server is available
 */
bool Server_socketOpen(void);

/**
 * Test if the server has a client connected.
 * 
 * @return true if there is a client connected
 */
bool Server_clientConnected(void);

/**
 * Close the connection if it is open and return the main idle state.
 */
void Server_connectionCompleted(void);

/**
 * Get the end the current field (the first whitespace character in the
 * incoming buffer).
 * 
 * @return a pointer to the first whitespace character or NULL
 */
char *Server_endOfCurrentField(void);

/**
 * Get the end of a line in the given buffer range.
 * 
 * @param begin     the start to inspect
 * @param end       the end of the inspection
 * @return          the end of the line or NULL if it is not yet complete
 */
char *Server_findLineEnd(char *begin, char *end);

/**
 * Find the end of the current line starting at the beginning of the main
 * parsing buffer.
 * 
 * @return          the end of the line
 */
char *Server_bufferLineEnd(void);

/**
 * Test if the main buffer matches given string.
 * 
 * @param compare   the string to check
 * @return          true on a match
 */
bool Server_bufferMatches(const char *compare);

/**
 * Test if the main buffer starts with the given string
 * 
 * @param token     the string to search for
 * @return          the pointer to the end of the match or NULL if not found
 */
char *Server_bufferStartsWith(const char *token);

/**
 * Test if this is the end of the header block.  That is the buffer begins
 * with a newline (so we received and empty line).  This also consumes the
 * end marker.
 * 
 * @return true if the headers have ended
 */
bool Server_headersEnded(void);

/**
 * Handle generic read requests on the open connection.
 */
void Server_genericRead(void);

/**
 * Handle the generic upload of a segment of memory.
 */
void Server_uploadMemory(void);

/**
 * Call a raw handler on the connected socket.
 * 
 * @param handler   the handler to call
 */
void Server_handleRaw(HTTP_RawHandler handler);

/**
 * Handle an ongoing user POST request.
 */
void Server_userPOST(void);

/**
 * Handle on ongoing user code upload POST request.
 */
void Server_uploadPOST(void);

/**
 * Handle processing while waiting for a connection.
 */
void Server_waitForConnection(void);

typedef enum {
    /** More data was added to the buffer. */
    ServerRead_MoreData,
    /** No new data was available. */
    ServerRead_Nothing,
    /** An error occurred and the client socket has been closed. */
    ServerRead_Error,
} ServerReadStatus;

/**
 * Read pending data into the common buffer.
 * 
 * @return the read status
 */
ServerReadStatus Server_read(void);

/**
 * Consume some bytes out of the common buffer.
 * 
 * @param end   the end pointer (the first byte after the consume)
 */
void Server_consumeBuffer(char *end);



/**
 * Write the 400 Bad Request response.
 */
void Server_badRequest(void);

/**
 * Write the 401 Unauthorized response.
 */
void Server_unauthorized(void);

/**
 * Write the 404 Not Found response.
 */
void Server_notFound(void);

/**
 * Write the standard 200 OK response and the headers to begin a text 
 * document.
 */
void Server_standardTextBegin(void);

/**
 * Write the standard text document footers.
 */
void Server_standardTextFooters(void);

/**
 * Write out the given string response with a page that will reload to
 * the main page.
 * 
 * @param string    the response body
 */
void Server_reloadStringResponse(const char *string);

/**
 * Write out a page with the given thread's debugging information.
 * 
 * @param index     the thread index
 */
void Server_threadDebugInformation(int index);

/**
 * Write out the main landing page.
 */
void Server_mainPage(void);

/**
 * Respond with a download initiating response for the given memory as
 * the specified file name.
 * 
 * @param filename      the client side filename
 * @param addressStart  the address to start reading at
 * @param length        the number of bytes to respond with
 */
void Server_memoryAsFile(const char *filename, uint32_t addressStart,
                         uint32_t length);

#endif
