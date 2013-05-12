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

#include <libopencm3/stm32/desig.h>

#include "kernel.h"
#include "control.h"
#include "netapi.h"
#include "server.h"
#include "util.h"
#include "version.h"

extern unsigned _kernelbegin, _userbegin;

static void write(const void *wr, uint32_t n)
{
    if (httpClientSocket == -1)
        return;
    send(httpClientSocket, wr, n);
}

static void writeString(const char *str)
{
    write(str, strlen(str));
}

static void writeStandardHeaders(void)
{
    writeString(
"Connection: close\r\n"
"Cache-Control: no-cache\r\n");
}

static void writeHTMLHeader(void)
{
    writeStandardHeaders();
    writeString(
"Content-Type: text/html\r\n"
"\r\n"
"<!DOCTYPE html>\n"
"<html>"
    "<head>"
        "<title>" PAGE_TITLE "</title>");
}

static void writeHTMLHeaderClose(void)
{
    writeString(
    "</head>"
    "<body>");
}

static void writeStandardTextHeader(void)
{    
    writeHTMLHeader();
    writeHTMLHeaderClose();
}

static void writeReloadMainHeader()
{
    writeHTMLHeader();
    writeString("<meta http-equiv=\"refresh\" content=\"5;URL='/fulcrum'\" />");
    writeHTMLHeaderClose();
}

void Server_standardTextFooters(void)
{
    writeString(
    "</body>"
"</html>");
}

void Server_unauthorized(void)
{
    writeString("HTTP/1.1 401 Unauthorized\r\n");
    writeString("WWW-Authenticate: Basic realm=\"" PAGE_TITLE " Access Control\"\r\n");
    writeStandardTextHeader();
    writeString("401 Unauthorized");
    Server_standardTextFooters();
    Server_connectionCompleted();
}

void Server_badRequest(void)
{
    writeString("HTTP/1.1 400 Bad Request\r\n");
    writeStandardTextHeader();
    writeString("400 Bad Request");
    Server_standardTextFooters();
    Server_connectionCompleted();
}

void Server_notFound(void)
{
    writeString("HTTP/1.1 404 Not Found\r\n");
    writeStandardTextHeader();
    writeString("404 Not Found");
    Server_standardTextFooters();
    Server_connectionCompleted();
}

static void write200OK(void)
{
    writeString("HTTP/1.1 200 OK\r\n");
}

void Server_reloadStringResponse(const char *string)
{
    write200OK();
    writeReloadMainHeader();
    writeString(string);
    writeString("  Returning to <a href=\"/fulcrum\">main page</a> in five seconds.");
    Server_standardTextFooters();
    Server_connectionCompleted();
}

void Server_standardTextBegin(void)
{
    write200OK();
    writeStandardTextHeader();
}

static void outputDecimalNumber(uint32_t n)
{
    if (n == 0) {
        write("0", 1);
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
    write(buffer, length);
}

static void writeHexadecimalWord(uint32_t n)
{
    char buffer[8];
    int shift = 28;
    for (char *output=buffer; shift>=0; shift -= 4, ++output) {
        uint8_t digit = (uint8_t)((n >> shift) & 0xF);
        if (digit < 10) {
            *output = digit + '0';
        } else {
            *output = digit + 'A' - 10;
        }
    }
    write(buffer, 8);
}

static void outputTimeDuration(uint32_t seconds)
{
    bool first = true;
    if (seconds >= 86400) {
        uint32_t n = seconds/86400;
        outputDecimalNumber(n);
        writeString(" days");
        seconds -= n*86400;
        first = false;
    }
    if (seconds >= 3600) {
        if (!first)
            writeString(", ");
        uint32_t n = seconds/3600;
        outputDecimalNumber(n);
        writeString(" hours");
        seconds -= n*3600;
        first = false;
    }
    if (seconds >= 60) {
        if (!first)
            writeString(", ");
        uint32_t n = seconds/60;
        outputDecimalNumber(n);
        writeString(" minutes");
        seconds -= n*60;
        first = false;
    }
    
    if (!first)
        writeString(", ");
    outputDecimalNumber(seconds);
    writeString(" seconds");
}

static void writePeriod(void)
{
    writeString(".");
}

static void writeHostedRetryTime(void)
{
    writeString("<br>Automatic code restart in ");
    outputTimeDuration(systemStatus.detailed.userFaulted.retryTime);
    writePeriod();
}

static void writeDebugRegisterRowBegin()
{
    writeString("<tr><td>");
}

static void writeDebugRegisterRowEnd(uint32_t value)
{
    writeString("</td><td>");
    writeHexadecimalWord(value);
    writeString("</td></tr>");
}

static void writeDebugRegisterFixed(const char *name, uint32_t value)
{
    writeDebugRegisterRowBegin();
    writeString(name);
    writeDebugRegisterRowEnd(value);
}

static void writeDebugRegisters(const DebugRegisters *reg)
{
    writeString(
"<table border=\"1\" title=\"Registers\">");
    for (int r=0; r<13; ++r) {
        writeDebugRegisterRowBegin();
        writeString("R");
        outputDecimalNumber(r);
        writeDebugRegisterRowEnd(reg->r[r]);
    }
    writeDebugRegisterFixed("LR", reg->lr);
    writeDebugRegisterFixed("PC", reg->pc);
    writeDebugRegisterFixed("SP", reg->sp);
    writeDebugRegisterFixed("PSR", reg->psr);
}

static void writeTableRegisterHex(uint32_t value, const char *name)
{
    writeString("</td></tr><tr><td>");
    writeString(name);
    writeString("</td><td>");
    writeHexadecimalWord(value);
}

static void writeFaultDebug(void)
{
    writeString("<br><br>State at time of fault:<br>");
    writeDebugRegisters(&systemStatus.detailed.userFaulted.
        debug.fault.registers);
    writeString(
"<table border=\"1\" title=\"Fault Information\">"
    "<tr><td>SHCSR</td><td>");
        writeHexadecimalWord(systemStatus.detailed.userFaulted.
            debug.fault.data.SHCSR);
    writeTableRegisterHex(systemStatus.detailed.userFaulted.
         debug.fault.data.CFSR, "CFSR");
    writeTableRegisterHex(systemStatus.detailed.userFaulted.
        debug.fault.data.HFSR, "HFSR");
    writeTableRegisterHex(systemStatus.detailed.userFaulted.
        debug.fault.data.DFSR, "DFSR");
    writeTableRegisterHex(systemStatus.detailed.userFaulted.
        debug.fault.data.MMFAR, "MMFAR");
    writeTableRegisterHex(systemStatus.detailed.userFaulted.
        debug.fault.data.BFAR, "BFAR");
    writeTableRegisterHex(systemStatus.detailed.userFaulted.
        debug.fault.data.AFSR, "AFSR");
    writeString(
    "</td></tr>"
"</table>");
}

static void writeUserCodeHalted(const char *reason)
{
    writeString("Hosted application code halted ");
    writeString(reason);
}

static void writeUserCodeBasic(const char *tail) 
{
    writeString("Hosted application code ");
    writeString(tail);
}

static void writeBasicLink(const char *target, const char *text)
{
    writeString("<a href=\"/fulcrum/");
    writeString(target);
    writeString("\">");
    writeString(text);
    writeString("</a> ");
}

static void formBegin(const char *target)
{
    writeString("<br>"
"<form name=\"input\" action=\"/fulcrum/");
    writeString(target);
    writeString("\" method=\"post\" ");
}

static void writeBasicButton(const char *target, const char *text)
{
    formBegin(target);
    writeString("><input type=\"submit\" value=\"");
    writeString(text);
    writeString("\">"
"</form>");
}

void Server_mainPage(void)
{
    Server_standardTextBegin();
    
    writeString(Kernel_version());
    writeString(" with CC3000 version ");
    {
        uint16_t ver = ioctl_get_version();
        outputDecimalNumber(ver >> 8);
        writePeriod();
        outputDecimalNumber(ver & 0xFF);
    }
    writeString(" online for ");
    outputTimeDuration(uptime);
    writeString(".<br><br>System status: ");
    switch (systemStatus.state) {
    case RunState_Initialize:
    case RunState_ResetInitialize:
    case RunState_Setup:
        writeUserCodeBasic("not present.");
        break;
    
    case RunState_UserExecuting:
        writeUserCodeBasic("currently executing.");
        break;
    case RunState_UserHalted:
        writeUserCodeBasic("present but halted.");
        break;
    
    case RunState_FailureHalted_SocketIO:
        writeUserCodeHalted("due to excessive socket faults.");
        writeHostedRetryTime();
        break;
    case RunState_FailureHalted_SoftwareWatchdog:
        writeUserCodeHalted("due to a software watchdog fault.");
        writeHostedRetryTime();
        break;
    
    case RunState_ResetHalted_LowPower:
        writeUserCodeHalted("after a low power resume reset.");
        writeHostedRetryTime();
        break;
    case RunState_ResetHalted_WindowWatchDog:
        writeUserCodeHalted("after a window watchdog triggered reset.");
        writeHostedRetryTime();
        break;
    case RunState_ResetHalted_IndependentWatchDog:
        writeUserCodeHalted("after an independent watchdog triggered reset.");
        writeHostedRetryTime();
        break;
    case RunState_ResetHalted_SoftwareReset:
        writeUserCodeHalted("after a software triggered reset.");
        writeHostedRetryTime();
        break;
    
    case RunState_Fault_NMI:
        writeUserCodeHalted("after a non-maskable interrupt.");
        writeHostedRetryTime();
        writeFaultDebug();
        break;
    case RunState_Fault_Hard:
        writeUserCodeHalted("after a hard fault.");
        writeHostedRetryTime();
        writeFaultDebug();
        break;
    case RunState_Fault_Memory:
        writeUserCodeHalted("after a memory fault.");
        writeHostedRetryTime();
        writeFaultDebug();
        break;
    case RunState_Fault_Bus:
        writeUserCodeHalted("after a bus fault.");
        writeHostedRetryTime();
        writeFaultDebug();
        break;
    case RunState_Fault_Usage:
        writeUserCodeHalted("after a usage fault.");
        writeHostedRetryTime();
        writeFaultDebug();
        break;
    }
    
    writeString("<br><br><br>");

    if (systemStatus.state == RunState_UserExecuting) {
        writeString("Thread status: ");
        for (int i=1; i<MAXIMUM_THREADS; i++) {
            if (!Thread_isRunning(i))
                continue;
            writeString("<a href=\"/fulcrum/thread_");
            outputDecimalNumber(i);
            writeString("\">");
            outputDecimalNumber(i);
            writeString("</a> ");
        }
        
        writeString("<br><br><br>");
    }
    
    writeString("RAM: ");
    writeBasicLink("ram_kernel.bin", "kernel");
    if (Control_haveUserCode()) {
        writeBasicLink("ram_user.bin", "user");
        writeBasicLink("ram_all.bin", "all");
    }
    writeString("<br>Firmware: ");
    writeBasicLink("rom_kernel.bin", "kernel");
    if (Control_haveUserCode()) {
        writeBasicLink("rom_user.bin", "user");
        writeBasicLink("rom_all.bin", "all");
    }
    
    if (systemStatus.state == RunState_UserExecuting) {
        writeBasicButton("user_halt", "Stop User Code");
    } else if (Control_haveUserCode()) {
        writeBasicButton("user_start", "Start User Code");
    }
    writeBasicButton("system_reboot", "Reboot System");
    
    writeString("<br><br><br>");
    formBegin("user_upload");
    writeString("enctype=\"multipart/form-data\">"
    "Upload user code binary (");
    outputDecimalNumber(Kernel_totalFlashSize() - 
        ((uint32_t)(&_userbegin) - (uint32_t)(&_kernelbegin))/1024);
    writeString("KiB maximum):<br>"
    "<input type=\"file\" name=\"datafile\">"
    "<input type=\"submit\" value=\"Upload\">"
"</form>");
    
    Server_standardTextFooters();
    Server_connectionCompleted();
}

void Server_threadDebugInformation(int index)
{
    Server_standardTextBegin();
    
    writeString("Thread ");
    outputDecimalNumber(index);
    writeString(" debug information:<br>");
    
    DebugRegisters reg;
    Thread_debugInfo(index, &reg);
    writeDebugRegisters(&reg);
    
    Server_standardTextFooters();
    Server_connectionCompleted();
}

void Server_memoryAsFile(const char *filename, uint32_t addressStart,
                         uint32_t length)
{
    write200OK();
    writeStandardHeaders();
    writeString(
"Content-Type: application/octet-stream\r\n"
"Content-Disposition: attachment; filename=\"");
    writeString(filename);
    writeString("\"\r\n"
"Content-Length: ");
    outputDecimalNumber(length);
    writeString("\r\n"
"\r\n");


    controlStateStartTime = systick;
    controlData.staticSend.address = addressStart;
    controlData.staticSend.remaining = length;
    controlState = Upload_Memory;
}

void Server_uploadMemory(void)
{
    if (httpClientSocket < 0) {
        controlState = Idle;
        return;
    }
    if (Kernel_elapsed(controlStateStartTime) > 60 * TICK_RATE) {
        Server_connectionCompleted();
        return;
    }
    
    fd_set tx;
    FD_ZERO(&tx);    
    FD_SET(httpClientSocket, &tx);
    
    int rc = select(NULL, &tx, NULL, 1);
    if (rc < 0) {
        Control_networkFault();
        return;
    }
    if (rc == 0 || !FD_ISSET(httpClientSocket, &tx)) {
        Thread_yield();
        return;
    }
    
    int nSend = (int)controlData.staticSend.remaining;
    if (nSend > 1400)
        nSend = 1400;
        
    send(httpClientSocket, (void *)controlData.staticSend.address, nSend);
 
    controlData.staticSend.remaining -= nSend;
    controlData.staticSend.address += nSend;
    if (controlData.staticSend.remaining <= 0) {
        Server_connectionCompleted();
    }
}

void Server_handleRaw(HTTP_RawHandler handler)
{
    if (Kernel_elapsed(controlStateStartTime) > 60 * TICK_RATE) {
        Server_connectionCompleted();
        (handler)(controlData.base.buffer.data, 0, -1);
        return;
    }
    
    ServerReadStatus status = Server_read();
    switch (status) {
    case ServerRead_MoreData:
        break;
    case ServerRead_Nothing:
        return;
    case ServerRead_Error:
        (handler)(controlData.base.buffer.data, 0, -1);
        return;
    }
    
    if (!(handler)(controlData.base.buffer.data, controlData.base.buffer.length, 
            httpClientSocket)) {
        Server_connectionCompleted();
    } else {
        controlData.base.buffer.length = 0;
    }
}
