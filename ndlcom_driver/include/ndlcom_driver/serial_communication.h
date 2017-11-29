
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

/**
 * @addtogroup Communication
 * @{
 * @addtogroup Communication_Serial
 * @{
 * @addtogroup Communication_Serial_Communication Low level communication
 * @{
 */
#ifndef _SERIALCOMMUNICATION_H_
#define _SERIALCOMMUNICATION_H_

#define MAX_BUFFER_LENGTH 2048 // BUG: acm-irgedwas linux-treiber hat Probleme mit ]1,2048[ buffer size (DON'T CHANGE!)

#define NO_PARITY   0
#define EVEN_PARITY 1
#define ODD_PARITY  2

#ifdef linux
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#else // win
#include <windows.h>
#endif // lin/win

/*! @brief Try to establish a connection to a serial port.
 *
 *  @param port serial port to connect to
 *  @param baud baud rate for the serial port
 *  @param parity for the serial port (NO_PARITY, EVEN_PARITY, ODD_PARITY)
 *  @param oldtio old setting of the serial port (linux only)
 *  @return lin: >0 handle for the serial port
 *              <=0 error
 *          win: INVALID_HANDLE_ERROR error
 *               else handle for the serial port
 */
#ifdef linux
int  UART_connect(const char *port, int baud, int parity, struct termios *oldtio);
int  UART_connect(const char *address, int port, int parity);
#else // win
HANDLE UART_connect(const char *port, int baud, int parity);
#endif // lin/win

#ifdef linux
void UART_disconnect(int portHandle, struct termios *tio);
#else // win
void UART_disconnect(HANDLE portHandle);
#endif // lin/win

#ifdef linux
int UART_send(int portHandle, const char *buf, size_t len);
#else // win
int UART_send(HANDLE portHandle, const char *buf, int len);
#endif // lin/win

#ifdef linux
int UART_receive(int portHandle, char *buf, size_t maxLen);
int UART_receive_timeout(int portHandle, char *buf, size_t maxLen, struct timeval *timeout);
#else // win
int UART_receive(HANDLE portHandle, char *buf, int maxLen);
void printLastError();
#endif // lin/win


#endif //_SERIALCOMMUNICATION_H_

/**
 * @}
 * @}
 * @}
 */
