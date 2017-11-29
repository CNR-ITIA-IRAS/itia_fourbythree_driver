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
