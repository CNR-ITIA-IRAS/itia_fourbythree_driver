#include "ndlcom_driver/serial_communication.h"
#include <stdio.h>
#include <assert.h>

//#define DEBUG 1

#ifdef DEBUG
FILE *debugRXFile;
FILE *debugTXFile;
#endif

#ifdef __linux__
// -------------------------------------------------------------------------------------------------
//                             linux specific code
// -------------------------------------------------------------------------------------------------

#include <sys/select.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <termio.h>
#include <err.h>
#include <linux/serial.h>
// -----------------------------------------------
//               UART_connect
// -----------------------------------------------

#if USBCOM
#elif ETHCOM
  #include <itia_sutils/udp_client_server.h>
  udp_client_server::udp_server udp_server_ = NULL;
#else
  #error "None COM modality selected. Please add -DUSBCOM=1 -DETHCOM=0 or viceversa"
#endif

int UART_connect(const char *port, int baud, int parity, struct termios *oldtio)
{
    struct termios newtio;
    struct serial_struct serinfo;
    speed_t baudrate;
    int portHandle = 0;
    int alt_speed = 0;

#ifdef DEBUG
    printf("Try to open Serial port %s, with %u baud, parity %d.\n", port, baud, parity);

    debugRXFile = NULL;
    debugTXFile = NULL;
    if ((debugRXFile = fopen("debugRX.txt","wb")) == NULL) {
        printf("ERROR(UART_connect): Can't open debugRX.txt.\n");
        return -1;
    }
    if ((debugTXFile = fopen("debugTX.txt","wb")) == NULL) {
        printf("ERROR(UART_connect): Can't open debugTX.txt.\n");
        return -1;
    }
#endif

    if ((portHandle = open(port, O_RDWR | O_NOCTTY | O_NDELAY))<0) {
        printf("ERROR(UART_connect): Cannot open port '%s'\n",port);
        goto cleanup;
    }

#ifdef DEBUG
    printf("portHandle: %d\n",portHandle);
#endif

    fcntl(portHandle, F_SETFL, 0);

    /* we wanna have exclusive access to the port. see "man 4 tty_ioctl" (or "the internet") */
    ioctl(portHandle, TIOCEXCL);

    // save old settings
    if (oldtio!=NULL)
        tcgetattr(portHandle, oldtio);

    switch (baud) {
    case 300:
        baudrate = B300;
        break;
    case 1200:
        baudrate = B1200;
        break;
    case 9600:
        baudrate = B9600;
        break;
    case 19200:
        baudrate = B19200;
        break;
    case 38400:
        baudrate = B38400;
        break;
    case 57600:
        baudrate = B57600;
        break;
    case 115200:
        baudrate = B115200;
        break;
#ifdef B230400
    case 230400:
        baudrate = B230400;
        break;
#endif
#ifdef B500000
    case 500000:
        baudrate = B500000;
        break;
#endif
#ifdef B576000
    case 576000:
        baudrate = B576000;
        break;
#endif
#ifdef B921600
    case 921600:
        baudrate = B921600;
        break;
#endif
#ifdef B1000000
    case 1000000:
        baudrate = B1000000;
        break;
#endif
#ifdef B2000000
    case 2000000:
        baudrate = B2000000;
        break;
#endif
#ifdef B4000000
    case 4000000:
        baudrate = B4000000;
        break;
#endif
    default:
        printf("WARNING(UART_connect): baudrate %d is not a default speed, trying alternative mode for custom speed\n",baud);
        
        alt_speed = baud;
        baudrate = B38400;
        break;
    }

    // make newtio struct raw
    memset(&newtio,0,sizeof(newtio));
    cfmakeraw(&newtio);

    // set control mode: 8n1, local connection, enable receive
    newtio.c_cflag |= CLOCAL | CREAD;

    switch (parity) {
    case EVEN_PARITY:
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case ODD_PARITY:
        newtio.c_cflag |= PARENB | PARODD;
        break;
    default:
        newtio.c_cflag &= ~(PARENB & PARODD);
        break;
    }

    newtio.c_cc[VMIN] = 0;
    newtio.c_cc[VTIME] = 0;

#ifdef DEBUG
    printf("c_cflag = %o\n",(unsigned int)newtio.c_cflag);
    printf("c_lflag = %o\n",(unsigned int)newtio.c_lflag);
    printf("c_iflag = %o\n",(unsigned int)newtio.c_iflag);
    printf("c_oflag = %o\n",(unsigned int)newtio.c_oflag);
#endif

    /* store wanted in/out speeds in termio-structure */
    if( cfsetspeed(&newtio, baudrate)!= 0)
    {
        perror("serial_communication.c failed to encode input speed in termios structure");
        goto cleanup;
    }

    int closestSpeed;
    
    if (alt_speed > 0) {
        
        // configure port to use custom speed instead of 38400
        ioctl(portHandle, TIOCGSERIAL, &serinfo);
        serinfo.flags = (serinfo.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
        printf("Got serinfo.baud_base = %i from hardware.\n", serinfo.baud_base);
        serinfo.custom_divisor = (serinfo.baud_base + (alt_speed / 2)) / alt_speed;
        printf("Calculated custom serinfo.custom_divisor = %i.\n", serinfo.custom_divisor);
        closestSpeed = serinfo.baud_base / serinfo.custom_divisor;

        if (closestSpeed < alt_speed * 98 / 100 || closestSpeed > alt_speed * 102 / 100) {
            fprintf(stderr, "Cannot set serial port speed to custom speed %i. Closest possible is %i\n", alt_speed, closestSpeed);
            goto cleanup;
        }

        ioctl(portHandle, TIOCSSERIAL, &serinfo);
    }

    // flush the port and set new settings
    tcflush(portHandle, TCIFLUSH);
    tcsetattr(portHandle,TCSANOW,&newtio);

    /* check if output speed was really set correctly! */
    struct termios checktio;
    tcgetattr(portHandle, &checktio);
    
    if (alt_speed <= 0) {
    if (baudrate != cfgetispeed(&checktio))
    {
        printf("serial_communication.c failed to set input speed for %s to %i\n",port,baud);
        goto cleanup;
    }
    if (baudrate != cfgetospeed(&checktio))
    {
        printf("serial_communication.c failed to set input speed for %s to %i\n",port,baud);
        goto cleanup;
    }
    } else {
        ioctl(portHandle, TIOCGSERIAL, &serinfo);
        closestSpeed = serinfo.baud_base / serinfo.custom_divisor;
        
        if (closestSpeed < alt_speed * 98 / 100 || closestSpeed > alt_speed * 102 / 100) {
            fprintf(stderr, "serial_communication.c failed to set custom serial port speed for  %s to %i (is: %i)\n", port, alt_speed, closestSpeed);
            goto cleanup;
        }
    }

#ifdef DEBUG
    printf("Connect OK\n");
#endif

    /* ordinary end of this function */
    return portHandle;

    /* removing any mess we did */
cleanup:

#ifdef DEBUG
    if (debugRXFile!=NULL)
    {
        fclose(debugRXFile);
        debugRXFile = NULL;
    }
    if (debugTXFile!=NULL)
    {
        fclose(debugTXFile);
        debugTXFile = NULL;
    }
#endif
    if (oldtio!=0)
    {
        tcsetattr(portHandle, TCSANOW, oldtio);
    }
    if (portHandle!=0)
    {
        close(portHandle);
        portHandle = 0;
    }
    return -1;
}

// -----------------------------------------------
//              UART_disconnect
// -----------------------------------------------
void UART_disconnect(int portHandle, struct termios *tio)
{
#ifdef DEBUG
    printf("Close serial port.\n");
#endif

    // restore old port settings
    if (tio!=NULL)
        tcsetattr(portHandle,TCSANOW,tio);

    // close port
    close(portHandle);

#ifdef DEBUG
    // close debug files
    fclose(debugRXFile);
    fclose(debugTXFile);
#endif
}

// -----------------------------------------------
//              UART_send
// -----------------------------------------------
int UART_send(int portHandle, const char *buf, size_t len)
{
    unsigned int count = 0;

    if (portHandle<0) {
        printf("ERROR(UART_send): Port not open.\n");
        return -1;
    }

    // wait for output buffer empty
    if (tcdrain(portHandle) != 0) {
        printf("ERROR(UART_send): tcdrain reports '%s'\n", strerror(errno));
    }

    // write data
    while(count<len)
    {
        int ret = write(portHandle, buf+count, len-count);
        assert(ret > 0);
        count += ret;
    }

#ifdef DEBUG
    // write txData to debug file
    fwrite(buf, 1, count, debugTXFile);
    fflush(debugTXFile);
#endif

    return count;
}

// -----------------------------------------------
//              UART_receive
// -----------------------------------------------
int UART_receive(int portHandle, char *buf, size_t maxLen)
{
    return UART_receive_timeout(portHandle, buf, maxLen, NULL);
}

int UART_receive_timeout(int portHandle, char *buf, size_t maxLen, struct timeval *timeout)
{
    int ret, count;
    fd_set readset;

    if (portHandle<0) {
        printf("ERROR(UART_receive): Port not open.\n");
        return -1;
    }

    // call select to block here until data is available
    FD_ZERO(&readset);
    FD_SET(portHandle, &readset);
    while (1) {
        // this will block indefinetely...
        ret = select(portHandle + 1, &readset, NULL, NULL, timeout);
        // ...and return a value "> 0" if some fd got ready (and "0" on timeout)
        if (ret > 0)
            break;
        else if (ret == 0)
            return 0;
        else {
            // sets errno if interrupted by system call.
            // see: http://stackoverflow.com/a/16565576
            switch (errno) {
            case EINTR:
                continue;
            default:
                printf("ERROR(UART_receive): select() failed with '%s'\n", strerror(errno));
                return -1;
            }
        }
    }

    // read data
    count = read(portHandle, buf, maxLen);

#ifdef DEBUG
    // write rxData to debug file
    fwrite(buf, 1, count, debugRXFile);
    fflush(debugRXFile);
#endif

    return count;
}






#else // win
// -------------------------------------------------------------------------------------------------
//                             windows specific code
// -------------------------------------------------------------------------------------------------

// -----------------------------------------------
//               UART_connect
// -----------------------------------------------
HANDLE UART_connect(const char *port, int baud, int parity)
{

    HANDLE portHandle;

    printf("Try to open Serial port %s, with %u baud, parity %d\n", port, baud, parity);

    if (parity!=0) {
        printf("WARNING(UART_connect): EVEN/ODD parity not supported yet (connect without parity)!\n");
        parity=0;
    }

#ifdef DEBUG
    debugRXFile = fopen("debugRX.txt","wb");
    debugTXFile = fopen("debugTX.txt","wb");
#endif

    // convert port names >10 to crazy windows name
    char portName[20];
    if (strlen(port)>4)
        sprintf(portName,"\\\\.\\%s",port);
    else
        sprintf(portName,"%s",port);

    portHandle = CreateFileA(portName,
                             GENERIC_WRITE | GENERIC_READ,
                             0,
                             0,
                             OPEN_EXISTING,
                             0, //FILE_ATTRIBUTE_NORMAL,
                             0);

    if (portHandle == INVALID_HANDLE_VALUE) {
        printf("ERROR(UART_connect): CreateFile failed.\n");
        printLastError();
        return INVALID_HANDLE_VALUE;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(portHandle, &dcbSerialParams)) {
        printLastError();
        UART_disconnect(portHandle);
        return INVALID_HANDLE_VALUE;
    }

    switch (baud) {
    case 9600:
        dcbSerialParams.BaudRate = CBR_9600;
    case 14400:
        dcbSerialParams.BaudRate = CBR_14400;
    case 38400:
        dcbSerialParams.BaudRate = CBR_38400;
    case 57600:
        dcbSerialParams.BaudRate = CBR_57600;
    case 115200:
        dcbSerialParams.BaudRate = CBR_115200;
    default:
        dcbSerialParams.BaudRate = CBR_115200;
    }

    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;

    dcbSerialParams.fBinary  = TRUE;
    dcbSerialParams.fParity  = FALSE;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(portHandle, &dcbSerialParams)) {
        printLastError();
        UART_disconnect(portHandle);
        return INVALID_HANDLE_VALUE;
    }

    // Setting the timeout parameters for the serial port
    COMMTIMEOUTS timeouts={0};
    timeouts.ReadIntervalTimeout=MAXDWORD;//50;
    timeouts.ReadTotalTimeoutConstant=0;//50;
    timeouts.ReadTotalTimeoutMultiplier=0;//MAXDWORD;//0;//10;
    timeouts.WriteTotalTimeoutConstant=50;
    timeouts.WriteTotalTimeoutMultiplier=10;

    if (!SetCommTimeouts(portHandle, &timeouts)) {
        printLastError();
        UART_disconnect(portHandle);
        return INVALID_HANDLE_VALUE;
    }

    return portHandle;
}


void UART_disconnect(HANDLE portHandle)
{
    printf("UART_disconnect: Close serial port\n");
    CloseHandle(portHandle);

#ifdef DEBUG
    // close debug files
    fclose(debugRXFile);
    fclose(debugTXFile);
#endif
}

int UART_send(HANDLE portHandle, const char *buf, int len)
{
    DWORD count;

    if (portHandle == INVALID_HANDLE_VALUE) {
        printf("ERROR(UART_send): can't send data - port is not open\n");
        return -1;
    }

    if (!WriteFile(portHandle,buf,len,&count,0)) {
        printf("ERROR(UART_send): WriteFile failed.\n");
        printLastError();
        return -1;
    }

    if (count!=len)
        printf("WARNING(UART_send): only send %d of %d bytes\n",(int)count,len);

    if (!FlushFileBuffers(portHandle))
        printf("ERROR(UART_send): FlushFileBuffers failed.\n");

#ifdef DEBUG
    // write txData to debug file
    fwrite(buf, 1, count, debugTXFile);
    fflush(debugTXFile);
#endif

    return count;
}

int UART_receive(HANDLE portHandle, char *buf, int maxLen)
{
    DWORD count;
  
    if (portHandle == INVALID_HANDLE_VALUE) {
        printf("ERROR(UART_receive): can't send data - port is not open\n");
        return -1;
    }

    // TODO: blocking read
    do {
        if (!ReadFile(portHandle,buf,maxLen,&count,0)) {
            printf("ERROR(UART_receive): ReadFile failed.\n");
            printLastError();
            return -1;
        }
    } while (count < 1);

#ifdef DEBUG
    // write rxData to debug file
    fwrite(buf, 1, count, debugRXFile);
    fflush(debugRXFile);
#endif

    return count;
}

void printLastError()
{
    char lastError[1024];
    FormatMessageA(
                   FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                   NULL,
                   GetLastError(),
                   MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                   lastError,
                   1024,
                   NULL);

    printf("ERROR(SerialCommunication): LastError: %s\n",lastError);
}

#endif // lin/win
