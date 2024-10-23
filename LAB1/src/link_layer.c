// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E 
#define A_Tx 0x03
#define C_SET 0x03
#define C_UA 0x07

typedef enum {
    START,      
    FLAG_RCV,  
    A_RCV,      
    C_RCV,        
    BCC_OK,     
    STOP_STATE     
} State;

unsigned char byte_test;
int bytesWritten_test;
int bytesReceived_test;
volatile int alarmSet = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmSet = FALSE;
    alarmCount++;
    printf("UA not received - Retrying\n");
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    (void)signal(SIGALRM, alarmHandler);
    State state = START;
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return -1;
    }

    if (connectionParameters.role == LlTx) {
        unsigned char message_test[5] = {FLAG, A_Tx, C_SET, A_Tx ^ C_SET, FLAG};

        while (alarmCount < connectionParameters.nRetransmissions) {
            if (!alarmSet) {
                bytesWritten_test = writeBytesSerialPort(message_test, 5);

                if (bytesWritten_test == 5) {
                    alarmSet = TRUE;
                    alarm(connectionParameters.timeout);
                } else {
                    perror("Byte writing error");
                    closeSerialPort();
                    return -1;
                }
            }

            bytesReceived_test = readByteSerialPort(&byte_test);

            if (bytesReceived_test == 1) {
                switch (state) {
                    case START:
                        if (byte_test == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte_test == A_Tx) {
                            state = A_RCV;
                        } else if (byte_test != FLAG){
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (byte_test == C_UA) {
                            state = C_RCV;
                        } else if (byte_test == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte_test == (A_Tx ^ C_UA)) {
                            state = BCC_OK;
                        } else if (byte_test == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte_test == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        } 
                        break;    
                    case STOP_STATE:
                        break;
                }
            }

            if (state == STOP_STATE) {
                alarm(0);
                alarmSet = FALSE;
                printf("Connection successfully tested and working\n");
                closeSerialPort();
                return 1;
            }
        }

        perror("Failed connection - Too many attempts");
        closeSerialPort();
        return -1;

    } else if (connectionParameters.role == LlRx) {

        int retryCount = 0;

        while (retryCount < connectionParameters.nRetransmissions) {
            bytesReceived_test = readByteSerialPort(&byte_test);

            if (bytesReceived_test == 1) {
                switch (state) {
                    case START:
                        if (byte_test == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte_test == A_Tx) {
                            state = A_RCV;
                        } else if (byte_test != FLAG) {
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (byte_test == C_SET) {
                            state = C_RCV;
                        } else if (byte_test == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte_test == (A_Tx ^ C_SET)) {
                            state = BCC_OK;
                        } else if (byte_test == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte_test == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        } 
                        break;    
                    case STOP_STATE:
                        break;
                }
            }

            if (state == STOP_STATE) {
                unsigned char answer_test[5] = {FLAG, A_Tx, C_UA, A_Tx ^ C_UA, FLAG};

                bytesWritten_test = writeBytesSerialPort(answer_test, 5);

                if (bytesWritten_test != 5) {
                    perror("Error writing response frame");
                    closeSerialPort();
                    return -1;
                }
                
                break;
            }

            retryCount++;
            usleep(connectionParameters.timeout * 1000);
        }

        if (retryCount >= connectionParameters.nRetransmissions) {
            perror("Receiver failed - Too many attempts");
            closeSerialPort();
            return -1;
        }

        closeSerialPort();
        return 1;

    } else {
        perror("Error on recognizing role");
        alarm(0);
        closeSerialPort();
        return -1;
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
