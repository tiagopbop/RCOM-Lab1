// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG            0x7E
#define A_TRANS         0x03

#define C_SET           0x03
#define C_UA            0x07

#define C_RR(sequenceNum) (0xAA | (sequenceNum))
#define C_REJ(sequenceNum) (0x54 | (sequenceNum))
#define C_SEQ(sequenceNum) (sequenceNum << 7)

#define C_DISC          0x0B
#define ESCAPE          0x7D

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE,
    DATA_STATE,
    ESCAPE_STATE
} State;

LinkLayer info;
State state = START;

int sequenceNum = 0;

volatile int alarmSet = FALSE;
int alarmCount = 0;

unsigned int totalFramesExchanged = 0;
unsigned int retries = 0;

unsigned char byte;

void alarmHandler(int signal) {
    alarmSet = FALSE;
    alarmCount++;
    retries++;
    printf("\nCouldn't receive frame in time - Retrying...\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {

    info = connectionParameters;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return -1;
    }

    int bytesWritten = 0;

    if (connectionParameters.role == LlTx) {
        (void)signal(SIGALRM, alarmHandler);

        unsigned char frame_test[5] = {FLAG, A_TRANS, C_SET, A_TRANS ^ C_SET, FLAG};

        alarmCount = 0;
        
        while (alarmCount < connectionParameters.nRetransmissions) {
            
            bytesWritten = writeBytesSerialPort(frame_test, 5);
            totalFramesExchanged++;

            if (bytesWritten == 5) {
                alarm(connectionParameters.timeout);
                alarmSet = TRUE;
            } else {
                printf("Error while writting test frame\n");
                closeSerialPort();
                return -1;
            }

            state = START;

            while (alarmSet && state != STOP_STATE) {

                int byteRead = readByteSerialPort(&byte);

                if (byteRead == 1) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                            break;
                        case FLAG_RCV:
                            if (byte == A_TRANS) {
                                state = A_RCV;
                            } else if (byte != FLAG){
                                state = START;
                            }
                            break;
                        case A_RCV:
                            if (byte == C_UA) {
                                state = C_RCV;
                            } else if (byte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                            break;
                        case C_RCV:
                            if (byte == (A_TRANS ^ C_UA)) {
                                state = BCC_OK;
                            } else if (byte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                            break;
                        case BCC_OK:
                            if (byte == FLAG) {
                                state = STOP_STATE;
                            } else {
                                state = START;
                            }
                            break;
                        default:
                            break;
                    }
                }
            }

            if (state == STOP_STATE) {
                alarm(0);
                alarmSet = FALSE;
                printf("Connection successfully tested and working!\n\n");
                return 1;
            }
        }

        printf("Opening connection failed - Too many attempts\n");
        closeSerialPort();
        return -1;

    } else if (connectionParameters.role == LlRx) {

        while (state != STOP_STATE) {

            int byteRead = readByteSerialPort(&byte);

            if (byteRead == 1) {
                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_TRANS) {
                            state = A_RCV;
                        } else if (byte != FLAG) {
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (byte == C_SET) {
                            state = C_RCV;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte == (A_TRANS ^ C_SET)) {
                            state = BCC_OK;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        }
                        break;
                    default:
                        break;
                }
            }

            if (state == STOP_STATE) {
                unsigned char answer_test[5] = {FLAG, A_TRANS, C_UA, A_TRANS ^ C_UA, FLAG};

                bytesWritten = writeBytesSerialPort(answer_test, 5);
                totalFramesExchanged++;

                if (bytesWritten == 5) {
                    return 1;
                } else {
                    printf("Error while writing response test frame\n");
                    closeSerialPort();
                    return -1;
                }
            }
        }

    }
    
    printf("Error on recognizing role\n");
    alarm(0);
    closeSerialPort();
    return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {

    unsigned char frame[bufSize * 2 + 8];

    unsigned char control_byte = 0;
    unsigned char bcc2 = 0;

    alarmCount = 0;
    alarmSet = FALSE;

    frame[0] = FLAG;
    frame[1] = A_TRANS;
    frame[2] = C_SEQ(sequenceNum);
    frame[3] = A_TRANS ^ C_SEQ(sequenceNum);

    int n = 4;

    for (int i = 0; i < bufSize; i++) {
        unsigned char current_byte = buf[i];

        switch (current_byte) {
            case ESCAPE:
                frame[n] = ESCAPE; 
                n++;
                frame[n] = ESCAPE ^ 0x20; 
                n++;
                break;
            case FLAG:
                frame[n] = ESCAPE; 
                n++;
                frame[n] = FLAG ^ 0x20; 
                n++;
                break;
            default:
                frame[n] = current_byte; 
                n++;
                break;
        }

        bcc2 ^= current_byte;
    }

    if (bcc2 == FLAG) {
        frame[n] = ESCAPE;
        n++;
        frame[n] = FLAG ^ 0x20;
        n++;
    } else if (bcc2 == ESCAPE) {
        frame[n] = ESCAPE; 
        n++;
        frame[n] = ESCAPE ^ 0x20;
        n++;
    } else {
        frame[n] = bcc2;
        n++;
    }

    frame[n] = FLAG;
    n++;

    int bytesWritten = 0;

    sequenceNum = 1 - sequenceNum;

    while (alarmCount < info.nRetransmissions) {

        bytesWritten = writeBytesSerialPort(frame, n);;
        totalFramesExchanged++;

        if (bytesWritten == n) {
                alarm(info.timeout);
                alarmSet = TRUE;
            } else {
                printf("Error while writting frame\n");
                closeSerialPort();
                return -1;
            }
        
        state = START;

        while (alarmSet && state != STOP_STATE) {

            int byteRead = readByteSerialPort(&byte);

            if (byteRead > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_TRANS) {
                            state = A_RCV;
                        } else if (byte != FLAG) {
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (byte == C_RR(sequenceNum) || byte == C_REJ(1 - sequenceNum)) {
                            control_byte = byte;
                            state = C_RCV;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte == (A_TRANS ^ control_byte)) {
                            state = BCC_OK;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        if (state == STOP_STATE && control_byte == C_RR(sequenceNum)) {
            alarm(0);
            printf("Packet exchanged successfully!\n");
            return n;
        }
    }
    return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llread(unsigned char *packet) {
    unsigned char bcc2 = 0;
    int bytesWritten = 0;
    int n = 0;

    state = START;

    while (state != STOP_STATE) {
        int byteRead = readByteSerialPort(&byte);

        if (byteRead > 0) {
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_TRANS) {
                        state = A_RCV;
                    } else if (byte != FLAG) {
                        state = START;
                    }
                    break;
                case A_RCV:
                    if (byte == C_SEQ(sequenceNum)) {
                        state = C_RCV;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_TRANS ^ C_SEQ(sequenceNum))) {
                        state = BCC_OK;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        state = STOP_STATE;
                    } else if (byte == ESCAPE) {
                        state = ESCAPE_STATE;
                    } else {
                        packet[n] = byte; 
                        n++;
                        state = DATA_STATE;
                    }
                    break;
                case DATA_STATE:
                    if (byte == ESCAPE) {
                        state = ESCAPE_STATE;
                    } else if (byte == FLAG) {
                        state = STOP_STATE;
                        n--;
                    } else {
                        packet[n] = byte; 
                        n++;
                    }
                    break;
                case ESCAPE_STATE:
                    if (byte == (FLAG ^ 0x20)) {
                        packet[n] = FLAG; 
                        n++;
                        state = DATA_STATE;
                    } else if (byte == (ESCAPE ^ 0x20)) {
                        packet[n] = ESCAPE; 
                        n++;
                        state = DATA_STATE;
                    } else {
                        state = START;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    for (int i = 0; i < n; i++) {
        bcc2 ^= packet[i];
    }

    if (bcc2 != packet[n]) {
        printf("\nError - Mismatch of the BCC2\n");

        unsigned char answer[5] = {FLAG, A_TRANS, C_REJ(sequenceNum), A_TRANS ^ C_REJ(sequenceNum), FLAG};

        bytesWritten = writeBytesSerialPort(answer, 5);
        totalFramesExchanged++;

        if (bytesWritten != 5) {
            printf("Error writing answer frame\n");
        }

        return -1;
    }
    sequenceNum = 1 - sequenceNum;

    unsigned char answer[5] = {FLAG, A_TRANS, C_RR(sequenceNum), A_TRANS ^ C_RR(sequenceNum), FLAG};

    bytesWritten = writeBytesSerialPort(answer, 5);
    totalFramesExchanged++;

    if (bytesWritten == 5) {
        printf("\nPacket read successfully!\n");
        return n;
    } else {
        printf("Error writing answer frame\n");
        return -1;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llclose(int showStatistics) {

    printf("\nClosing serial port connection...\n");

    unsigned char disc[5] = {FLAG, A_TRANS, C_DISC, A_TRANS ^ C_DISC, FLAG};
    unsigned char ua[5] = {FLAG, A_TRANS, C_UA, A_TRANS ^ C_UA, FLAG};

    alarmCount = 0;
    state = START;

    if (info.role == LlTx) {

        while (alarmCount < info.nRetransmissions) {

            int bytesWritten = writeBytesSerialPort(disc, 5);
            totalFramesExchanged++;

            if (bytesWritten == 5) {
                alarm(info.timeout);
                alarmSet = TRUE;
            } else {
                perror("Error sending first DISC frame");
                return -1;
            }

            while (alarmSet) {

                int bytesRead = readByteSerialPort(&byte);

                if (bytesRead > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            if (byte == A_TRANS) {
                                state = A_RCV;
                            } else if (byte != FLAG) {
                                state = START;
                            }
                            break;
                        case A_RCV:
                            if (byte == C_DISC) {
                                state = C_RCV;
                            } else if (byte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                            break;
                        case C_RCV:
                            if (byte == (A_TRANS ^ C_DISC)) {
                                state = BCC_OK;
                            } else if (byte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                            break;
                        case BCC_OK:
                            if (byte == FLAG) {
                                state = STOP_STATE;
                            } else {
                                state = START;
                            }
                            break;
                        default:
                            break;
                    }
                }
            }

            if (state == STOP_STATE) {
                alarm(0);
                alarmSet = FALSE;
                break;
            }
        }

        if (state != STOP_STATE) {
            printf("Failed to receive second DISC frame\n");
            return -1;
        }

        int bytesWritten = writeBytesSerialPort(ua, 5);
        totalFramesExchanged++;

        if (bytesWritten != 5) {
            perror("Error sending final UA frame");
            return -1;
        }

    } else if (info.role == LlRx) {
        
        while (state != STOP_STATE) {

            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_TRANS) {
                            state = A_RCV;
                        } else if (byte != FLAG) {
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (byte == C_DISC) {
                            state = C_RCV;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte == (A_TRANS ^ C_DISC)) {
                            state = BCC_OK;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                            alarmSet = FALSE;
                        } else {
                            state = START;
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        if (state != STOP_STATE) {
            printf("Failed to receive first DISC frame\n");
            return -1;
        }

        int bytesWritten = writeBytesSerialPort(disc, 5);;
        totalFramesExchanged++;

        if (bytesWritten != 5) {
            perror("Error sending second DISC frame");
            return -1;
        }

        state = START;

        while (state != STOP_STATE) {

            int byteRead = readByteSerialPort(&byte);

            if (byteRead > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_TRANS) {
                            state = A_RCV;
                        } else if (byte != FLAG) {
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (byte == C_UA) {
                            state = C_RCV;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte == (A_TRANS ^ C_UA)) {
                            state = BCC_OK;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }

    if (showStatistics) {
        printf("\n-----------------------------------------\n");
        printf("Statistics:\n");
        printf("\nTotal number of frames exchanged successfully: %d\n", totalFramesExchanged);
        printf("Total number of retries needed: %d\n", retries);
        printf("\n-----------------------------------------\n");
    }

    int closed = closeSerialPort();
    return closed;
}
