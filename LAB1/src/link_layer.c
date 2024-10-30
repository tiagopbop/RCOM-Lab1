// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E 
#define A_Tx 0x03
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B

#define ESCAPE 0x7D
#define SEQ0 0x00
#define SEQ1 0x80
#define RR 0x05
#define REJ 0x01

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA,
    STOP_STATE
} State;

LinkLayer info;

unsigned char byte;
volatile int alarmSet = FALSE;
int alarmCount;

int seqNum = 0;

void alarmHandler(int signal) {
    alarmSet = FALSE;
    alarmCount++;
    printf("Time exceeded - Retrying\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    (void)signal(SIGALRM, alarmHandler);

    info = connectionParameters;

    State state = START;
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return -1;
    }

    int bytesWritten;

    if (connectionParameters.role == LlTx) {
        unsigned char message_test[5] = {FLAG, A_Tx, C_SET, A_Tx ^ C_SET, FLAG};

        alarmCount = 0;

        while (alarmCount < connectionParameters.nRetransmissions) {

            if (!alarmSet) {
                bytesWritten = writeBytesSerialPort(message_test, 5);

                if (bytesWritten == 5) {
                    alarmSet = TRUE;
                    alarm(connectionParameters.timeout);
                } else {
                    printf("Byte writing error\n");
                    closeSerialPort();
                    return -1;
                }
            }

            int byteReceived = readByteSerialPort(&byte);

            if (byteReceived == 1) {
                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_Tx) {
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
                        if (byte == (A_Tx ^ C_UA)) {
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
                    case STOP_STATE:
                        break;
                    default:
                        state = START;
                        break;
                }
            }

            if (state == STOP_STATE) {
                alarm(0);
                alarmSet = FALSE;
                printf("Connection successfully tested and working\n");
                return 1;
            }
        }

        printf("Failed connection - Too many attempts");
        closeSerialPort();
        return -1;

    } else if (connectionParameters.role == LlRx) {

        int count = 0;

        while (count != bytesWritten) {
            int byteReceived = readByteSerialPort(&byte);

            if (byteReceived == 1) {
                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_Tx) {
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
                        if (byte == (A_Tx ^ C_SET)) {
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
                    case STOP_STATE:
                        break;
                    default:
                        state = START;
                        break;
                }
            }

            if (state == STOP_STATE) {
                unsigned char answer_test[5] = {FLAG, A_Tx, C_UA, A_Tx ^ C_UA, FLAG};

                bytesWritten = writeBytesSerialPort(answer_test, 5);

                if (bytesWritten != 5) {
                    printf("Error writing response frame\n");
                    closeSerialPort();
                    return -1;
                }
                
                break;
            }

            count++;
        }

        if (state != STOP_STATE) {
            printf("Receiver failed - Too many attempts\n");
            closeSerialPort();
            return -1;
        }

        return 1;

    } else {
        printf("Error on recognizing role\n");
        alarm(0);
        closeSerialPort();
        return -1;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {

    unsigned char frame[MAX_PAYLOAD_SIZE * 2 + 10];
    unsigned char controlByte;

    frame[0] = FLAG; 
    frame[1] = A_Tx; 

    if (seqNum == 0) {
        controlByte = SEQ0;
    } else {
        controlByte = SEQ1;
    }

    frame[2] = controlByte;
    frame[3] = A_Tx ^ controlByte;

    int n = 4;

    unsigned char bcc2 = 0;
    unsigned char current;

    for (int i = 0; i < bufSize; i++) {
        current = buf[i];

    switch (current) {
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
            frame[n] = current; 
            n++;
            break;
        }

        bcc2 ^= current;
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

    alarmCount = 0;
    while (alarmCount < info.nRetransmissions) {

        int bytesWritten;

        if (!alarmSet) {
            bytesWritten = writeBytesSerialPort(frame, n);

            if (bytesWritten == n) {
                printf("Successfully written\n");
                alarmSet = TRUE;
                alarm(info.timeout);
            } else {
                printf("Error writing frame\n");
                return -1;
            }
        }

        //usleep(info.timeout * 1000);

        int byteReceived = readByteSerialPort(&byte);

        if (byteReceived == 1) {

            if (byte == (RR | (seqNum << 7))) {

                if (seqNum == 0) { 
                    seqNum = 1;
                } else {
                    seqNum = 0;
                }

                alarm(0);
                alarmSet = FALSE;
                return bytesWritten;
            }

            if (byte == (REJ | (seqNum << 7))) { 
                alarmCount++;
                alarm(info.timeout);
                continue;
            }
        }
    }

    alarm(0);
    printf("Max retransmissions reached\n");
    return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llread(unsigned char *packet) {
    State state = START;

    int bytesRead = 0;
    unsigned char bcc2 = 0;
    int n = 0;
    int stuffy = 0;

    while (state != STOP_STATE) {

        int byteReceived = readByteSerialPort(&byte);

        if (byteReceived == 1) {
            bytesRead++;
            unsigned char controlByte;
            switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    n = 0;
                }
                break;

            case FLAG_RCV:
                if (byte == A_Tx) {
                    state = A_RCV;
                } else if (byte != FLAG) {
                    state = START;
                }
                break;

            case A_RCV:
                if (byte == (SEQ0 | seqNum) || byte == (SEQ1 | seqNum)) {
                    controlByte = byte;
                    state = C_RCV;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case C_RCV:
                if (byte == (A_Tx ^ controlByte)) {
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
                    state = DATA;
                    packet[n++] = byte;
                    bcc2 = byte;
                }
                break;

            case DATA:
                if (byte == ESCAPE) {
                    stuffy = 1;
                    continue;
                }

                if (stuffy) {
                    byte ^= 0x20;
                    stuffy = 0;
                }

                if (byte == FLAG) {

                    if (bcc2 != 0) {

                        unsigned char rej[5] = {FLAG, A_Tx, REJ | (seqNum << 7), A_Tx ^ (REJ | (seqNum << 7)), FLAG};
                        writeBytesSerialPort(rej, 5);
                        return -1;
                    }
                    state = STOP_STATE;
                } else {
                    packet[n++] = byte;
                    bcc2 ^= byte;
                }
                break;

            case STOP_STATE:
                break;

            default:
                state = START;
                break;
            }
        }
    }

    unsigned char rr[5] = {FLAG, A_Tx, RR | (seqNum << 7), A_Tx ^ (RR | (seqNum << 7)), FLAG};
    writeBytesSerialPort(rr, 5);

    if (seqNum == 0) { 
        seqNum = 1;
    } else {
        seqNum = 0;
    }
    
    printf("Successfully read\n");
    return bytesRead - 2;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LLCLOSE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int llclose(int showStatistics) {
    
    State state = START;
    unsigned char discFrame[5] = {FLAG, A_Tx, C_DISC, A_Tx ^ C_DISC, FLAG}; 
    unsigned char uaFrame[5] = {FLAG, A_Tx, C_UA, A_Tx ^ C_UA, FLAG};  

    int bytesWritten;   

    if (info.role == LlTx) {

        alarmCount = 0;
        while (alarmCount < info.nRetransmissions) {
            if (!alarmSet) {
                bytesWritten = writeBytesSerialPort(discFrame, 5);
                if (bytesWritten != 5) {
                    printf("Error writing DISC frame\n");
                    return -1;
                }
                alarmSet = TRUE;
                alarm(info.timeout);
            }

            int bytesReceived = readByteSerialPort(&byte);

            if (bytesReceived == 1) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_Tx) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == C_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_Tx ^ C_DISC)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) state = STOP_STATE;
                        else state = START;
                        break;
                    case STOP_STATE:
                        break;
                    default:
                        state = START;
                        break;
                }
            }

            if (state == STOP_STATE) {
                alarm(0);
                alarmSet = FALSE;
                printf("DISC frame received from receiver\n");
                break;
            }
        }

        if (state != STOP_STATE) {
            printf("Failed to receive DISC frame from receiver\n");
            return -1;
        }

        bytesWritten = writeBytesSerialPort(uaFrame, 5);
        if (bytesWritten != 5) {
            printf("Error writing UA frame\n");
            return -1;
        }

    } else if (info.role == LlRx) {
        state = START;

        int count = 0;

        while (count != bytesWritten) {
            int bytesReceived = readByteSerialPort(&byte);

            if (bytesReceived == 1) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_Tx) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == C_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_Tx ^ C_DISC)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) state = STOP_STATE;
                        break;
                    case STOP_STATE:
                        break;
                    default:
                        state = START;
                        break;    
                }
            }

            if (state == STOP_STATE) {
                printf("DISC frame received from transmitter\n");

                bytesWritten = writeBytesSerialPort(discFrame, 5);
                if (bytesWritten != 5) {
                    printf("Error writing DISC frame\n");
                    return -1;
                }
                
                break;
            }

            count++;
        }

        if (state == STOP_STATE) {
            state = START;

            while (state != STOP_STATE) {
                
                int bytesReceived = readByteSerialPort(&byte);

                if (bytesReceived == 1) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == A_Tx) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == C_UA) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (A_Tx ^ C_UA)) state = BCC_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC_OK:
                            if (byte == FLAG) state = STOP_STATE;
                            break;
                        case STOP_STATE:
                            break;
                        default:
                            state = START;
                            break;
                    }
                }


            }

            if (state == STOP_STATE) {
                printf("UA frame received from transmitter\n");
            }

            if (state != STOP_STATE) {
                printf("Failed to receive UA frame from transmissor\n");
                return -1;
            }
        }

        if (state != STOP_STATE) {
            printf("Receiver failed - Too many attempts\n");
            closeSerialPort();
            return -1;
        }
    }

    if (showStatistics) {
        printf("Statistics:\n");
        printf("Serial port: %s\n", info.serialPort);
        
        if (info.role == LlTx) {
            printf("Role: Transmitter\n");
        } else {
            printf("Role: Receiver\n");
        }
        
        printf("Max. number of tries: %d\n", info.nRetransmissions);
        printf("Alarm timeout: %d seconds\n", info.timeout);
    }

    int check = closeSerialPort();
    return check;
}
