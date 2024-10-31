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
#define A_RECEIV        0x01
#define C_SET           0x03
#define C_UA            0x07
#define C_RR0           0xAA
#define C_RR1           0xAB
#define C_REJ0          0x54
#define C_REJ1          0x55
#define C_DISC          0x0B
#define C_I0            0x00
#define C_I1            0x80
#define ESCAPE          0x7D

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE,
    DATA,
    ESCAPE_STATE
} State;

// Variables used in the process
LinkLayer parameters;
State state = START;
volatile int waitAlarm = FALSE;
unsigned char control_flag = 0;
int currSeq = 0;
int alarmCount = 0;

// Statistics Variables
unsigned int totalFramesSent = 0;
unsigned int totalFramesReceived = 0;
unsigned int retransmissions = 0;
unsigned int totalDataBytes = 0;
struct timeval start_time, end_time;

unsigned char byte;


void alarmHandler(int signal)
{
    waitAlarm = FALSE;
    printf("Couldnt receive frame\n");
    alarmCount++;
}



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
// Create connection between Tx and Rx
int llopen(LinkLayer connectionParameters) {

    parameters = connectionParameters;

    State state = START;
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return -1;
    }

    int bytesWritten=0;

    if (connectionParameters.role == LlTx) {
        (void)signal(SIGALRM, alarmHandler);
        unsigned char message_test[5] = {FLAG, A_TRANS, C_SET, A_TRANS ^ C_SET, FLAG};

        alarmCount = 0;
        state = START;

        while (alarmCount < connectionParameters.nRetransmissions) {

            bytesWritten = writeBytesSerialPort(message_test, 5);

            if (bytesWritten == 5) {
                alarm(connectionParameters.timeout);
                waitAlarm = TRUE;
            } else {
                printf("Byte writing error\n");
                closeSerialPort();
                return -1;
            }


            while (waitAlarm && state != STOP_STATE) {

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
                        case STOP_STATE:
                            break;
                        default:
                            state = START;
                            break;
                    }
                }

            }

            if (state == STOP_STATE) {
                alarm(0);
                waitAlarm = FALSE;
                printf("Connection successfully tested and working\n");
                return 1;
            }
        }

        printf("Failed connection - Too many attempts");
        closeSerialPort();
        return -1;

    } else if (connectionParameters.role == LlRx) {

        int count = 0;

        while (state != STOP_STATE) {
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
                    case STOP_STATE:
                        break;
                    default:
                        state = START;
                        break;
                }
            }

            if (state == STOP_STATE) {
                unsigned char answer_test[5] = {FLAG, A_TRANS, C_UA, A_TRANS ^ C_UA, FLAG};

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
////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
// Creates the frame whose data is in the buf
// Starts by adding the header of the frame and then adds the buf's data into the frame using byte stuffing;
// After that, adds the BBC2 and FLAG to the final of the frame. Finally, sends the frame to the receiver and waits for the response
int llwrite(const unsigned char *buf, int bufSize)
{
    printf("Writting bytes...\n");

    alarmCount = 0;

    unsigned char iframe[bufSize*2], bcc2 = 0;
    int idx = 0;

    // Frame's header
    iframe[idx] = FLAG; idx++;
    iframe[idx] = A_TRANS; idx++;
    iframe[idx] = currSeq ? C_I1 : C_I0; idx++;
    iframe[idx] = A_TRANS ^ iframe[idx-1]; idx++;

    // Adds buf's data into the frame
    for (int i = 0; i < bufSize; i++) {
        unsigned char currByte = buf[i];

        switch (currByte) {
            case ESCAPE:
                iframe[idx] = ESCAPE; idx++;
                iframe[idx] = ESCAPE ^ 0x20; idx++;
                break;
            case FLAG:
                iframe[idx] = ESCAPE; idx++;
                iframe[idx] = FLAG ^ 0x20; idx++;
                break;
            default:
                iframe[idx] = currByte; idx++;
                break;
        }

        bcc2 ^= currByte;
    }

    iframe[idx] = bcc2;

    if (bcc2 == FLAG) {
        iframe[idx] = ESCAPE; idx++;
        iframe[idx] = FLAG ^ 0x20;
    } else if (bcc2 == ESCAPE) {
        iframe[idx] = ESCAPE; idx++;
        iframe[idx] = ESCAPE ^ 0x20;
    }
    idx++;

    iframe[idx] = FLAG;

    alarmCount = 0;
    int bytesSent = 0;
    waitAlarm = FALSE;
    state = START;
    unsigned char response = 0;

    // Sends frame and wait for answer. If response not received, resends the frame
    while (alarmCount < parameters.nRetransmissions) {

        if (!waitAlarm) {
            retransmissions += alarmCount > 1 ? 1 : 0;
            totalFramesSent++;
            bytesSent = writeBytesSerialPort(iframe, idx+1);;
            printf("Written bytes on frame: %d\n", bytesSent);

            waitAlarm = TRUE;
            alarm(parameters.timeout);
            state = START;
        }

        int bytesResponse = readByteSerialPort(&response);

        if (bytesResponse > 0) {
            switch (state) {
                case START:
                    if (response == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case FLAG_RCV:
                    if (response == A_TRANS) {
                        state = A_RCV;
                    } else if (response != FLAG) {
                        state = START;
                    }
                    break;
                case A_RCV:
                    if ((parameters.role == LlTx && (response == C_UA || response == C_RR0 || response == C_RR1 || response == C_REJ0 || response == C_REJ1)) ||
                        (parameters.role == LlRx && (response == C_SET || response == C_I0 || response == C_I1))) {
                        control_flag = response;
                        state = C_RCV;
                    } else if (response == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case C_RCV:
                    if (response == (A_TRANS ^ control_flag)) {
                        state = BCC_OK;
                    } else if (response == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if (response == FLAG) {
                        state = STOP_STATE;
                        printf("Frame received!\n");
                    }
                    break;
                case STOP_STATE:
                case DATA:
                case ESCAPE_STATE:
                default:
                    break;
            }
        }

        if (state == STOP_STATE && control_flag == (currSeq? C_RR0 : C_RR1)) {
            alarm(0);
            printf("Info frame received\n");
            currSeq = 1 - currSeq;
            return idx;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
// Using a state machine fills the packet with the important data (using byte destuffing)
// After that, checks if the frame's BBC2 matches with the calculation
// If BBC2 correct, sends an answer to the Tx. If not, rejects the frame.
int llread(unsigned char *packet)
{
    State state_read = START;
    unsigned char control_byte = 0, byte = 0;
    int idx = 0;

    // Reads one byte at a time. Adds data to the packet
    while (state_read != STOP_STATE) {
        int byteRead = readByteSerialPort(&byte);

        if (byteRead > 0) {
            switch (state_read) {
                case START:
                    if (byte == FLAG) {
                        state_read = FLAG_RCV;
                    } else {
                        state_read = START;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_TRANS) {
                        state_read = A_RCV;
                    } else if (byte != FLAG) {
                        state_read = START;
                    }
                    break;
                case A_RCV:
                    if (byte == C_I0 || byte == C_I1) {
                        control_byte = byte;
                        state_read = C_RCV;
                    } else if (byte == FLAG) {
                        state_read = FLAG_RCV;
                    } else {
                        state_read = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_TRANS ^ control_byte)) {
                        state_read = BCC_OK;
                    } else if (byte == FLAG) {
                        state_read = FLAG_RCV;
                    } else {
                        state_read = START;
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        state_read = STOP_STATE;
                    } else if (byte == ESCAPE) {
                        state_read = ESCAPE_STATE;
                    } else {
                        packet[idx] = byte; idx++;
                        state_read = DATA;
                    }
                    break;
                case DATA:
                    if (byte == ESCAPE) {
                        state_read = ESCAPE_STATE;
                    } else if (byte == FLAG) {
                        state_read = STOP_STATE;
                    } else {
                        packet[idx] = byte; idx++;
                    }
                    break;
                case ESCAPE_STATE:
                    if (byte == (FLAG ^ 0x20)) {
                        state_read = DATA;
                        packet[idx] = FLAG; idx++;
                    } else if (byte == (ESCAPE ^ 0x20)) {
                        state_read = DATA;
                        packet[idx] = ESCAPE; idx++;
                    } else {
                        state_read = START;
                    }
                    break;
                case STOP_STATE:
                    break;
                default:
                    break;
            }
        }
    }

    // Calculate BBC2
    unsigned char bcc2 = 0;
    for (int i = 0; i < idx-1; i++) {
        bcc2 ^= packet[i];
    }

    unsigned char control_response = 0;

    // Reject frame
    if (bcc2 != packet[idx-1]) {
        printf("Wrong bcc2!\n");

        control_response = currSeq ? C_REJ0 : C_REJ1;

        unsigned char response[5] = {FLAG, A_TRANS, control_response, A_TRANS ^ control_response, FLAG};
        writeBytesSerialPort(response, 5);;
        return -1;
    }

    // Send answer
    control_response = currSeq ? C_RR0 : C_RR1;
    unsigned char response[5] = {FLAG, A_TRANS, control_response, A_TRANS ^ control_response, FLAG};
    int writtenBytes = writeBytesSerialPort(response, 5);
    printf("Written bytes on response: %d\n", writtenBytes);
    totalFramesReceived++;
    totalDataBytes += idx;
    currSeq = 1 - currSeq;
    return idx;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
// Tx tries to send the DISC frame and receive another DISC
// If DISC was successful, Tx sends UA frame
int llclose(int showStatistics) {
    state = START;
    unsigned char byte;
    unsigned char discFrame[5] = {FLAG, A_RECEIV, C_DISC, A_RECEIV ^ C_DISC, FLAG};
    unsigned char uaFrame[5] = {FLAG, A_RECEIV, C_UA, A_RECEIV ^ C_UA, FLAG};

    (void)signal(SIGALRM, alarmHandler);
    alarmCount = 0;

    if (parameters.role == LlTx) {
        while (alarmCount < parameters.nRetransmissions) {
            int bytesWritten = writeBytesSerialPort(discFrame, 5);
            printf("Transmitter sent DISC frame bytes: %d\n", bytesWritten);

            if (bytesWritten != 5) {
                perror("Error writing DISC frame");
                return -1;
            }

            alarm(parameters.timeout);
            waitAlarm = TRUE;

            while (waitAlarm) {
                int bytesRead = readByteSerialPort(&byte);
                if (bytesRead > 0) {
                    printf("Transmitter received byte: %02X, State: %d\n", byte, state);

                    switch (state) {
                        case START:
                            if (byte == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;

                        case FLAG_RCV:
                            if (byte == A_RECEIV) {
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
                            if (byte == (A_RECEIV ^ C_DISC)) {
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
                                waitAlarm = FALSE;
                                alarm(0);
                            } else {
                                state = START;
                            }
                            break;

                        default:
                            state = START;
                            break;
                    }
                } else if (bytesRead < 0) {
                    perror("Error reading byte");
                    return -1;
                }
            }

            if (state == STOP_STATE) {
                break;
            }
        }

        if (state != STOP_STATE) {
            printf("Transmitter failed to receive DISC frame\n");
            return -1;
        }

        int bytesWritten = writeBytesSerialPort(uaFrame, 5);
        printf("Transmitter sent UA frame bytes: %d\n", bytesWritten);

        if (bytesWritten != 5) {
            perror("Error sending UA frame");
            return -1;
        }

        if (showStatistics) {
            gettimeofday(&end_time, NULL);
            double executionTime = (end_time.tv_sec - start_time.tv_sec) +
                                   (end_time.tv_usec - start_time.tv_usec) / 1000000.0;
            double FER = (double)(retransmissions) / (double)(totalFramesSent);
            printf("=== Transmitter Statistics ===\n");
            printf("Total Execution Time: %.2f seconds\n", executionTime);
            printf("Total Frames Sent: %u\n", totalFramesSent);
            printf("Total Retransmissions: %u\n", retransmissions);
            printf("Frame Error Rate (FER): %.2f\n", FER);
            printf("=============================\n");
        }

    } else if (parameters.role == LlRx) {
        while (alarmCount < parameters.nRetransmissions) {
            int bytesRead = readByteSerialPort(&byte);
            if (bytesRead > 0) {
                printf("Receiver received byte: %02X, State: %d\n", byte, state);

                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        }
                        break;

                    case FLAG_RCV:
                        if (byte == A_RECEIV) {
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
                        if (byte == (A_RECEIV ^ C_DISC)) {
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
                            waitAlarm = FALSE;
                        } else {
                            state = START;
                        }
                        break;

                    default:
                        state = START;
                        break;
                }
            }

            if (state == STOP_STATE) {
                break;
            }
        }

        if (state != STOP_STATE) {
            printf("Receiver failed to receive DISC frame\n");
            return -1;
        }

        int bytesWritten = writeBytesSerialPort(discFrame, 5);;
        printf("Receiver sent DISC frame bytes: %d\n", bytesWritten);

        if (bytesWritten != 5) {
            perror("Error sending DISC frame");
            return -1;
        }

        alarm(parameters.timeout);
        waitAlarm = TRUE;

        while (waitAlarm) {
            int bytesRead = readByteSerialPort(&byte);
            if (bytesRead > 0) {
                printf("Receiver received byte: %02X\n", byte);
                if (byte == FLAG) {
                    state = STOP_STATE;
                    waitAlarm = FALSE;
                }
            }
        }

        if (showStatistics) {
            unsigned int totalBitsTransferred = totalDataBytes * 8;
            double bitrate = parameters.baudRate;
            double efficiency = (double)totalBitsTransferred / bitrate;
            printf("=== Receiver Statistics ===\n");
            printf("Total Frames Received: %u\n", totalFramesReceived);
            printf("Total Data Transferred: %u bytes\n", totalDataBytes);
            printf("Efficiency (S): %.2f\n", efficiency);
            printf("Total Data Received: %u bytes\n", totalDataBytes);
            printf("============================\n");
        }
    }
    int clstat = closeSerialPort();
    return clstat;
}
