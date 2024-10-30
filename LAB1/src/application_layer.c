// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdlib.h>
#include <string.h>

#include <stdio.h>

#define CTRL_START 1
#define CTRL_END   3

#define FILE_NAME  1
#define FILE_SIZE  0

int createControlPacket(unsigned char *packet, int packetType, const char *fileName, int fileSize) {
    int n = 0;

    packet[n++] = packetType;

    packet[n++] = FILE_NAME;
    int fileNameSize = strlen(fileName);
    packet[n++] = fileNameSize;
    memcpy(&packet[n], fileName, fileNameSize);
    n += fileNameSize;

    packet[n++] = FILE_SIZE;
    packet[n++] = sizeof(int);
    memcpy(&packet[n], &fileSize, sizeof(int));
    n += sizeof(int);

    return n;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {

    LinkLayer info;

    strcpy(info.serialPort, serialPort);
    info.baudRate = baudRate;
    info.nRetransmissions = nTries;
    info.timeout = timeout;

    if (!strcmp(role, "tx")) {
        info.role = LlTx;
    } else {
        info.role = LlRx;
    }

    if (llopen(info) < 0) {
        printf("Failed to open connection\n");
        return;
    }

    int fileSize;
    
    if (info.role == LlTx) {

        FILE *file = fopen(filename, "rb");

        if (!file) {
            printf("Failed to open file for reading\n");
            llclose(1);
            return;
        }

        fseek(file, 0, SEEK_END);
        fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);

        unsigned char controlPacket[MAX_PAYLOAD_SIZE];
        
        int controlPacketSize;
        controlPacketSize = createControlPacket(controlPacket, CTRL_START, filename, fileSize);

        if (llwrite(controlPacket, controlPacketSize) < 0) {
            printf("Failed to send START control packet\n");
            fclose(file);
            llclose(1);
            return;
        }

        unsigned char fileB[MAX_PAYLOAD_SIZE];
        int bytesWritten;

        while ((bytesWritten = fread(fileB, 1, MAX_PAYLOAD_SIZE, file)) > 0) {

            if (llwrite(fileB, bytesWritten) < 0) {
                printf("Failed to send data packet\n");
                fclose(file);
                llclose(1);
                return;
            }
        }

        controlPacketSize = createControlPacket(controlPacket, CTRL_END, filename, fileSize);
        
        if (llwrite(controlPacket, controlPacketSize) < 0) {
            printf("Failed to send END control packet\n");
            fclose(file);
            llclose(1);
            return;
        }

        printf("File transmission successful\n");
        fclose(file);
        return;

    } else if (info.role == LlRx) {
        
        FILE *file = fopen(filename, "wb");
        
        if (!file) {
            printf("Failed to open file for writing\n");
            llclose(1);
            return;
        }

        unsigned char fileB[MAX_PAYLOAD_SIZE];
        int bytesRead;
        int bytesReg = 0;
        int temp;

        bytesRead = llread(fileB);

        if (bytesRead <= 0 || fileB[0] != CTRL_START) {
            printf("Failed to receive START control packet\n");
            fclose(file);
            llclose(1);
            return;
        }
        
        while (bytesReg < fileSize) {

            if ((bytesRead = llread(fileB)) > 0) {

                if (fileB[0] == CTRL_END) {
                    break;
                }

                temp = fwrite(fileB, 1, bytesRead, file);

                bytesReg += temp;

                if (temp != bytesRead) {
                    printf("Error writing to file\n");
                    fclose(file);
                    llclose(1);
                    return;
                }
            }
        }

        printf("File reception successful\n");
        fclose(file);
    }

    if (llclose(1) < 0) {
        printf("Error closing link layer connection\n");
    }
}
