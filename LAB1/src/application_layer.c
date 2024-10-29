// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdlib.h>
#include <string.h>

#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {

    LinkLayer info;

    strcpy(info.serialPort, serialPort);
    info.baudRate = baudRate;
    info.nRetransmissions = nTries;
    info.timeout = timeout;
    if (role == "tx") {
        info.role = LlTx;
    } else {
        info.role = LlRx;
    }

    if (llopen(info) < 0) {
        perror("Failed to open connection");
        return;
    }
    
    if (role == LlTx) {

        FILE *file = fopen(filename, "rb");

        if (!file) {
            perror("Failed to open file for reading");
            llclose(1);
            return;
        }

        unsigned char fileB[MAX_PAYLOAD_SIZE];
        int bytesWritten;

        bytesWritten = fread(fileB, 1, MAX_PAYLOAD_SIZE, file);

        while (bytesWritten > 0) {
            if (llwrite(fileB, bytesRead) < 0) {
                perror("Failed to send data");
                fclose(file);
                llclose(1);
                return;
            }
        }

        printf("File transmission successful\n");
        fclose(file);

    } else if (connectionParameters.role == LlRx) {
        
        FILE *file = fopen(filename, "wb");
        
        if (!file) {
            perror("Failed to open file for writing");
            llclose(1);
            return;
        }

        unsigned char fileB[MAX_PAYLOAD_SIZE];
        int bytesRead;

        bytesRead = llread(fileB);

        while (bytesRead > 0) {
            if (fwrite(fileB, 1, bytesRead, file) != bytesRead) {
                perror("Error writing to file");
                fclose(file);
                llclose(1);
                return;
            }
        }

        printf("File reception successful\n");
        fclose(file);
    }

    if (llclose(1) < 0) {
        perror("Error closing link layer connection");
    }
}
