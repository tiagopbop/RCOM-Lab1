// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

#define TYPE_START 0x01
#define TYPE_END 0x03
#define TYPE_DATA 0x02
#define FILE_SIZE 0x00

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


    if (info.role == LlTx) {

        FILE *file = fopen(filename, "rb");

        if (!file) {
            printf("Failed to open file for reading\n");
            llclose(1);
            return;
        }

        fseek(file, 0, SEEK_END);
        size_t f_size = ftell(file);
        fseek(file, 0, SEEK_SET);

        unsigned char CTRLpacket_START[MAX_PAYLOAD_SIZE] = {0};

        CTRLpacket_START[0] = TYPE_START;
        CTRLpacket_START[1] = FILE_SIZE;
        CTRLpacket_START[2] = 4;

        CTRLpacket_START[3] = f_size >> 24;
        CTRLpacket_START[4] = (f_size >> 16) & 0xFF;
        CTRLpacket_START[5] = (f_size >> 8) & 0xFF;
        CTRLpacket_START[6] = f_size & 0xFF;

        printf("Sending START control packet...\n");

        if ((llwrite(CTRLpacket_START, 7) < 0)) {
            printf("Error sending control packet START\n");
            fclose(file);
            exit(-1);
        } else {
            printf("Control packet START sent successfully!\n");
        }

        int packetNum = 0;
        int bytesReadFromFile = 0;
        unsigned char temp[MAX_PAYLOAD_SIZE] = {0};

        while ((bytesReadFromFile = fread(temp, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            printf("\nCurrent packet's number: %d\n", packetNum);

            unsigned char DATApacket[MAX_PAYLOAD_SIZE + 4];

            DATApacket[0] = TYPE_DATA;
            DATApacket[1] = packetNum;
            DATApacket[2] = (bytesReadFromFile >> 8) & 0xFF;
            DATApacket[3] = bytesReadFromFile & 0xFF;

            for (int i = 0; i < bytesReadFromFile; i++) {
                DATApacket[4 + i] = temp[i];
            }

            if (llwrite(DATApacket, bytesReadFromFile + 4) < 0) {
                printf("Error sending data packet\n");
                fclose(file);
                exit(-1);
            }

            packetNum++;
        }

        unsigned char CTRLpacket_END[MAX_PAYLOAD_SIZE] = {0};

        CTRLpacket_END[0] = TYPE_END;
        CTRLpacket_END[1] = FILE_SIZE;
        CTRLpacket_END[2] = 4;

        CTRLpacket_END[3] = f_size >> 24;
        CTRLpacket_END[4] = (f_size >> 16) & 0xFF;
        CTRLpacket_END[5] = (f_size >> 8) & 0xFF;
        CTRLpacket_END[6] = f_size & 0xFF;

        printf("\nSending END control packet...\n");

        if ((llwrite(CTRLpacket_END, 7) < 0)) {
            printf("Error sending control packet END\n");
            fclose(file);
            exit(-1);
        } else {
            printf("\nControl packet END sent successfully!\n");
        }

        printf("\nFile transmission successful!\n");
        fclose(file);

    } else if (info.role == LlRx) {

        FILE *file = fopen(filename, "wb");

        if (!file) {
            printf("Failed to open file for writing\n");
            llclose(1);
            return;
        }

        size_t f_size = 0;

        unsigned char CTRLpacket_START[MAX_PAYLOAD_SIZE] = {0};

        llread(CTRLpacket_START);

        if (CTRLpacket_START[0] != TYPE_START || CTRLpacket_START[1] != FILE_SIZE || CTRLpacket_START[2] != 4) {
            printf("Error receiving the first bytes of control packet START\n");
            fclose(file);
            exit(-1);
        }

        f_size = (CTRLpacket_START[3] << 24) | (CTRLpacket_START[4] << 16) | (CTRLpacket_START[5] << 8) | CTRLpacket_START[6];
        printf("\nControl packet START received successfully!\n");

        int packetNum = 0;
        int bytesWrittenIntoNewFile = 0;
        unsigned char temp[MAX_PAYLOAD_SIZE] = {0};

        while (bytesWrittenIntoNewFile < f_size) {
            printf("\nCurrent packet's number: %d", packetNum);

            unsigned char DATApacket[MAX_PAYLOAD_SIZE + 4] = {0};

            int bytesReceived = llread(DATApacket);

            if (DATApacket[0] != TYPE_DATA || DATApacket[1] != packetNum) {
                printf("Error receiving data packet\n");
                fclose(file);
                exit(-1);
            }

            if (bytesReceived > 0) {
                int p_size = DATApacket[2] * 256 + DATApacket[3];

                for (int i = 0; i < p_size; i++) {
                    temp[i] = DATApacket[4 + i];
                }
                bytesWrittenIntoNewFile += fwrite(temp, 1, p_size, file);

                packetNum++;
            }
        }

        unsigned char CTRLpacket_END[MAX_PAYLOAD_SIZE] = {0};

        llread(CTRLpacket_END);

        if (CTRLpacket_END[0] != TYPE_END || CTRLpacket_END[1] != FILE_SIZE || CTRLpacket_END[2] != 4) {
            printf("Error receiving the first bytes of control packet END\n");
            fclose(file);
            exit(-1);
        }

        f_size = (CTRLpacket_END[3] << 24) | (CTRLpacket_END[4] << 16) | (CTRLpacket_END[5] << 8) | CTRLpacket_END[6];
        printf("\nControl packet END received successfully!\n\n");

        printf("File reception successful!\n");
        fclose(file);
    }


    if (llclose(1) < 0) {
        printf("Error closing currently open connection\n");
    } else {
        printf("\nConnection closed successfully!\n\n");
    }
}
