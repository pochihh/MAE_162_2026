/*
 * Developed by Toby Chen, con mucho amor <3
 * Author Email: pc.toby.chen@gmail.com
 * Date: May 1, 2025
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include "tlvcodec.h"

uint8_t FRAME_HEADER_MAGIC_NUM[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
size_t MAX_TLVS = 16; // max number of TLVs in a frame

// encoder function definitions
void initEncodeDescriptor(struct TlvEncodeDescriptor *descriptor, size_t bufferSize, uint32_t deviceId, bool crc)
{
    descriptor->buffer = (uint8_t *)malloc(bufferSize);
    if (descriptor->buffer == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for buffer\n");
        exit(EXIT_FAILURE);
    }
    descriptor->bufferIndex = sizeof(struct FrameHeader);
    descriptor->bufferSize = bufferSize;

    // init magic num
    for (int i = 0; i < sizeof(FRAME_HEADER_MAGIC_NUM); i++)
    {
        descriptor->frameHeader.magicNum[i] = FRAME_HEADER_MAGIC_NUM[i];
    }
    descriptor->frameHeader.deviceId = deviceId;
    descriptor->crc = crc;
    descriptor->frameHeader.numTotalBytes.value = 0;
    descriptor->frameHeader.frameNum = 0;
    descriptor->frameHeader.numTlvs = 0;
}

void releaseEncodeDescriptor(struct TlvEncodeDescriptor *descriptor)
{
    if (descriptor->buffer != NULL)
    {
        free(descriptor->buffer);
        descriptor->buffer = NULL;
    }
}

void addTlvPacket(struct TlvEncodeDescriptor *descriptor, uint32_t tlvType, uint32_t tlvLen, const void *dataAddr)
{
    if (descriptor->bufferIndex + sizeof(struct TlvHeader) + tlvLen > descriptor->bufferSize)
    {
        fprintf(stderr, "Buffer overflow\n");
        exit(EXIT_FAILURE);
    }
    descriptor->tlvHeader.tlvType = tlvType;
    descriptor->tlvHeader.tlvLen = tlvLen;
    memcpy(descriptor->buffer + descriptor->bufferIndex, &descriptor->tlvHeader, sizeof(struct TlvHeader));
    descriptor->bufferIndex += sizeof(struct TlvHeader);
    memcpy(descriptor->buffer + descriptor->bufferIndex, dataAddr, tlvLen);
    descriptor->bufferIndex += tlvLen;
    descriptor->frameHeader.numTlvs++;
}

int wrapupBuffer(struct TlvEncodeDescriptor *descriptor)
{
    // add padding bytes to the size of MSG_BUFFER_SEGMENT_LEN
    size_t padding = descriptor->bufferIndex % MSG_BUFFER_SEGMENT_LEN;
    if (padding != 0)
    {
        padding = MSG_BUFFER_SEGMENT_LEN - padding;
        memset(descriptor->buffer + descriptor->bufferIndex, 0, padding);
        descriptor->bufferIndex += padding;
    }

    // Update total bytes
    descriptor->frameHeader.numTotalBytes.value = descriptor->bufferIndex;

    // copy the frame header to the beginning of the buffer
    memcpy(descriptor->buffer, &descriptor->frameHeader, sizeof(struct FrameHeader));

    // Calculate crc here if enabled
    if (descriptor->crc)
    {
        // Calculate CRC32 and store in descriptor->frameHeader.checksum
        descriptor->frameHeader.checksum = CRC32(descriptor->buffer + CRC32_BYTES2IGNORE, descriptor->bufferIndex - CRC32_BYTES2IGNORE);
    }
    else
    {
        descriptor->frameHeader.checksum = 0;
    }

    // copy crc result to the buffer
    memcpy(descriptor->buffer + 12, &descriptor->frameHeader.checksum, sizeof(uint32_t));

    // return the total bytes of the frame
    return (int)(descriptor->frameHeader.numTotalBytes.value); // return the total bytes of the frame
}

void resetDescriptor(struct TlvEncodeDescriptor *descriptor)
{
    descriptor->bufferIndex = sizeof(struct FrameHeader);
    descriptor->frameHeader.frameNum++;
    descriptor->frameHeader.numTlvs = 0;
}

// decoder function definitions
void initDecodeDescriptor(struct TlvDecodeDescriptor *descriptor, size_t bufferSize, bool crc, void (*callback)(enum DecodeErrorCode *error, const struct FrameHeader *frameHeader, struct TlvHeader *tlvHeaders, uint8_t **tlvData))
{
    // allocate the buffer
    descriptor->buffer = (uint8_t *)malloc(bufferSize);
    if (descriptor->buffer == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for buffer\n");
        exit(EXIT_FAILURE);
    }
    // allocate the tlv header and data
    descriptor->tlvHeaders = (struct TlvHeader *)malloc(sizeof(struct TlvHeader) * MAX_TLVS);
    if (descriptor->tlvHeaders == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for tlv headers\n");
        free(descriptor->buffer);
        exit(EXIT_FAILURE);
    }
    descriptor->tlvData = (uint8_t **)malloc(sizeof(uint8_t *) * MAX_TLVS);
    if (descriptor->tlvData == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for tlv data\n");
        free(descriptor->buffer);
        free(descriptor->tlvHeaders);
        exit(EXIT_FAILURE);
    }

    // initialize the descriptor
    descriptor->bufferSize = bufferSize;
    descriptor->bufferIndex = 0;
    descriptor->crc = crc;
    descriptor->ofst = 0;
    descriptor->decodeState = Init;
    descriptor->errorCode = NoError;
    descriptor->callback = callback;
}

void releaseDecodeDescriptor(struct TlvDecodeDescriptor *descriptor)
{
    if (descriptor->buffer != NULL)
    {
        free(descriptor->buffer);
        descriptor->buffer = NULL;
    }
}

void decode(struct TlvDecodeDescriptor *descriptor, const uint8_t *data, size_t dataLen)
{
    // iterate the data to decode and send them to decodePacket() one by one
    for (size_t i = 0; i < dataLen; i++)
    {
        decodePacket(descriptor, &data[i]);
    }
}

void decodePacket(struct TlvDecodeDescriptor *descriptor, const uint8_t *data)
{
    switch (descriptor->decodeState)
    {
    case Init:
        // check if the byte is the first byte of the magic number
        if (*data == FRAME_HEADER_MAGIC_NUM[0])
        {
            descriptor->bufferIndex = 0;
            descriptor->buffer[descriptor->bufferIndex++] = *data;
            descriptor->ofst = 1;
            descriptor->decodeState = MagicNum;
        }
        break;
    case MagicNum:
        // waiting for the full magic number sets
        if (*data == FRAME_HEADER_MAGIC_NUM[descriptor->ofst++])
        {
            descriptor->buffer[descriptor->bufferIndex++] = *data;
            if (descriptor->ofst >= sizeof(FRAME_HEADER_MAGIC_NUM))
            {
                descriptor->ofst = 0;
                descriptor->decodeState = TotalPacketLen;
            }
        }
        else
        {
            resetDecodeDescriptor(descriptor);
        }
        break;
    case TotalPacketLen:
        // Waiting for the total packet length
        descriptor->frameHeader.numTotalBytes.payload[descriptor->ofst++] = *data; // extracting the total packet length for later use
        descriptor->buffer[descriptor->bufferIndex++] = *data;                     // the data should still be added to the buffer

        if (descriptor->ofst >= sizeof(descriptor->frameHeader.numTotalBytes)) // 4 bytes
        {
            // check if the total packet length is valid
            if (descriptor->frameHeader.numTotalBytes.value > descriptor->bufferSize || descriptor->frameHeader.numTotalBytes.value < descriptor->bufferIndex)
            {
                // send result with error
                descriptor->errorCode = TotalPacketLenError;
                if (descriptor->callback != NULL)
                {
                    descriptor->callback(&descriptor->errorCode, &descriptor->frameHeader, NULL, NULL);
                }
                resetDecodeDescriptor(descriptor); // also changes the state to init
            }
            else if (descriptor->frameHeader.numTotalBytes.value == descriptor->bufferIndex)
            {
                // this is a TLV packet without payload
                parseFrame(descriptor);
                resetDecodeDescriptor(descriptor); // also changes the state to init
            }
            else
            {
                descriptor->decodeState = WaitFullFrame;
            }
        }
        break;
    case WaitFullFrame:
        descriptor->buffer[descriptor->bufferIndex++] = *data;
        if (descriptor->bufferIndex >= descriptor->frameHeader.numTotalBytes.value)
        {
            descriptor->ofst = 0;
            parseFrame(descriptor);
            resetDecodeDescriptor(descriptor); // also changes the state to init
        }
        break;
    default:
        resetDecodeDescriptor(descriptor);
        break;
    }
}

void parseFrame(struct TlvDecodeDescriptor *descriptor)
{
    descriptor->frameHeader = *(struct FrameHeader *)(descriptor->buffer); // parse all info for a single frame header
    descriptor->ofst = sizeof(struct FrameHeader);                         // offset used for the whole frame

    // Check CRC32 error if enabled
    if (descriptor->crc)
    {
        uint32_t validCrc32 = CRC32(descriptor->buffer + CRC32_BYTES2IGNORE, descriptor->frameHeader.numTotalBytes.value - CRC32_BYTES2IGNORE);
        if (descriptor->frameHeader.checksum != validCrc32)
        {
            descriptor->errorCode = CrcError;
            if (descriptor->callback != NULL)
            {
                descriptor->callback(&descriptor->errorCode, &descriptor->frameHeader, NULL, NULL);
            }
            return;
        }
    }

    // parse TLVs and return the results all together
    for (size_t i = 0; i < descriptor->frameHeader.numTlvs; ++i)
    {
        // TL header (Type, Length)
        descriptor->tlvHeaders[i] = *(struct TlvHeader *)(descriptor->buffer + descriptor->ofst);
        descriptor->ofst += sizeof(struct TlvHeader);

        // check if the tlv length is valid
        if (descriptor->tlvHeaders[i].tlvLen > descriptor->frameHeader.numTotalBytes.value - descriptor->ofst)
        {
            descriptor->errorCode = TlvLenError;
            if (descriptor->callback != NULL)
            {
                descriptor->callback(&descriptor->errorCode, &descriptor->frameHeader, NULL, NULL);
            }
            return;
        }else{
            // valid tlv length; add data pointer for output
            descriptor->tlvData[i] = descriptor->buffer + descriptor->ofst;
            descriptor->ofst += descriptor->tlvHeaders[i].tlvLen;
        }
    }

    // all the tlv headers and data are valid; send result
    if (descriptor->callback != NULL)
    {
        descriptor->callback(&descriptor->errorCode, &descriptor->frameHeader, descriptor->tlvHeaders, descriptor->tlvData);
    }

    return;
}

void resetDecodeDescriptor(struct TlvDecodeDescriptor *descriptor)
{
    descriptor->decodeState = Init;
    descriptor->bufferIndex = 0;
    descriptor->ofst = 0;
    descriptor->errorCode = NoError;
}

// CRC32 function
uint32_t CRC32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF; // initial value

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i]; // reflect input byte (handled naturally by the XOR)
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320; // reversed polynomial of 0x04C11DB7
            else
                crc = crc >> 1;
        }
    }

    return crc ^ 0xFFFFFFFF; // final XOR
}