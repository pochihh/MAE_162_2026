/*
 * Developed by Toby Chen, con mucho amor <3
 * Author Email: pc.toby.chen@gmail.com
 * Date: May 1, 2025
 * License: MIT
 */

#ifndef TLV_CODEC_H
#define TLV_CODEC_H
#include <stdbool.h>

// define TLVCodec structs

struct FrameHeader
{
	uint8_t magicNum[8];
	union
	{
		uint32_t value;
		uint8_t payload[4];
	} numTotalBytes;
	uint32_t checksum;
	uint32_t deviceId;
	uint32_t frameNum;
	uint32_t numTlvs;
};

struct TlvHeader
{
	uint32_t tlvType;
	uint32_t tlvLen;
};

// TLV encoder
struct TlvEncodeDescriptor
{
	uint8_t *buffer;
	size_t bufferSize;
	size_t bufferIndex;
	bool crc; // true if CRC is enabled
	struct FrameHeader frameHeader;
	struct TlvHeader tlvHeader;
};

// TLV encoder functions declaration
void initEncodeDescriptor(struct TlvEncodeDescriptor *descriptor, size_t bufferSize, uint32_t deviceId, bool crc);

void releaseEncodeDescriptor(struct TlvEncodeDescriptor *descriptor);

void addTlvPacket(struct TlvEncodeDescriptor *descriptor, uint32_t tlvType, uint32_t tlvLen, const void *dataAddr);

int wrapupBuffer(struct TlvEncodeDescriptor *descriptor);

void resetDescriptor(struct TlvEncodeDescriptor *descriptor);

// TLV decoder
enum FrameDecodeState
{
	Init = 0,
	MagicNum = 1,
	TotalPacketLen = 2,
	WaitFullFrame = 3
};

enum DecodeErrorCode
{
	NoError = 0,
	CrcError = 1,
	TotalPacketLenError = 2,
	BufferOutOfIndex = 3,
	UnpackFrameHeaderError = 4,
	TlvError = 5,
	TlvLenError = 6
};

struct TlvDecodeDescriptor
{
	uint8_t *buffer;
	size_t bufferSize;
	size_t bufferIndex;
	bool crc; // true if CRC is enabled
	enum FrameDecodeState decodeState;
	enum DecodeErrorCode errorCode;
	size_t ofst;
	struct FrameHeader frameHeader;

	// add a function handle to store a callback function
	void (*callback)(enum DecodeErrorCode *error, const struct FrameHeader *frameHeader, struct TlvHeader *tlvHeaders, uint8_t **tlvData);

	// return list of pointers for the tlv results
	struct TlvHeader *tlvHeaders;
	uint8_t **tlvData;
};

void initDecodeDescriptor(struct TlvDecodeDescriptor *descriptor, size_t bufferSize, bool crc, void (*callback)(enum DecodeErrorCode *error, const struct FrameHeader *frameHeader, struct TlvHeader *tlvHeaders, uint8_t **tlvData));

void releaseDecodeDescriptor(struct TlvDecodeDescriptor *descriptor);

void decode(struct TlvDecodeDescriptor *descriptor, const uint8_t *data, size_t dataLen);

void decodePacket(struct TlvDecodeDescriptor *descriptor, const uint8_t *data);

void parseFrame(struct TlvDecodeDescriptor *descriptor);

void resetDecodeDescriptor(struct TlvDecodeDescriptor *descriptor);

// utility functions
// magic num
extern uint8_t FRAME_HEADER_MAGIC_NUM[8];
extern size_t MAX_TLVS; // max number of TLVs in a frame
#define MSG_BUFFER_SEGMENT_LEN 8 // length of the message buffer segment

// CRC32
#define CRC32_BYTES2IGNORE 16 // first 16 bytes of header
uint32_t CRC32(const uint8_t *data, size_t length);

#endif // TLV_CODEC_H