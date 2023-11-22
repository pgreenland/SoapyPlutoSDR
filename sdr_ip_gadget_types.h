#pragma once

#include <cstdint>

/* Definitions - packet magic number */
#define SDR_IP_GADGET_MAGIC (0x4F544C50)

/* Commands */
#define SDR_IP_GADGET_COMMAND_START_TX (0x00)
#define SDR_IP_GADGET_COMMAND_START_RX (0x01)
#define SDR_IP_GADGET_COMMAND_STOP_TX (0x02)
#define SDR_IP_GADGET_COMMAND_STOP_RX (0x03)

/* Type definitions */
#pragma pack(push,1)
typedef struct
{
	/* Magic word, most basic protection against stray packets */
	uint32_t magic;

	/* Command */
	uint32_t cmd;

} cmd_ip_header_t;

typedef struct
{
	/* Command header */
	cmd_ip_header_t hdr;

	/* Bitmask of enabled channels */
	uint32_t enabled_channels;

	/*
	** Timestamping enabled
	** Each tx packet has a timestamp / sequence number. If this flag is set this sequence
	** is taken from the first UDP packet to be written to the IIO buffer. Any subsequent packets
	** used to fill the buffer have their sequence number checked against the expected.
	** If this flag is cleared a local sequence number variable is used and packets are checked to
	** monitor their order, but are otherwise provided directly to the IIO library.
	*/
	bool timestamping_enabled;

	/*
	** Buffer size (in samples) to request from IIO library
	** Note: This should include space for the 64-bit timestamp.
	** For example with RX0's I and Q channels enabled, each sample will be 2 * 16bit = 32bit
	** therefore a timestamp will occupy 64bit / 32bit = 2 samples. If a timestamp were to be provided
	** at the start of each buffer's worth of samples, an additional two samples would need to be added to
	** the buffer space.
	** Likewise if RX0 and RX1's I and Q channels were enabled, each sample will be 4 * 16bit = 64bit
	** as such only one sample would be required for the timestamp.
	*/
	uint32_t buffer_size;

} cmd_ip_tx_start_req_t;

typedef struct
{
	/* Command header */
	cmd_ip_header_t hdr;

	/* RX host data port (will use client's IP but need to know the selected ephemeral port) */
	uint16_t data_port;

	/* Bitmask of enabled channels */
	uint32_t enabled_channels;

	/*
	** Timestamping enabled
	** Each rx packet has a timestamp / sequence number. If this flag is set this sequence
	** number is taken from the start of the IIO buffer and incremented for each UDP packet generated.
	** If this flag is cleared a local sequence number variable is used and incremented for each sample
	*/
	bool timestamping_enabled;

	/*
	** Buffer size (in samples) to request from IIO library
	** Note: This should include space for the 64-bit timestamp.
	** For example with RX0's I and Q channels enabled, each sample will be 2 * 16bit = 32bit
	** therefore a timestamp will occupy 64bit / 32bit = 2 samples. If a timestamp were to be provided
	** at the start of each buffer's worth of samples, an additional two samples would need to be added to
	** the buffer space.
	** Likewise if RX0 and RX1's I and Q channels were enabled, each sample will be 4 * 16bit = 64bit
	** as such only one sample would be required for the timestamp.
	*/
	uint32_t buffer_size;

	/* UDP packet size (bytes) */
	/*
	** Typically 1472 (1500 byte ethernet payload - 20 byte IP header - 8 byte UDP header)
	** Could be enlarged to 8972 (9000 byte ethernet payload - 20 byte IP header - 8 byte UDP header) if using jumbo frames
	*/
	uint16_t packet_size;

} cmd_ip_rx_start_req_t;

typedef struct
{
	/* Command header */
	cmd_ip_header_t hdr;

	/* No arguments */

} cmd_ip_stop_req_t;

typedef union
{
	cmd_ip_header_t hdr;
	cmd_ip_tx_start_req_t start_tx;
	cmd_ip_rx_start_req_t start_rx;
	cmd_ip_stop_req_t stop;

} cmd_ip_t;

typedef struct
{
	/* Magic word, most basic protection against stray packets */
	uint32_t magic;

	/* Block index / count */
	uint8_t block_index;
	uint8_t block_count;

	/* Spare */
	uint16_t unused;

	/* Timestamp / sequence number */
	uint64_t seqno;

} data_ip_hdr_t;
#pragma pack(pop)
