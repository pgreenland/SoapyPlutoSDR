#pragma once

#include <cstdint>

/* Definitions */
#define SDR_USB_GADGET_COMMAND_START (0x10)
#define SDR_USB_GADGET_COMMAND_STOP (0x11)
#define SDR_USB_GADGET_COMMAND_TARGET_RX (0x0)
#define SDR_USB_GADGET_COMMAND_TARGET_TX (0x1)

/* Type definitions */
#pragma pack(push,1)
typedef struct
{
	/* Bitmask of enabled channels */
	uint32_t enabled_channels;

	/*
	** Buffer size (in samples)
	** Note: This should include space for the 64-bit timestamp.
	** For example with RX0's I and Q channels enabled, each sample will be 2 * 16bit = 32bit
	** therefore a timestamp will occupy 64bit / 32bit = 2 samples. If a timestamp were to be provided
	** at the start of each buffer's worth of samples, an additional two samples would need to be added to
	** the buffer space.
	** Likewise if RX0 and RX1's I and Q channels were enabled, each sample will be 4 * 16bit = 64bit
	** as such only one sample would be required for the timestamp.
	*/
	uint32_t buffer_size;

} cmd_usb_start_request_t;
#pragma pack(pop)
