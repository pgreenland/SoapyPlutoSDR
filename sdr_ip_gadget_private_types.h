#pragma once

/* Libraries */
#include <vector>

/* Shared types */
#include "sdr_ip_gadget_types.h"

typedef struct
{
	/* Data packet header (containing timestamp / sequence number) */
	data_ip_hdr_t hdr;

	/* Data packet payload */
	std::vector<uint8_t> payload;

} hdr_payload_t;
