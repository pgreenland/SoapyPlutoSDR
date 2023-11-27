#pragma once

/* Libraries */
#include <cstdint>
#include <vector>

typedef struct
{
	/* Data packet timestamp / sequence number */
	uint64_t seqno;

	/* Data packet payload */
	std::vector<uint8_t> payload;

} seq_payload_t;
