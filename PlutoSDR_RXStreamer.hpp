#pragma once

#include <cstddef>

class rx_streamer {
	public:
		virtual ~rx_streamer() = default;

		virtual size_t recv(void * const *buffs,
							const size_t numElems,
							int &flags,
							long long &timeNs,
							const long timeoutUs=100000) = 0;

		virtual int start(const int flags,
						  const long long timeNs,
						  const size_t numElems) = 0;

		virtual int stop(const int flags,
						 const long long timeNs) = 0;

		virtual void set_buffer_size_by_samplerate(const size_t _samplerate) = 0;

		virtual size_t get_mtu_size() = 0;
};
