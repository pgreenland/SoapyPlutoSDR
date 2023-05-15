#pragma once

#include <cstddef>

class tx_streamer {
	public:
		virtual ~tx_streamer() = default;

		virtual int send(const void * const *buffs,
						 const size_t numElems,
						 int &flags,
						 const long long timeNs,
						 const long timeoutUs) = 0;

		virtual int flush(const long timeoutUs) = 0;

		virtual int start(const int flags,
						  const long long timeNs,
						  const size_t numElems) = 0;

		virtual int stop(const int flags,
						 const long long timeNs) = 0;

		virtual size_t get_mtu_size() = 0;

		virtual void set_samplerate(const size_t _samplerate) = 0;
};
