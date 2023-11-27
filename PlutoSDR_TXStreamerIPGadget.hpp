#pragma once

#include <vector>
#include <thread>
#include <atomic>

#include <iio.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <libusb.h>
#pragma GCC diagnostic pop

#include <SoapySDR/Types.hpp>

#include "PlutoSDR_StreamFormat.hpp"
#include "PlutoSDR_TXStreamer.hpp"

#include "FIFO.hpp"

#include "sdr_ip_gadget_private_types.h"

class tx_streamer_ip_gadget : public tx_streamer {
	public:
		tx_streamer_ip_gadget(const iio_device *dev, int sock_control, int sock_data, size_t udp_packet_size, const plutosdrStreamFormat format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args, uint32_t timestamp_every);
		~tx_streamer_ip_gadget();

		int send(const void * const *buffs,
				 const size_t numElems,
				 int &flags,
				 const long long timeNs,
				 const long timeoutUs);

		int flush(const long timeoutUs);

		int start(const int flags,
		const long long timeNs,
		const size_t numElems);

		int stop(const int flags,
		const long long timeNs);

		size_t get_mtu_size();

		void set_samplerate(const size_t _samplerate);

	private:
		// IIO device
		const iio_device *dev;

		// IP gadget
		int sock_control;
		int sock_data;
		size_t udp_packet_size;

		// Sample format
		const plutosdrStreamFormat format;

		// How often to expect a timestamp in the stream
		uint32_t timestamp_every;

		// Read thread
		std::thread thread;

		// Stop flag
		std::atomic<bool> thread_stop;

		// Queue
		TSQueue<std::shared_ptr<seq_payload_t>> queue;

		// IIO channel list
		std::vector<iio_channel*> channel_list;

		// Channel bitmask
		uint32_t enabled_channels;

		// Sample rate
		long long sample_rate;

		// Expected sample size (bytes)
		uint32_t sample_size_bytes;

		// Expected timestamp size (samples)
		uint32_t timestamp_size_samples;

		// Buffer size, excluding timestamp (samples)
		uint32_t buffer_size_samples;

		// Number of UDP packets required to transfer a buffer
		size_t packets_per_buffer;

		// Direct copy supported
		bool direct_copy;

		// Current buffer being processed by send
		std::shared_ptr<seq_payload_t> curr_buffer;
		size_t curr_buffer_samples_stored;
		uint64_t curr_buffer_timestamp;

		// Update buffer size, restarting stream if required
		void set_buffer_size(const size_t _buffer_size);

		// Write thread - fetches data from fifo and sends to USB device
		void thread_func(uint32_t curr_enabled_channels, uint32_t curr_buffer_size_samples);

		// Private start / stop functions
		void _start(void);
		void _stop(void);
};
