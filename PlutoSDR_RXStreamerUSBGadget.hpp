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
#include "PlutoSDR_RXStreamer.hpp"

#include "FIFO.hpp"

class rx_streamer_usb_gadget : public rx_streamer {
	public:
		rx_streamer_usb_gadget(const iio_device *dev, libusb_device_handle* usb_dev, uint8_t intfc_num, uint8_t ep_num, const plutosdrStreamFormat format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args, uint32_t timestamp_every);
		~rx_streamer_usb_gadget();

		size_t recv(void * const *buffs,
					const size_t numElems,
					int &flags,
					long long &timeNs,
					const long timeoutUs=100000);

		int start(const int flags,
				  const long long timeNs,
				  const size_t numElems);

		int stop(const int flags,
				 const long long timeNs=100000);

		void set_buffer_size_by_samplerate(const size_t _samplerate);

		size_t get_mtu_size();

	private:
		// IIO device
		const iio_device *dev;

		// USB gadget
		libusb_device_handle* usb_dev;
		uint8_t intfc_num, ep_num;

		// Sample format
		const plutosdrStreamFormat format;

		// How often to expect a timestamp in the stream
		uint32_t timestamp_every;

		// Read thread
		std::thread thread;

		// Stop flag
		std::atomic<bool> thread_stop;

		// Queue
		TSQueue<std::shared_ptr<std::vector<uint8_t>>> queue;

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

		// Fixed buffer size (user supplies or set due to timestamp)
		bool fixed_buffer_size;

		// Buffer size (samples)
		uint32_t buffer_size_samples;

		// Direct copy supported
		bool direct_copy;

		// Current buffer being processed by receive
		std::shared_ptr<std::vector<uint8_t>> curr_buffer;
		size_t curr_buffer_samples_remaining;
		size_t curr_buffer_offset;
		uint64_t curr_buffer_timestamp;

		// Update buffer size, restarting stream if required
		void set_buffer_size(const size_t _buffer_size);

		// Read thread - fetches data from USB device and places into fifo
		void thread_func(uint32_t curr_enabled_channels, uint32_t curr_buffer_size_samples);

		// Private start / stop functions
		void _start(void);
		void _stop(void);
};
