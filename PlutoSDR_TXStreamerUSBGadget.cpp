#include <cstring>

#include <sched.h>
#include <pthread.h>

#include "PlutoSDR_TXStreamerUSBGadget.hpp"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

#include "PlutoSDR_TimestampEvery.hpp"

#include "sdr_usb_gadget_types.h"

tx_streamer_usb_gadget::tx_streamer_usb_gadget(const iio_device *_dev, libusb_device_handle* _usb_dev, uint8_t _intfc_num, uint8_t _ep_num, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args, uint32_t _timestamp_every):
	dev(_dev), usb_dev(_usb_dev), intfc_num(_intfc_num), ep_num(_ep_num), format(_format), timestamp_every(_timestamp_every), thread_stop(false), queue(16, false), curr_buffer_timestamp(0)

{
	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	// This doesn't look right in general....but works for the purposes of enabling a single channel here
	enabled_channels = 0;
	for (unsigned int i = 0; i < channelIDs.size() * 2; i++) {
		struct iio_channel *chn = iio_device_get_channel(dev, i);

		// Store channel for sample conversions later
		channel_list.push_back(chn);

		// Add bit to enabled channels
		enabled_channels |= (1 << i);
	}

	// Retrieve sample rate
	iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"sampling_frequency", &sample_rate);

	// Calculate expected sample size
	sample_size_bytes = channel_list.size() * sizeof(uint16_t);

	// Calculate timestamp size
	timestamp_size_samples = (channel_list.size() == 2 ? 2 : 1);

	// Calculate buffer size based on timestamping
	uint32_t buffer_len_timestamp = timestamp_every;
	if (buffer_len_timestamp > 0) {
		// Add space for timestamp, based on number of enabled channels
		// With one channel enabled, a timestamp takes up the space of two samples
		// With two channels enabled, a timestamp takes up the space of one sample
		buffer_len_timestamp += timestamp_size_samples;
	}

	// Calculate buffer length
	if (args.count("bufflen") != 0) {
		// Buffer length provided
		size_t buffer_length;

		// Convert argument from string to integer
		try {
			buffer_length = std::stoi(args.at("bufflen"));

		} catch (const std::invalid_argument &) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "bad bufflen provided");
			throw std::runtime_error("bad bufflen provided\n");
		}

		// If buffer length provided, while timestamping enabled, check that two values are equal
		// for now every buffer is expected to have a timestamp
		if (    (buffer_length > 0)
			&& (buffer_len_timestamp > 0)
			&& (buffer_length != buffer_len_timestamp)
		   ) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "bufflen provided incompatible with timestamp_every");
			throw std::runtime_error("bufflen provided incompatible with timestamp_every\n");
		}

		// Set length
		set_buffer_size(buffer_length);

	} else if (buffer_len_timestamp > 0) {
		// Buffer length set based on timestamping
		set_buffer_size(buffer_len_timestamp);

	} else {
		// Buffer length not provided and timestamping disabled, assume same default as iio tx streamer
		set_buffer_size(4096);
	}

	// Setup timestamping
	SoapyPlutoSDR_TimestampEvery::update_device_timestamp_every(dev, timestamp_every, channel_list.size());

	// Assume direct copying is supported
	direct_copy = true;

	// Direct copy only supported for a single channel (of I + Q samples)
	if (channel_list.size() != 2) direct_copy = false;

	// Check endianess
	int16_t test_dst, test_src = 0x1234;
	iio_channel_convert(channel_list[0], &test_dst, (const void *)&test_src);
	if (test_src != test_dst) direct_copy = false;

	// Report status
	SoapySDR_logf(SOAPY_SDR_INFO, "Has direct TX copy: %d", (int)direct_copy);
}

tx_streamer_usb_gadget::~tx_streamer_usb_gadget()
{
	if (thread.joinable()) {
		_stop();
	}
}

int tx_streamer_usb_gadget::send(const void * const *buffs,
								 const size_t numElems,
								 int &flags,
								 const long long timeNs,
								 const long timeoutUs)
{
	// Convert timestamp, in case we need to use it
	uint64_t temp_timestamp = SoapySDR::timeNsToTicks(timeNs, sample_rate);

	// Check if timestamping enabled
	if (	curr_buffer
		 && (timestamp_every > 0)
		 && (flags & SOAPY_SDR_HAS_TIME)
	   ) {
		// Buffer available, timestamping enabled and time provided
		// Calculate timestamp difference and sample count difference
		uint64_t timestamp_diff = (temp_timestamp - curr_buffer_timestamp);
		size_t curr_buffer_free_samples = buffer_size_samples - curr_buffer_samples_stored;

		// Calculate how many samples to add to buffer
		// No need to reset samples as buffer is zero initialized when created
		size_t samples_to_fill = std::min(timestamp_diff, curr_buffer_free_samples);

		// Increment sample count
		curr_buffer_samples_stored += samples_to_fill;

		// Increment timestamp ticks
		curr_buffer_timestamp += samples_to_fill;

		// Flush buffer if full
		if (curr_buffer_samples_stored == buffer_size_samples) {
			// Flush and return error code
			int rc = flush(timeoutUs);
			if (0 != rc) return rc;
		}
	}

	if (!curr_buffer) {
		// Allocate new buffer
		curr_buffer = std::make_shared<std::vector<uint8_t>>();
		curr_buffer->resize(buffer_size_samples * sample_size_bytes);

		// Reset samples stored and offset
		curr_buffer_samples_stored = 0;

		// Check if timestamping enabled
		if (timestamp_every > 0) {
			// Timestamping enabled and timestamp provided, capture it
			if (temp_timestamp < curr_buffer_timestamp) {
				// Warn timestamp has jumped backwards
				SoapySDR_logf(SOAPY_SDR_WARNING, "Backwards timestamp step!");
			}

			// Capture timestamp
			curr_buffer_timestamp = temp_timestamp;

			// Place timestamp at start of buffer
			*((uint64_t*)curr_buffer->data()) = curr_buffer_timestamp;

			// Increment sample count
			curr_buffer_samples_stored += timestamp_size_samples;
		}
	}

	// Work out how many items to copy
	size_t items = std::min(buffer_size_samples - curr_buffer_samples_stored, numElems);

	int16_t src = 0;
	int16_t const *src_ptr = &src;

	if (direct_copy && format == PLUTO_SDR_CS16) {
		// optimize for single TX, 2 channel (I/Q), same endianess direct copy
		int16_t *dst_ptr = (int16_t *)curr_buffer->data() + curr_buffer_samples_stored * 2;
		memcpy(dst_ptr, buffs[0], 2 * sizeof(int16_t) * items);

	} else if (direct_copy && format == PLUTO_SDR_CS12) {
		int16_t *dst_ptr = (int16_t *)curr_buffer->data() + curr_buffer_samples_stored * 2;
		uint8_t const *samples_cs12 = (uint8_t *)buffs[0];

		for (size_t index = 0; index < items; ++index) {
			// consume 24 bit (iiqIQQ)
			uint16_t src0 = uint16_t(*(samples_cs12++));
			uint16_t src1 = uint16_t(*(samples_cs12++));
			uint16_t src2 = uint16_t(*(samples_cs12++));
			// produce 2x 16 bit, note the output is MSB aligned, scale=32768
			// note: byte0 = i[11:4]; byte1 = {q[7:4], i[15:12]}; byte2 = q[15:8];
			*dst_ptr = int16_t((src1 << 12) | (src0 << 4));
			dst_ptr++;
			*dst_ptr = int16_t((src2 << 8) | (src1 & 0xf0));
			dst_ptr++;
		}

	} else if (direct_copy && format == PLUTO_SDR_CS8) {
		int16_t *dst_ptr = (int16_t *)curr_buffer->data() + curr_buffer_samples_stored * 2;
		int8_t const *samples_cs8 = (int8_t *)buffs[0];

		for (size_t index = 0; index < items * 2; ++index) {
			// consume (2x) 8bit (IQ)
			// produce (2x) 16 bit, note the output is MSB aligned, scale=32768
			*dst_ptr = int16_t(*samples_cs8) << 8;
			samples_cs8++;
			dst_ptr++;
		}

	} else if (format == PLUTO_SDR_CS12) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "CS12 not available with this endianess or channel layout");
		throw std::runtime_error("CS12 not available with this endianess or channel layout");

	} else {
		for (unsigned int k = 0; k < channel_list.size(); k++) {
			iio_channel *chn = channel_list[k];
			unsigned int index = k / 2;

			uint8_t *dst_ptr = curr_buffer->data() + (curr_buffer_samples_stored * sample_size_bytes) + (sizeof(uint16_t) * k);

			// note that TX expects samples MSB aligned, unlike RX which is LSB aligned
			// Given two channels, caller provides a single buffer for I/Q sample pairs.
			// Given four channels, caller provides two buffers. Channels 0,1 become I/Q pairs of buffer 0, Channels 2,3 become I/Q pairs of buffer 1.
			if (format == PLUTO_SDR_CS16) {

				int16_t *samples_cs16 = (int16_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					src = samples_cs16[j * 2 + (k % 2)];
					iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
					dst_ptr += sample_size_bytes;
				}
			}
			else if (format == PLUTO_SDR_CF32) {

				float *samples_cf32 = (float *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					src = (int16_t)(samples_cf32[j * 2 + (k % 2)] * 32767.999f); // 32767.999f (0x46ffffff) will ensure better distribution
					iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
					dst_ptr += sample_size_bytes;
				}
			}
			else if (format == PLUTO_SDR_CS8) {

				int8_t *samples_cs8 = (int8_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					src = (int16_t)(samples_cs8[j * 2 + (k % 2)] << 8);
					iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
					dst_ptr += sample_size_bytes;
				}
			}
		}
	}

	// Increment items in buffer
	curr_buffer_samples_stored += items;

	// Increment timestamp ticks
	curr_buffer_timestamp += items;

	if (curr_buffer_samples_stored == buffer_size_samples || (flags & SOAPY_SDR_END_BURST && numElems == items)) {
		// Buffer is full, or caller has indicated this is the end of the burst, send buffer, returning any errors
		int rc = flush(timeoutUs);
		if (0 != rc) return rc;
	}

	// Return number of samples copied
	return items;
}

int tx_streamer_usb_gadget::flush(const long timeoutUs)
{
	int result = 0;

	if (!queue.push(curr_buffer, (timeoutUs > 0), timeoutUs)) {
		// Failed to push entry within timeout
		result = SOAPY_SDR_TIMEOUT;
	}

	// Reset buffer
	curr_buffer.reset();

	return result;
}

int tx_streamer_usb_gadget::start(const int flags,
								  const long long timeNs,
								  const size_t numElems)
{
	// Issue start command
	_start();

	return 0;
}

int tx_streamer_usb_gadget::stop(const int flags,
								 const long long timeNs)
{
	// Issue stop command
	_stop();

	return 0;
}

size_t tx_streamer_usb_gadget::get_mtu_size()
{
	// Return size of buffer data area
	return buffer_size_samples - timestamp_size_samples;
}

void tx_streamer_usb_gadget::set_samplerate(const size_t _samplerate)
{
	// Store updated sample rate
	sample_rate = _samplerate;
}

void tx_streamer_usb_gadget::set_buffer_size(const size_t _buffer_size)
{
	if (buffer_size_samples != _buffer_size) {
		// Is thread currently running?
		bool was_running = thread.joinable();

		// Buffer size changing
		if (was_running) {
			// Stop stream
			_stop();
		}

		// Save new buffer size
		buffer_size_samples = _buffer_size;

		if (was_running) {
			// Start stream
			_start();
		}
	}
}

void tx_streamer_usb_gadget::thread_func(uint32_t curr_enabled_channels, uint32_t curr_buffer_size_samples)
{
	cmd_usb_start_request_t cmd;
	size_t curr_buffer_size_bytes;

	// Start stream
	cmd.enabled_channels = curr_enabled_channels;
	cmd.buffer_size = curr_buffer_size_samples;
	int rc = libusb_control_transfer(usb_dev,
									 LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
									 SDR_USB_GADGET_COMMAND_START,
									 SDR_USB_GADGET_COMMAND_TARGET_TX,
									 intfc_num,
									 (unsigned char*)&cmd,
									 sizeof(cmd),
									 1000);
	if (rc < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Failed to start TX stream (%d)", rc);
		return;
	}

	// Calculate buffer size in bytes
	curr_buffer_size_bytes = curr_buffer_size_samples * sample_size_bytes;

	// Declare buffer
	std::shared_ptr<std::vector<uint8_t>> buffer;

	// Keep running until told to stop
	while (!thread_stop.load()) {
		// Read buffer from queue with 1s timeout
		if (queue.pop(buffer, true, 1000000) && (buffer->size() == curr_buffer_size_bytes)) {
			// Retrieved valid data block, send it with 1s timeout
			int bytes_transferred = 0;
			int rc = libusb_bulk_transfer(usb_dev, ep_num, buffer->data(), buffer->size(), &bytes_transferred, 1000);
			if (LIBUSB_SUCCESS != rc || ((size_t)bytes_transferred != buffer->size())) {
				// Transfer failed
			}
		}
	}

	// Stop stream
	rc = libusb_control_transfer(usb_dev,
								 LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
								 SDR_USB_GADGET_COMMAND_STOP,
								 SDR_USB_GADGET_COMMAND_TARGET_TX,
								 intfc_num,
								 nullptr,
								 0,
								 1000);
	if (rc < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Failed to stop TX stream (%d)", rc);
		return;
	}
}

void tx_streamer_usb_gadget::_start(void)
{
	if (!thread.joinable()) {
		// Reset signal
		thread_stop = false;

		// Start thread, passing it current settings
		thread = std::thread(&tx_streamer_usb_gadget::thread_func, this, enabled_channels, buffer_size_samples);

		// Attempt to increase thread priority
		int max_prio = sched_get_priority_max(SCHED_RR);
		if (max_prio >= 0) {
			sched_param sch;
			sch.sched_priority = max_prio;
			if (int rc = pthread_setschedparam(thread.native_handle(), SCHED_RR, &sch)) {
				SoapySDR_logf(SOAPY_SDR_WARNING, "Failed to set TX thread priority (%d)", rc);
			}
		} else {
			SoapySDR_logf(SOAPY_SDR_WARNING, "Failed to query thread schedular priorities");
		}
	}
}

void tx_streamer_usb_gadget::_stop(void)
{
	if (thread.joinable()) {
		// Signal thread to stop
		thread_stop = true;

		// Wait for thread to stop
		thread.join();
	}
}
