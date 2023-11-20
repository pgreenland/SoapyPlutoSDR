#include <cstring>

#include <sched.h>
#include <pthread.h>

#include <arpa/inet.h>

#include "PlutoSDR_TXStreamerIPGadget.hpp"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

#include "sdr_ip_gadget_types.h"

tx_streamer_ip_gadget::tx_streamer_ip_gadget(const iio_device *_dev, int _sock_control, int _sock_data, size_t _udp_packet_size, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args, uint32_t _timestamp_every):
	dev(_dev), sock_control(_sock_control), sock_data(_sock_data), udp_packet_size(_udp_packet_size), format(_format), timestamp_every(_timestamp_every), queue(16, false)

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
			&& (timestamp_every > 0)
		   ) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "bufflen provided incompatible with timestamp_every");
			throw std::runtime_error("bufflen provided incompatible with timestamp_every\n");
		}

		// Set length
		set_buffer_size(buffer_length);

	} else if (timestamp_every > 0) {
		// Buffer length set based on timestamping
		set_buffer_size(timestamp_every);

	} else {
		// Buffer length not provided and timestamping disabled, assume same default as iio tx streamer
		set_buffer_size(4096);
	}

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

	// Reset current timestamp
	curr_buffer_timestamp = 0;
}

tx_streamer_ip_gadget::~tx_streamer_ip_gadget()
{
	if (thread.joinable()) {
		_stop();
	}
}

int tx_streamer_ip_gadget::send(const void * const *buffs,
								 const size_t numElems,
								 int &flags,
								 const long long timeNs,
								 const long timeoutUs)
{
	// Check if timestamping enabled
	if (timestamp_every > 0) {
		// Timestamping enabled, check time provided
		if (0 == (flags & SOAPY_SDR_HAS_TIME)) {
			// No time provided
			SoapySDR_logf(SOAPY_SDR_ERROR, "Timestamping enabled but no timestamp provided");
			throw std::runtime_error("Timestamping enabled but no timestamp provided");
		}

		// Check if a buffer is open
		if (curr_buffer) {
			// Buffer already open, convert first sample timestamp
			uint64_t temp_timestamp = SoapySDR::timeNsToTicks(timeNs, sample_rate);

			// Calculate timestamp difference and sample count difference
			uint64_t timestamp_diff = (temp_timestamp - curr_buffer_timestamp);
			size_t curr_buffer_free_samples = buffer_size_samples - curr_buffer_samples_stored;

			// Calculate how many samples to add to buffer
			// No need to reset samples as buffer is zero initialized when created
			size_t samples_to_fill = std::min(timestamp_diff, curr_buffer_free_samples);

			// Increment sample count
			curr_buffer_samples_stored += samples_to_fill;

			// Flush buffer if full
			if (curr_buffer_samples_stored == buffer_size_samples) {
				// Flush and return error code
				int rc = flush(timeoutUs);
				if (0 != rc) return rc;
			}
		}
	}

	if (!curr_buffer) {
		// Allocate new buffer
		curr_buffer = std::make_shared<hdr_payload_t>();
		curr_buffer->hdr.magic = SDR_IP_GADGET_MAGIC;
		curr_buffer->payload.resize(buffer_size_samples * sample_size_bytes);

		// Reset samples stored and offset
		curr_buffer_samples_stored = 0;

		// Check if timestamping enabled
		if (timestamp_every > 0) {
			// Capture timestamp
			curr_buffer_timestamp = SoapySDR::timeNsToTicks(timeNs, sample_rate);

			// Store timestamp in header
			curr_buffer->hdr.seqno = curr_buffer_timestamp;
		} else {
			// Store current sequence number in header
			curr_buffer->hdr.seqno = curr_buffer_timestamp;
		}
	}

	// Work out how many items to copy
	size_t items = std::min(buffer_size_samples - curr_buffer_samples_stored, numElems);

	int16_t src = 0;
	int16_t const *src_ptr = &src;

	if (direct_copy && format == PLUTO_SDR_CS16) {
		// optimize for single TX, 2 channel (I/Q), same endianess direct copy
		int16_t *dst_ptr = (int16_t *)curr_buffer->payload.data() + curr_buffer_samples_stored * 2;
		memcpy(dst_ptr, buffs[0], 2 * sizeof(int16_t) * items);

	} else if (direct_copy && format == PLUTO_SDR_CS12) {
		int16_t *dst_ptr = (int16_t *)curr_buffer->payload.data() + curr_buffer_samples_stored * 2;
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
		int16_t *dst_ptr = (int16_t *)curr_buffer->payload.data() + curr_buffer_samples_stored * 2;
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

			uint8_t *dst_ptr = curr_buffer->payload.data() + (curr_buffer_samples_stored * sample_size_bytes) + (sizeof(uint16_t) * k);

			// note that TX expects samples MSB aligned, unlike RX which is LSB aligned
			if (format == PLUTO_SDR_CS16) {

				int16_t *samples_cs16 = (int16_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					src = samples_cs16[j*2+k];
					iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
					dst_ptr += sample_size_bytes;
				}
			}
			else if (format == PLUTO_SDR_CF32) {

				float *samples_cf32 = (float *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					src = (int16_t)(samples_cf32[j*2+k] * 32767.999f); // 32767.999f (0x46ffffff) will ensure better distribution
					iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
					dst_ptr += sample_size_bytes;
				}
			}
			else if (format == PLUTO_SDR_CS8) {

				int8_t *samples_cs8 = (int8_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					src = (int16_t)(samples_cs8[j*2+k] << 8);
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

int tx_streamer_ip_gadget::flush(const long timeoutUs)
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

int tx_streamer_ip_gadget::start(const int flags,
								  const long long timeNs,
								  const size_t numElems)
{
	// Issue start command
	_start();

	return 0;
}

int tx_streamer_ip_gadget::stop(const int flags,
								 const long long timeNs)
{
	// Issue stop command
	_stop();

	return 0;
}

size_t tx_streamer_ip_gadget::get_mtu_size()
{
	// Return size of buffer data area
	return buffer_size_samples;
}

void tx_streamer_ip_gadget::set_samplerate(const size_t _samplerate)
{
	// Store updated sample rate
	sample_rate = _samplerate;
}

void tx_streamer_ip_gadget::set_buffer_size(const size_t _buffer_size)
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

void tx_streamer_ip_gadget::thread_func(uint32_t curr_enabled_channels, uint32_t curr_buffer_size_samples)
{
	cmd_ip_t cmd;

	// Start stream
	cmd.hdr.magic = SDR_IP_GADGET_MAGIC;
	cmd.hdr.cmd = SDR_IP_GADGET_COMMAND_START_TX;
	cmd.start_tx.enabled_channels = curr_enabled_channels;
	cmd.start_tx.timestamping_enabled = (timestamp_every > 0);
	cmd.start_tx.buffer_size = curr_buffer_size_samples;
	if (timestamp_every > 0) cmd.start_tx.buffer_size += timestamp_size_samples;
	int rc = sendto(sock_control, &cmd, sizeof(cmd.start_tx), 0, NULL, 0);
	if (rc < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Failed to send start TX stream cmd (%d)", rc);
		return;
	}

	// Declare buffer
	std::shared_ptr<hdr_payload_t> buffer;

	// Keep running until told to stop
	while (!thread_stop.load()) {
		// Read buffer from queue with 1s timeout
		if (queue.pop(buffer, true, 1000000)) {
			// Retrieved valid data block, prepare scatter/gather to send header and data
			struct iovec iov[2];
			struct msghdr msg;
			std::memset(&msg, 0x00, sizeof(msg));
			msg.msg_iov = iov;
			msg.msg_iovlen = 2;
			iov[0].iov_base = &buffer->hdr;
			iov[0].iov_len = sizeof(buffer->hdr);

			/* Break block down into datagrams for transmission */
			size_t offset = 0;
			uint8_t *payload = buffer->payload.data();
			while (offset < buffer->payload.size())
			{
				/* Calculate number of bytes left to send */
				size_t bytes_to_send = (buffer->payload.size() - offset);

				/* Limit to configured packet size, reserving space for the header */
				if ((bytes_to_send + sizeof(buffer->hdr)) > udp_packet_size) {
					bytes_to_send = udp_packet_size - sizeof(buffer->hdr);
				}

				/* Calculate number of samples to send */
				size_t samples_to_send = bytes_to_send / sample_size_bytes;

				/* Update bytes to send, effectively rounding to whole number of samples */
				bytes_to_send = samples_to_send * sample_size_bytes;

				/* Send datagram */
				iov[1].iov_base = &payload[offset];
				iov[1].iov_len = bytes_to_send;
				rc = sendmsg(sock_data, &msg, 0);
				if (-1 == rc) {
					// Transfer failed
				}

				/* Advance timestamp */
				buffer->hdr.seqno += samples_to_send;

				/* Advance offset */
				offset += bytes_to_send;
			}
		}
	}

	// Stop stream
	cmd.hdr.magic = SDR_IP_GADGET_MAGIC;
	cmd.hdr.cmd = SDR_IP_GADGET_COMMAND_STOP_TX;
	rc = sendto(sock_control, &cmd, sizeof(cmd.stop), 0, NULL, 0);
	if (rc < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Failed to send stop TX stream cmd (%d)", rc);
		return;
	}
}

void tx_streamer_ip_gadget::_start(void)
{
	if (!thread.joinable()) {
		// Reset signal
		thread_stop = false;

		// Start thread, passing it current settings
		thread = std::thread(&tx_streamer_ip_gadget::thread_func, this, enabled_channels, buffer_size_samples);

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

void tx_streamer_ip_gadget::_stop(void)
{
	if (thread.joinable()) {
		// Signal thread to stop
		thread_stop = true;

		// Wait for thread to stop
		thread.join();
	}
}
