#include "SoapyPlutoSDR.hpp"
#include <memory>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <algorithm>
#include <chrono>

#include <SoapySDR/Time.hpp>


std::vector<std::string> SoapyPlutoSDR::getStreamFormats(const int direction, const size_t channel) const
{
	std::vector<std::string> formats;

	formats.push_back(SOAPY_SDR_CS8);
	formats.push_back(SOAPY_SDR_CS12);
	formats.push_back(SOAPY_SDR_CS16);
	formats.push_back(SOAPY_SDR_CF32);

	return formats;
}

std::string SoapyPlutoSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	if (direction == SOAPY_SDR_RX) {
		fullScale = 2048; // RX expects 12 bit samples LSB aligned
	}
	else if (direction == SOAPY_SDR_TX) {
		fullScale = 32768; // TX expects 12 bit samples MSB aligned
	}
	return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;

	return streamArgs;
}


bool SoapyPlutoSDR::IsValidRxStreamHandle(SoapySDR::Stream* handle) const
{
    if (handle == nullptr) {
        return false;
    }

    //handle is an opaque pointer hiding either rx_stream or tx_streamer:
    //check that the handle matches one of them, consistently with direction:
    if (rx_stream) {
        //test if these handles really belong to us:
        if (reinterpret_cast<rx_streamer*>(handle) == rx_stream.get()) {
            return true;
        }
    }

    return false;
}

bool SoapyPlutoSDR::IsValidTxStreamHandle(SoapySDR::Stream* handle) const
{
    if (handle == nullptr) {
        return false;
    }

    //handle is an opaque pointer hiding either rx_stream or tx_streamer:
    //check that the handle matches one of them, consistently with direction:
    if (tx_stream) {
        //test if these handles really belong to us:
        if (reinterpret_cast<tx_streamer*>(handle) == tx_stream.get()) {
            return true;
        }
    }

    return false;
}

SoapySDR::Stream *SoapyPlutoSDR::setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels,
		const SoapySDR::Kwargs &args )
{
	//check the format
	plutosdrStreamFormat streamFormat;
	if (format == SOAPY_SDR_CF32) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
		streamFormat = PLUTO_SDR_CF32;
	}
	else if (format == SOAPY_SDR_CS16) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
		streamFormat = PLUTO_SDR_CS16;
	}
	else if (format == SOAPY_SDR_CS12) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS12.");
		streamFormat = PLUTO_SDR_CS12;
	}
	else if (format == SOAPY_SDR_CS8) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS8.");
		streamFormat = PLUTO_SDR_CS8;
	}
	else {
		throw std::runtime_error(
			"setupStream invalid format '" + format + "' -- Only CS8, CS12, CS16 and CF32 are supported by SoapyPlutoSDR module.");
	}

	// Handle any arguments provided during stream creation
	handle_direct_args(args);
	handle_loopback_args(args);
	handle_timestamp_every_arg(args, direction == SOAPY_SDR_TX);

	if(direction == SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		iio_channel_attr_write_bool(
			iio_device_find_channel(dev, "altvoltage0", true), "powerdown", false); // Turn ON RX LO

		if (-1 != ip_sdr_dev_control)
		{
			// Use ip streaming gadget
			this->rx_stream = std::unique_ptr<rx_streamer>(new rx_streamer_ip_gadget (rx_dev,
																					  ip_sdr_dev_control, ip_sdr_dev_data, udp_packet_size,
																					  streamFormat, channels, args, timestamp_every_rx));

		}
		#ifdef HAS_LIBUSB1
		else if (usb_sdr_dev)
		{
			// Use usb streaming gadget
			this->rx_stream = std::unique_ptr<rx_streamer>(new rx_streamer_usb_gadget (rx_dev,
																					   usb_sdr_dev, usb_sdr_intfc_num, usb_sdr_ep_in,
																					   streamFormat, channels, args, timestamp_every_rx));
		}
		else
		#endif
		{
			// Use IIO
			this->rx_stream = std::unique_ptr<rx_streamer>(new rx_streamer_iio (rx_dev, streamFormat, channels, args, timestamp_every_rx));
		}

        return reinterpret_cast<SoapySDR::Stream*>(this->rx_stream.get());
	}

	else if (direction == SOAPY_SDR_TX) {

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		iio_channel_attr_write_bool(
			iio_device_find_channel(dev, "altvoltage1", true), "powerdown", false); // Turn ON TX LO

		if (-1 != ip_sdr_dev_control)
		{
			// Use ip streaming gadget
			this->tx_stream = std::unique_ptr<tx_streamer>(new tx_streamer_ip_gadget (tx_dev,
																					  ip_sdr_dev_control, ip_sdr_dev_data, udp_packet_size,
																					  streamFormat, channels, args, timestamp_every_tx));

		}
		#ifdef HAS_LIBUSB1
		else if (usb_sdr_dev)
		{
			// Use usb streaming gadget
			this->tx_stream = std::unique_ptr<tx_streamer>(new tx_streamer_usb_gadget (tx_dev,
																					   usb_sdr_dev, usb_sdr_intfc_num, usb_sdr_ep_out,
																					   streamFormat, channels, args, timestamp_every_tx));
		}
		else
		#endif
		{
			// Use IIO
			this->tx_stream = std::unique_ptr<tx_streamer>(new tx_streamer_iio (tx_dev, streamFormat, channels, args, timestamp_every_tx));
		}

        return reinterpret_cast<SoapySDR::Stream*>(this->tx_stream.get());
	}

	return nullptr;

}

void SoapyPlutoSDR::closeStream( SoapySDR::Stream *handle)
{
    //scope lock:
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

        if (IsValidRxStreamHandle(handle)) {
            this->rx_stream.reset();

			iio_channel_attr_write_bool(
				iio_device_find_channel(dev, "altvoltage0", true), "powerdown", true); // Turn OFF RX LO
        }
    }

    //scope lock :
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

        if (IsValidTxStreamHandle(handle)) {
            this->tx_stream.reset();

			iio_channel_attr_write_bool(
				iio_device_find_channel(dev, "altvoltage1", true), "powerdown", true); // Turn OFF TX LO
        }
    }
}

size_t SoapyPlutoSDR::getStreamMTU( SoapySDR::Stream *handle) const
{
    std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

    if (IsValidRxStreamHandle(handle)) {

        return this->rx_stream->get_mtu_size();
    }

	if (IsValidTxStreamHandle(handle)) {
		return this->tx_stream->get_mtu_size();
	}

    return 0;
}

int SoapyPlutoSDR::activateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs,
		const size_t numElems )
{
	if (flags & ~SOAPY_SDR_END_BURST)
		return SOAPY_SDR_NOT_SUPPORTED;

    std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

    if (IsValidRxStreamHandle(handle)) {
        return this->rx_stream->start(flags, timeNs, numElems);
    }

    if (IsValidTxStreamHandle(handle)) {
        return this->tx_stream->start(flags, timeNs, numElems);
    }

    return 0;
}

int SoapyPlutoSDR::deactivateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs )
{
    //scope lock:
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

        if (IsValidRxStreamHandle(handle)) {
            return this->rx_stream->stop(flags, timeNs);
        }
    }

    //scope lock :
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

        if (IsValidTxStreamHandle(handle)) {
            return this->tx_stream->stop(flags, timeNs);
        }
    }

	return 0;
}

int SoapyPlutoSDR::readStream(
		SoapySDR::Stream *handle,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs )
{
    //the spin_mutex is especially very useful here for minimum overhead !
    std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

    if (IsValidRxStreamHandle(handle)) {
        return int(this->rx_stream->recv(buffs, numElems, flags, timeNs, timeoutUs));
    } else {
        return SOAPY_SDR_NOT_SUPPORTED;
    }
}

int SoapyPlutoSDR::writeStream(
		SoapySDR::Stream *handle,
		const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )
{
    std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

    if (IsValidTxStreamHandle(handle)) {
        return this->tx_stream->send(buffs, numElems, flags, timeNs, timeoutUs);;
    } else {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

}

int SoapyPlutoSDR::readStreamStatus(
		SoapySDR::Stream *stream,
		size_t &chanMask,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
	return SOAPY_SDR_NOT_SUPPORTED;
}

void rx_streamer_iio::set_buffer_size_by_samplerate(const size_t samplerate) {
	// Store new sample rate
	sample_rate = samplerate;

	// Skip update if user has supplied a fixed buffer size
	if (fixed_buffer_size) return;

    //Adapt buffer size (= MTU) as a tradeoff to minimize readStream overhead but at
    //the same time allow realtime applications. Keep it a power of 2 which seems to be better.
    //so try to target very roughly 60fps [30 .. 100] readStream calls / s for realtime applications.
    int rounded_nb_samples_per_call = (int)::round(samplerate / 60.0);

    int power_of_2_nb_samples = 0;

    while (rounded_nb_samples_per_call > (1 << power_of_2_nb_samples)) {
        power_of_2_nb_samples++;
    }

    this->set_buffer_size(1 << power_of_2_nb_samples);

	SoapySDR_logf(SOAPY_SDR_INFO, "Auto setting Buffer Size: %lu", (unsigned long)buffer_size);
}

rx_streamer_iio::rx_streamer_iio(const iio_device *_dev, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args, uint32_t _timestamp_every):
	dev(_dev), buf(nullptr), format(_format), timestamp_every(_timestamp_every)

{
	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-lpc not found!");
		throw std::runtime_error("cf-ad9361-lpc not found!");
	}
	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	for (i = 0; i < channelIDs.size() * 2; i++) {
		struct iio_channel *chn = iio_device_get_channel(dev, i);
		iio_channel_enable(chn);
		channel_list.push_back(chn);
	}

	// Retrieve sample rate
	iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"sampling_frequency", &sample_rate);

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

	// Assume buffer size will be supplied by user
	fixed_buffer_size = true;

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
		// Buffer length not provided and timestamping disabled, calculate based on sample rate
		fixed_buffer_size = false;
		set_buffer_size_by_samplerate(sample_rate);
	}
}

rx_streamer_iio::~rx_streamer_iio()
{
	if (buf) {
        iio_buffer_cancel(buf);
        iio_buffer_destroy(buf);
    }

    for (unsigned int i = 0; i < channel_list.size(); ++i) {
        iio_channel_disable(channel_list[i]);
    }



}

size_t rx_streamer_iio::recv(void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
    //
	if (items_in_buffer <= 0) {

       // auto before = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

	    if (!buf) {
		    return 0;
	    }

		ssize_t ret = iio_buffer_refill(buf);

        // auto after = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

		if (ret < 0)
			return SOAPY_SDR_TIMEOUT;

		items_in_buffer = (unsigned long)ret / iio_buffer_step(buf);

        // SoapySDR_logf(SOAPY_SDR_INFO, "iio_buffer_refill took %d ms to refill %d items", (int)(after - before), items_in_buffer);

		byte_offset = 0;

		// Check if timestamping enabled
		if (timestamp_every > 0) {
			// Extract timestamp from start of buffer
			curr_buffer_timestamp = *((uint64_t*)iio_buffer_start(buf));

			// Decrement samples and advance offset
			items_in_buffer -= timestamp_size_samples;
			byte_offset += sizeof(uint64_t);
		}
	}

	size_t items = std::min(items_in_buffer,numElems);

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (direct_copy) {
		// optimize for single RX, 2 channel (I/Q), same endianess direct copy
		// note that RX is 12 bits LSB aligned, i.e. fullscale 2048
		uint8_t *src = (uint8_t *)iio_buffer_start(buf) + byte_offset;
		int16_t const *src_ptr = (int16_t *)src;

		if (format == PLUTO_SDR_CS16) {

			::memcpy(buffs[0], src_ptr, 2 * sizeof(int16_t) * items);

		}
		else if (format == PLUTO_SDR_CF32) {

			float *dst_cf32 = (float *)buffs[0];

			for (size_t index = 0; index < items * 2; ++index) {
				*dst_cf32 = float(*src_ptr) / 2048.0f;
				src_ptr++;
				dst_cf32++;
			}

		}
		else if (format == PLUTO_SDR_CS12) {

			int8_t *dst_cs12 = (int8_t *)buffs[0];

			for (size_t index = 0; index < items; ++index) {
				int16_t i = *src_ptr++;
				int16_t q = *src_ptr++;
				// produce 24 bit (iiqIQQ), note the input is LSB aligned, scale=2048
				// note: byte0 = i[7:0]; byte1 = {q[3:0], i[11:8]}; byte2 = q[11:4];
				*dst_cs12++ = uint8_t(i);
				*dst_cs12++ = uint8_t((q << 4) | ((i >> 8) & 0x0f));
				*dst_cs12++ = uint8_t(q >> 4);
			}
		}
		else if (format == PLUTO_SDR_CS8) {

			int8_t *dst_cs8 = (int8_t *)buffs[0];

			for (size_t index = 0; index < items * 2; index++) {
				*dst_cs8 = int8_t(*src_ptr >> 4);
				src_ptr++;
				dst_cs8++;
			}
		}
	}
	else {
		int16_t conv = 0, *conv_ptr = &conv;

		for (unsigned int i = 0; i < channel_list.size(); i++) {
			iio_channel *chn = channel_list[i];
			unsigned int index = i / 2;

			uint8_t *src = (uint8_t *)iio_buffer_first(buf, chn) + byte_offset;

			if (format == PLUTO_SDR_CS16) {

				int16_t *dst_cs16 = (int16_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, conv_ptr, src);
					src += buf_step;
					dst_cs16[j * 2 + i] = conv;
				}
			}
			else if (format == PLUTO_SDR_CF32) {

				float *dst_cf32 = (float *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, conv_ptr, src);
					src += buf_step;
					dst_cf32[j * 2 + i] = float(conv) / 2048.0f;
				}
			}
			else if (format == PLUTO_SDR_CS8) {

				int8_t *dst_cs8 = (int8_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, conv_ptr, src);
					src += buf_step;
					dst_cs8[j * 2 + i] = int8_t(conv >> 4);
				}
			}

		}
	}

	items_in_buffer -= items;
	byte_offset += items * iio_buffer_step(buf);

	// Set flags
	flags = 0;
	if (timestamp_every > 0)
	{
		// Timestamp present
		flags |= SOAPY_SDR_HAS_TIME;
		timeNs = SoapySDR::ticksToTimeNs(curr_buffer_timestamp, sample_rate);

		// Increment timestamp ticks
		curr_buffer_timestamp += items;
	}

	return(items);

}

int rx_streamer_iio::start(const int flags,
		const long long timeNs,
		const size_t numElems)
{
    //force proper stop before
    stop(flags, timeNs);

    // re-create buffer
	buf = iio_device_create_buffer(dev, buffer_size, false);

	if (!buf) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create rx buffer!");
		throw std::runtime_error("Unable to create rx buffer!\n");
	}

	direct_copy = has_direct_copy();

	SoapySDR_logf(SOAPY_SDR_INFO, "Has direct RX copy: %d", (int)direct_copy);

	return 0;

}

int rx_streamer_iio::stop(const int flags,
		const long long timeNs)
{
    //cancel first
    if (buf) {
        iio_buffer_cancel(buf);
    }
    //then destroy
	if (buf) {
		iio_buffer_destroy(buf);
		buf = nullptr;
	}

    items_in_buffer = 0;
    byte_offset = 0;

	return 0;

}

void rx_streamer_iio::set_buffer_size(const size_t _buffer_size){

	if (!buf || this->buffer_size != _buffer_size) {
        //cancel first
        if (buf) {
            iio_buffer_cancel(buf);
        }
        //then destroy
        if (buf) {
            iio_buffer_destroy(buf);
        }

		items_in_buffer = 0;
        byte_offset = 0;

		buf = iio_device_create_buffer(dev, _buffer_size, false);
		if (!buf) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create tx buffer!");
			throw std::runtime_error("Unable to create tx buffer!\n");
		}

	}

	this->buffer_size=_buffer_size;
}

size_t rx_streamer_iio::get_mtu_size() {
	// Return size of buffer data area
	return buffer_size - timestamp_size_samples;
}

// return wether can we optimize for single RX, 2 channel (I/Q), same endianess direct copy
bool rx_streamer_iio::has_direct_copy()
{
	if (channel_list.size() != 2) // one RX with I + Q
		return false;

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (buf_step != 2 * sizeof(int16_t))
		return false;

	if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
		return false;

	int16_t test_dst, test_src = 0x1234;
	iio_channel_convert(channel_list[0], &test_dst, (const void *)&test_src);

	return test_src == test_dst;

}


tx_streamer_iio::tx_streamer_iio(const iio_device *_dev, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args, uint32_t _timestamp_every) :
	dev(_dev), format(_format), buf(nullptr), timestamp_every(_timestamp_every)
{

	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-dds-core-lpc not found!");
		throw std::runtime_error("cf-ad9361-dds-core-lpc not found!");
	}

	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	for (i = 0; i < channelIDs.size() * 2; i++) {
		iio_channel *chn = iio_device_get_channel(dev, i);
		iio_channel_enable(chn);
		channel_list.push_back(chn);
	}

	// Retrieve sample rate
	iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"sampling_frequency", &sample_rate);

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

	direct_copy = has_direct_copy();

	SoapySDR_logf(SOAPY_SDR_INFO, "Has direct TX copy: %d", (int)direct_copy);

}

tx_streamer_iio::~tx_streamer_iio(){

	if (buf) { iio_buffer_destroy(buf); }

	for(unsigned int i=0;i<channel_list.size(); ++i)
		iio_channel_disable(channel_list[i]);

}

void tx_streamer_iio::set_buffer_size(const size_t _buffer_size) {
	if (!buf || this->buf_size != _buffer_size) {
        //cancel first
        if (buf) {
            iio_buffer_cancel(buf);
        }
        //then destroy
        if (buf) {
            iio_buffer_destroy(buf);
        }

		items_in_buf = 0;

		buf = iio_device_create_buffer(dev, _buffer_size, false);
		if (!buf) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create tx buffer!");
			throw std::runtime_error("Unable to create tx buffer!\n");
		}

	}

	this->buf_size=_buffer_size;
}

size_t tx_streamer_iio::get_mtu_size() {
	// Return size of buffer data area
	return buf_size - timestamp_size_samples;
}

int tx_streamer_iio::send(	const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )

{
    if (!buf) {
        return 0;
    }

	// Check if timestamping enabled
	if (timestamp_every > 0) {
		// Timestamping enabled, check time provided
		if (0 == (flags & SOAPY_SDR_HAS_TIME)) {
			// No time provided
			SoapySDR_logf(SOAPY_SDR_ERROR, "Timestamping enabled but no timestamp provided");
			throw std::runtime_error("Timestamping enabled but no timestamp provided");
		}

		// Check if a buffer has samples
		if (items_in_buf > 0) {
			// Buffer has data, convert first sample timestamp
			uint64_t temp_timestamp = SoapySDR::timeNsToTicks(timeNs, sample_rate);

			// Calculate timestamp difference and sample count difference
			uint64_t timestamp_diff = (temp_timestamp - curr_buffer_timestamp);
			size_t curr_buffer_free_samples = buf_size - items_in_buf;

			// Calculate how many samples to add to buffer
			size_t samples_to_fill = std::min(timestamp_diff, curr_buffer_free_samples);

			// Zero new samples
			ptrdiff_t buf_step = iio_buffer_step(buf);
			uint8_t *buf_ptr = (uint8_t *)iio_buffer_start(buf) + items_in_buf * buf_step;
			uint8_t *buf_end = (uint8_t *)iio_buffer_start(buf) + (items_in_buf + samples_to_fill) * buf_step;
			memset(buf_ptr, 0, buf_end - buf_ptr);

			// Increment sample count
			items_in_buf += samples_to_fill;

			// Flush buffer if full
			if (items_in_buf == buf_size) {
				// Flush and return error code
				int ret = send_buf();

				if (ret < 0) {
					return SOAPY_SDR_ERROR;
				}

				if ((size_t)ret != buf_size) {
					return SOAPY_SDR_ERROR;
				}
			}
		}

		// Check if buffer is empty
		if (0 == items_in_buf) {
			// Capture timestamp
			curr_buffer_timestamp = SoapySDR::timeNsToTicks(timeNs, sample_rate);

			// Place timestamp at start of buffer
			*((uint64_t*)iio_buffer_start(buf)) = curr_buffer_timestamp;

			// Increment sample count
			items_in_buf += timestamp_size_samples;
		}
	}

	size_t items = std::min(buf_size - items_in_buf, numElems);

	int16_t src = 0;
	int16_t const *src_ptr = &src;
	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (direct_copy && format == PLUTO_SDR_CS16) {
		// optimize for single TX, 2 channel (I/Q), same endianess direct copy
		int16_t *dst_ptr = (int16_t *)iio_buffer_start(buf) + items_in_buf * 2;
		memcpy(dst_ptr, buffs[0], 2 * sizeof(int16_t) * items);
	}
	else if (direct_copy && format == PLUTO_SDR_CS12) {

		int16_t *dst_ptr = (int16_t *)iio_buffer_start(buf) + items_in_buf * 2;
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
	}
	else if (direct_copy && format == PLUTO_SDR_CS8) {

		int16_t *dst_ptr = (int16_t *)iio_buffer_start(buf) + items_in_buf * 2;
		int8_t const *samples_cs8 = (int8_t *)buffs[0];

		for (size_t index = 0; index < items * 2; ++index) {
			// consume (2x) 8bit (IQ)
			// produce (2x) 16 bit, note the output is MSB aligned, scale=32768
			*dst_ptr = int16_t(*samples_cs8) << 8;
			samples_cs8++;
			dst_ptr++;
		}
	}
	else if (format == PLUTO_SDR_CS12) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "CS12 not available with this endianess or channel layout");
		throw std::runtime_error("CS12 not available with this endianess or channel layout");
	}
	else

	for (unsigned int k = 0; k < channel_list.size(); k++) {
		iio_channel *chn = channel_list[k];
		unsigned int index = k / 2;

		uint8_t *dst_ptr = (uint8_t *)iio_buffer_first(buf, chn) + items_in_buf * buf_step;

		// note that TX expects samples MSB aligned, unlike RX which is LSB aligned
		if (format == PLUTO_SDR_CS16) {

			int16_t *samples_cs16 = (int16_t *)buffs[index];

			for (size_t j = 0; j < items; ++j) {
				src = samples_cs16[j*2+k];
				iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
				dst_ptr += buf_step;
			}
		}
		else if (format == PLUTO_SDR_CF32) {

			float *samples_cf32 = (float *)buffs[index];

			for (size_t j = 0; j < items; ++j) {
				src = (int16_t)(samples_cf32[j*2+k] * 32767.999f); // 32767.999f (0x46ffffff) will ensure better distribution
				iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
				dst_ptr += buf_step;
			}
		}
		else if (format == PLUTO_SDR_CS8) {

			int8_t *samples_cs8 = (int8_t *)buffs[index];

			for (size_t j = 0; j < items; ++j) {
				src = (int16_t)(samples_cs8[j*2+k] << 8);
				iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
				dst_ptr += buf_step;
			}
		}
	}

	items_in_buf += items;

	// Increment timestamp ticks
	curr_buffer_timestamp += items;

	if (items_in_buf == buf_size || (flags & SOAPY_SDR_END_BURST && numElems == items)) {
		int ret = send_buf();

		if (ret < 0) {
			return SOAPY_SDR_ERROR;
		}

		if ((size_t)ret != buf_size) {
			return SOAPY_SDR_ERROR;
		}
	}

	return items;

}

int tx_streamer_iio::flush(const long timeoutUs)
{
	return send_buf();
}

int tx_streamer_iio::send_buf()
{
    if (!buf) {
        return 0;
    }

	if (items_in_buf > 0) {
		if (items_in_buf < buf_size) {
			ptrdiff_t buf_step = iio_buffer_step(buf);
			uint8_t *buf_ptr = (uint8_t *)iio_buffer_start(buf) + items_in_buf * buf_step;
			uint8_t *buf_end = (uint8_t *)iio_buffer_end(buf);

			memset(buf_ptr, 0, buf_end - buf_ptr);
		}

		ssize_t ret = iio_buffer_push(buf);
		items_in_buf = 0;

		if (ret < 0) {
			return ret;
		}

		return int(ret / iio_buffer_step(buf));
	}

	return 0;

}

// return wether can we optimize for single TX, 2 channel (I/Q), same endianess direct copy
bool tx_streamer_iio::has_direct_copy()
{

	if (channel_list.size() != 2) // one TX with I/Q
		return false;

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (buf_step != 2 * sizeof(int16_t))
		return false;

	if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
		return false;

	int16_t test_dst, test_src = 0x1234;
	iio_channel_convert_inverse(channel_list[0], &test_dst, (const void *)&test_src);

	return test_src == test_dst;

}
