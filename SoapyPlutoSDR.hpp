#include <iio.h>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Formats.hpp>

#ifdef HAS_LIBUSB1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <libusb.h>
#pragma GCC diagnostic pop
#endif

#include "PlutoSDR_StreamFormat.hpp"
#include "PlutoSDR_RXStreamer.hpp"
#include "PlutoSDR_TXStreamer.hpp"
#include "PlutoSDR_RXStreamerIPGadget.hpp"
#include "PlutoSDR_TXStreamerIPGadget.hpp"
#ifdef HAS_LIBUSB1
#include "PlutoSDR_RXStreamerUSBGadget.hpp"
#include "PlutoSDR_TXStreamerUSBGadget.hpp"
#endif

class rx_streamer_iio : public rx_streamer {
	public:
		rx_streamer_iio(const iio_device *dev, const plutosdrStreamFormat format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
		~rx_streamer_iio();
		size_t recv(void * const *buffs,
				const size_t numElems,
				int &flags,
				long long &timeNs,
				const long timeoutUs=100000);
		int start(const int flags,
				const long long timeNs,
				const size_t numElems);

		int stop(const int flags,
				const long long timeNs);

		void set_buffer_size_by_samplerate(const size_t _samplerate);

        size_t get_mtu_size();

	private:

		void set_buffer_size(const size_t _buffer_size);
        void set_mtu_size(const size_t mtu_size);

		bool has_direct_copy();

		std::vector<iio_channel* > channel_list;
		const iio_device  *dev;

		size_t buffer_size;
		size_t byte_offset;
		size_t items_in_buffer;
		iio_buffer  *buf;
		const plutosdrStreamFormat format;
		bool direct_copy;
        size_t mtu_size;

};

class tx_streamer_iio : public tx_streamer {

	public:
		tx_streamer_iio(const iio_device *dev, const plutosdrStreamFormat format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
		~tx_streamer_iio();
		int send(const void * const *buffs,const size_t numElems,int &flags,const long long timeNs,const long timeoutUs );
		int flush(const long timeoutUs);
		int start(const int flags,
				  const long long timeNs,
				  const size_t numElems) {
			return 0;
		}
		int stop(const int flags,
				 const long long timeNs) {
			return flush(0);
		}
		size_t get_mtu_size() {
			return 4096;
		}
		void set_samplerate(const size_t _samplerate) {
			// Do nothing
		}
	private:
		int send_buf();
		bool has_direct_copy();

		std::vector<iio_channel* > channel_list;
		const iio_device  *dev;
		const plutosdrStreamFormat format;

		iio_buffer  *buf;
		size_t buf_size;
		size_t items_in_buf;
		bool direct_copy;

};

// A local spin_mutex usable with std::lock_guard
       //for lightweight locking for short periods.
class pluto_spin_mutex {

public:
    pluto_spin_mutex() = default;

    pluto_spin_mutex(const pluto_spin_mutex&) = delete;

    pluto_spin_mutex& operator=(const pluto_spin_mutex&) = delete;

    ~pluto_spin_mutex() { lock_state.clear(std::memory_order_release); }

    void lock() { while (lock_state.test_and_set(std::memory_order_acquire)); }

    void unlock() { lock_state.clear(std::memory_order_release); }

private:
    std::atomic_flag lock_state = ATOMIC_FLAG_INIT;
};


class SoapyPlutoSDR : public SoapySDR::Device{

	public:
		SoapyPlutoSDR( const SoapySDR::Kwargs & args );
		~SoapyPlutoSDR();

		/*******************************************************************
		 * Identification API
		 ******************************************************************/

		std::string getDriverKey( void ) const;


		std::string getHardwareKey( void ) const;


		SoapySDR::Kwargs getHardwareInfo( void ) const;


		/*******************************************************************
		 * Channels API
		 ******************************************************************/

		size_t getNumChannels( const int ) const;


		bool getFullDuplex( const int direction, const size_t channel ) const;

		/*******************************************************************
		 * Stream API
		 ******************************************************************/

		std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

		std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

		SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

		SoapySDR::Stream *setupStream(
				const int direction,
				const std::string &format,
				const std::vector<size_t> &channels = std::vector<size_t>(),
				const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );

		void closeStream( SoapySDR::Stream *stream );

		size_t getStreamMTU( SoapySDR::Stream *stream ) const;

		int activateStream(
				SoapySDR::Stream *stream,
				const int flags = 0,
				const long long timeNs = 0,
				const size_t numElems = 0 );

		int deactivateStream(
				SoapySDR::Stream *stream,
				const int flags = 0,
				const long long timeNs = 0 );

		int readStream(
				SoapySDR::Stream *stream,
				void * const *buffs,
				const size_t numElems,
				int &flags,
				long long &timeNs,
				const long timeoutUs = 100000 );

		int writeStream(
				SoapySDR::Stream *stream,
				const void * const *buffs,
				const size_t numElems,
				int &flags,
				const long long timeNs = 0,
				const long timeoutUs = 100000);

		int readStreamStatus(
				SoapySDR::Stream *stream,
				size_t &chanMask,
				int &flags,
				long long &timeNs,
				const long timeoutUs
				);


		/*******************************************************************
 		 * Sensor API
 		 ******************************************************************/

		std::vector<std::string> listSensors(void) const;

		SoapySDR::ArgInfo getSensorInfo(const std::string &key) const;

		std::string readSensor(const std::string &key) const;


		/*******************************************************************
		 * Settings API
		 ******************************************************************/

		SoapySDR::ArgInfoList getSettingInfo(void) const;


		void writeSetting(const std::string &key, const std::string &value);


		std::string readSetting(const std::string &key) const;


		/*******************************************************************
		 * Antenna API
		 ******************************************************************/

		std::vector<std::string> listAntennas( const int direction, const size_t channel ) const;


		void setAntenna( const int direction, const size_t channel, const std::string &name );


		std::string getAntenna( const int direction, const size_t channel ) const;


		/*******************************************************************
		 * Frontend corrections API
		 ******************************************************************/

		bool hasDCOffsetMode( const int direction, const size_t channel ) const;


		/*******************************************************************
		 * Gain API
		 ******************************************************************/

		std::vector<std::string> listGains( const int direction, const size_t channel ) const;


		bool hasGainMode(const int direction, const size_t channel) const;


		void setGainMode( const int direction, const size_t channel, const bool automatic );


		bool getGainMode( const int direction, const size_t channel ) const;


		void setGain( const int direction, const size_t channel, const double value );


		void setGain( const int direction, const size_t channel, const std::string &name, const double value );


		double getGain( const int direction, const size_t channel, const std::string &name ) const;


		SoapySDR::Range getGainRange( const int direction, const size_t channel, const std::string &name ) const;


		/*******************************************************************
		 * Frequency API
		 ******************************************************************/

		void setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


		double getFrequency( const int direction, const size_t channel, const std::string &name ) const;


		SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;


		std::vector<std::string> listFrequencies( const int direction, const size_t channel ) const;


		SoapySDR::RangeList getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const;


		/*******************************************************************
		 * Sample Rate API
		 ******************************************************************/

		void setSampleRate( const int direction, const size_t channel, const double rate );


		double getSampleRate( const int direction, const size_t channel ) const;


		std::vector<double> listSampleRates( const int direction, const size_t channel ) const;


		void setBandwidth( const int direction, const size_t channel, const double bw );


		double getBandwidth( const int direction, const size_t channel ) const;


		std::vector<double> listBandwidths( const int direction, const size_t channel ) const;

		SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;

		bool hasHardwareTime(const std::string &what) const;
		long long getHardwareTime(const std::string &what) const;
		void setHardwareTime(const long long timeNs, const std::string &what);

	private:

        bool IsValidRxStreamHandle(SoapySDR::Stream* handle) const;
        bool IsValidTxStreamHandle(SoapySDR::Stream* handle) const;

		bool is_sensor_channel(struct iio_channel *chn) const;
		double double_from_buf(const char *buf) const;
		double get_sensor_value(struct iio_channel *chn) const;
		std::string id_to_unit(const std::string &id) const;

		iio_device *dev;
		iio_device *rx_dev;
		iio_device *tx_dev;
		bool gainMode;

		mutable pluto_spin_mutex rx_device_mutex;
        mutable pluto_spin_mutex tx_device_mutex;

		bool decimation, interpolation;
		std::unique_ptr<rx_streamer> rx_stream;
        std::unique_ptr<tx_streamer> tx_stream;

		int loopback;
		uint32_t timestamp_every_rx;
		uint32_t timestamp_every_tx;

		void handle_direct_args(const SoapySDR::Kwargs & args);
		void handle_loopback_args(const SoapySDR::Kwargs & args);
		void handle_timestamp_every_arg(const SoapySDR::Kwargs & args, bool tx);

		/* "Direct" IO usb gadget */
		#ifdef HAS_LIBUSB1
		libusb_device_handle* usb_sdr_dev;
		uint8_t usb_sdr_intfc_num, usb_sdr_ep_in, usb_sdr_ep_out;
		void open_sdr_usb_gadget(void);
		#endif

		/* "Direct" IP ethernet gadget */
		int ip_sdr_dev_control;
		int ip_sdr_dev_data;
		size_t udp_packet_size;
		void open_sdr_ip_gadget(void);
};

