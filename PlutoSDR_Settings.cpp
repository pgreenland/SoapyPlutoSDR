#include "SoapyPlutoSDR.hpp"
#ifdef HAS_AD9361_IIO
#include <ad9361.h>
#endif
#ifdef HAS_LIBUSB1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <libusb.h>
#pragma GCC diagnostic pop
#endif

#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

static iio_context *ctx = nullptr;
#ifdef HAS_LIBUSB1
static libusb_context *usb_ctx = nullptr;
#endif

#define DIRECT_IP_PORT_CONTROL (30432) // IIOD + 1
#define DIRECT_IP_PORT_DATA (30433) // IIOD + 2

SoapyPlutoSDR::SoapyPlutoSDR( const SoapySDR::Kwargs &args ):
	dev(nullptr), rx_dev(nullptr),tx_dev(nullptr), decimation(false), interpolation(false), rx_stream(nullptr)
{

	gainMode = false;

	if (args.count("label") != 0)
		SoapySDR_logf( SOAPY_SDR_INFO, "Opening label %s...", args.at("label").c_str());

	if(ctx == nullptr)
	{
	  if(args.count("uri") != 0) {
		  SoapySDR_logf( SOAPY_SDR_INFO, "Opening URI %s...", args.at("uri").c_str());
		  ctx = iio_create_context_from_uri(args.at("uri").c_str());

	  }else if(args.count("hostname")!=0){
		  SoapySDR_logf( SOAPY_SDR_INFO, "Opening hostname %s...", args.at("hostname").c_str());
		  ctx = iio_create_network_context(args.at("hostname").c_str());
	  }else{
		  SoapySDR_logf( SOAPY_SDR_INFO, "Opening default context...");
		  ctx = iio_create_default_context();
	  }
	}

	if (ctx == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "no device context found.");
		throw std::runtime_error("no device context found");
	}

	dev = iio_context_find_device(ctx, "ad9361-phy");
	rx_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
	tx_dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");

	if (dev == nullptr || rx_dev == nullptr || tx_dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "no device found in this context.");
		throw std::runtime_error("no device found in this context");
	}

	this->setAntenna(SOAPY_SDR_RX, 0, "A_BALANCED");
	this->setGainMode(SOAPY_SDR_RX, 0, false);
	this->setAntenna(SOAPY_SDR_TX, 0, "A");

	// Assume not using direct mode
	#ifdef HAS_LIBUSB1
	usb_sdr_dev = nullptr;
	#endif
	ip_sdr_dev_control = -1;
	ip_sdr_dev_data = -1;

	// Assume loopback unchanged
	loopback = -1;

	// Assume timestamps disabled
	timestamp_every_tx = 0;
	timestamp_every_rx = 0;

	// Handle any arguments provided during device creation
	handle_direct_args(args);
	handle_loopback_args(args);
	handle_timestamp_every_arg(args, false);
	handle_timestamp_every_arg(args, true);
}

SoapyPlutoSDR::~SoapyPlutoSDR(void){

	long long samplerate=0;
	if(decimation){
		iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"sampling_frequency",&samplerate);
		iio_channel_attr_write_longlong(iio_device_find_channel(rx_dev, "voltage0", false),"sampling_frequency", samplerate);

	}

	if(interpolation){
		iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"sampling_frequency",&samplerate);
		iio_channel_attr_write_longlong(iio_device_find_channel(tx_dev, "voltage0", true),"sampling_frequency", samplerate);
	}

	if(ctx){
		iio_context_destroy(ctx);
		ctx = nullptr;
	}

	if(this->usb_sdr_dev){
		libusb_close(this->usb_sdr_dev);
		this->usb_sdr_dev = nullptr;
	}

	if(this->rx_stream){
		this->rx_stream.reset();
	}

	if(this->tx_stream){
		this->tx_stream.reset();
	}

	if(this->ip_sdr_dev_control){
		close(this->ip_sdr_dev_control);
		this->ip_sdr_dev_control = -1;
	}
	if(this->ip_sdr_dev_data){
		close(this->ip_sdr_dev_data);
		this->ip_sdr_dev_data = -1;
	}

	if(usb_ctx){
		libusb_exit(usb_ctx);
		usb_ctx = nullptr;
	}
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyPlutoSDR::getDriverKey( void ) const
{
	return "PlutoSDR";
}

std::string SoapyPlutoSDR::getHardwareKey( void ) const
{
	return "ADALM-PLUTO";
}

SoapySDR::Kwargs SoapyPlutoSDR::getHardwareInfo( void ) const
{
	SoapySDR::Kwargs info;

	unsigned int major, minor;
	char git_tag[8];
	iio_library_get_version(&major, &minor, git_tag);
	char lib_ver[100];
	snprintf(lib_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
	info["library_version"] = lib_ver;

	iio_context_get_version(ctx, &major, &minor, git_tag);
	char backend_ver[100];
	snprintf(backend_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
	info["backend_version"] = backend_ver;

	unsigned int nb_ctx_attrs = iio_context_get_attrs_count(ctx);
	for (unsigned int i = 0; i < nb_ctx_attrs; i++) {
		const char *key, *value;
		iio_context_get_attr(ctx, i, &key, &value);
		info[key] = value;
	}

	return info;
}


/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyPlutoSDR::getNumChannels( const int dir ) const
{
	return(1);
}

bool SoapyPlutoSDR::getFullDuplex( const int direction, const size_t channel ) const
{
	return(true);
}


/*******************************************************************
 * Sensor API
 ******************************************************************/

bool SoapyPlutoSDR::is_sensor_channel(struct iio_channel *chn) const
{
	return (!iio_channel_is_output(chn) &&
			(iio_channel_find_attr(chn, "raw") ||
			iio_channel_find_attr(chn, "input")));
}

double SoapyPlutoSDR::double_from_buf(const char *buf) const
{
	std::istringstream val_as_string(buf);
	val_as_string.imbue(std::locale::classic()); // ignore global C++ locale

	double val = 0.0;
	val_as_string >> val;

	return val;
}

double SoapyPlutoSDR::get_sensor_value(struct iio_channel *chn) const
{
	char buf[32];
	double val = 0.0;

	if (iio_channel_find_attr(chn, "input")) {
		if (iio_channel_attr_read(chn, "input", buf, sizeof(buf)) > 0) {
			val = double_from_buf(buf);
		}
	} else {
		if (iio_channel_attr_read(chn, "raw", buf, sizeof(buf)) > 0) {
			val = double_from_buf(buf);
		}

		if (iio_channel_find_attr(chn, "offset")) {
			if (iio_channel_attr_read(chn, "offset", buf, sizeof(buf)) > 0) {
				val += double_from_buf(buf);
			}
		}

		if (iio_channel_find_attr(chn, "scale")) {
			if (iio_channel_attr_read(chn, "scale", buf, sizeof(buf)) > 0) {
				val *= double_from_buf(buf);
			}
		}
	}

	return val / 1000.0;
}

std::string SoapyPlutoSDR::id_to_unit(const std::string& id) const
{
	static std::map<std::string, std::string> id_to_unit_table = {
		{ "current",	"A" },
		{ "power",	"W" },
		{ "temp",	"C" },
		{ "voltage",	"V" },
	};

	for (auto it_match : id_to_unit_table) {

		//if the id starts with a known prefix, retreive its unit.
		if (id.substr(0, it_match.first.size()) == it_match.first) {
			return it_match.second;
		}
	}
	return std::string();
}

std::vector<std::string> SoapyPlutoSDR::listSensors(void) const
{
	/*
	iio:device2: xadc
		10 channels found:
			temp0:  (input)
			voltage0: vccint (input)
			voltage1: vccaux (input)
			voltage2: vccbram (input)
			voltage3: vccpint (input)
			voltage4: vccpaux (input)
			voltage5: vccoddr (input)
			voltage6: vrefp (input)
			voltage7: vrefn (input)
			voltage8:  (input)
	iio:device0: adm1177
		2 channels found:
			current0:  (input)
			voltage0:  (input)
 	iio:device1: ad9361-phy
		9 channels found:
			temp0:  (input)
			voltage2:  (input)
	*/
	std::vector<std::string> sensors;

	sensors.push_back("xadc_temp0");
	sensors.push_back("xadc_voltage0");
	sensors.push_back("xadc_voltage1");
	sensors.push_back("xadc_voltage2");
	sensors.push_back("xadc_voltage3");
	sensors.push_back("xadc_voltage4");
	sensors.push_back("xadc_voltage5");
	sensors.push_back("xadc_voltage6");
	sensors.push_back("xadc_voltage7");
	sensors.push_back("xadc_voltage8");
	sensors.push_back("adm1177_current0");
	sensors.push_back("adm1177_voltage0");
	sensors.push_back("ad9361-phy_temp0");
	sensors.push_back("ad9361-phy_voltage2");

	return sensors;
}

SoapySDR::ArgInfo SoapyPlutoSDR::getSensorInfo(const std::string &key) const
{
	SoapySDR::ArgInfo info;

	std::size_t dash = key.find("_");
	if (dash < std::string::npos)
	{
		std::string deviceStr = key.substr(0, dash);
		std::string channelStr = key.substr(dash + 1);

		iio_device *dev = iio_context_find_device(ctx, deviceStr.c_str());
		if (!dev)
			return info;
		iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
		if (!chn)
			return info;

		const char *name = iio_channel_get_name(chn);
		info.key = key;
		if (name)
			info.name = name;
		info.type = SoapySDR::ArgInfo::FLOAT;
		info.value = "0.0";
		info.units = id_to_unit(channelStr);
	}

	return info;
}

std::string SoapyPlutoSDR::readSensor(const std::string &key) const
{
	std::string sensorValue;

	std::size_t dash = key.find("_");
	if (dash < std::string::npos)
	{
		std::string deviceStr = key.substr(0, dash);
		std::string channelStr = key.substr(dash + 1);

		iio_device *dev = iio_context_find_device(ctx, deviceStr.c_str());
		if (!dev)
			return sensorValue;
		iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
		if (!chn)
			return sensorValue;

		double value = get_sensor_value(chn);
		sensorValue.assign(std::to_string(value));
	}

	return sensorValue;
}


/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyPlutoSDR::getSettingInfo(void) const
{
	SoapySDR::ArgInfoList setArgs;

	return setArgs;
}

void SoapyPlutoSDR::writeSetting(const std::string &key, const std::string &value)
{



}


std::string SoapyPlutoSDR::readSetting(const std::string &key) const
{
	std::string info;

	return info;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyPlutoSDR::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	if(direction == SOAPY_SDR_RX) options.push_back( "A_BALANCED" );
	if(direction == SOAPY_SDR_TX) options.push_back( "A" );
	return(options);
}

void SoapyPlutoSDR::setAntenna( const int direction, const size_t channel, const std::string &name )
{
   if (direction == SOAPY_SDR_RX) {
       std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false), "rf_port_select", name.c_str());
	}

	else if (direction == SOAPY_SDR_TX) {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", true), "rf_port_select", name.c_str());

	}
}


std::string SoapyPlutoSDR::getAntenna( const int direction, const size_t channel ) const
{
	std::string options;

	if (direction == SOAPY_SDR_RX) {
		options = "A_BALANCED";
	}
	else if (direction == SOAPY_SDR_TX) {

		options = "A";
	}
	return options;
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyPlutoSDR::hasDCOffsetMode( const int direction, const size_t channel ) const
{
	return(false);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyPlutoSDR::listGains( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	options.push_back("PGA");
	return(options);
}

bool SoapyPlutoSDR::hasGainMode(const int direction, const size_t channel) const
{
	if (direction == SOAPY_SDR_RX)
		return true;
	return false;
}

void SoapyPlutoSDR::setGainMode( const int direction, const size_t channel, const bool automatic )
{

	gainMode = automatic;
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		if (gainMode) {

			iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false), "gain_control_mode", "slow_attack");

		}else{

			iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false), "gain_control_mode", "manual");
		}

	}
}

bool SoapyPlutoSDR::getGainMode(const int direction, const size_t channel) const
{
	return gainMode;
}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const double value )
{
	long long gain = (long long) value;
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),"hardwaregain", gain);

	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		gain = gain - 89;
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),"hardwaregain", gain);

	}

}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const std::string &name, const double value )
{
	this->setGain(direction,channel,value);
}

double SoapyPlutoSDR::getGain( const int direction, const size_t channel, const std::string &name ) const
{
	long long gain = 0;

	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"hardwaregain",&gain )!=0)
			return 0;

	}

	else if(direction==SOAPY_SDR_TX){

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"hardwaregain",&gain )!=0)
			return 0;
		gain = gain + 89;
	}
	return double(gain);
}

SoapySDR::Range SoapyPlutoSDR::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{
	if(direction==SOAPY_SDR_RX)
		return(SoapySDR::Range(0, 73));
	return(SoapySDR::Range(0,89));

}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyPlutoSDR::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
	long long freq = (long long)frequency;
	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "altvoltage0", true),"frequency", freq);
	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "altvoltage1", true),"frequency", freq);

	}

}

double SoapyPlutoSDR::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
  	long long freq = 0;

	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "altvoltage0", true),"frequency",&freq )!=0)
			return 0;

	}

	else if(direction==SOAPY_SDR_TX){

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "altvoltage1", true),"frequency",&freq )!=0)
			return 0;

	}

	return double(freq);

}

SoapySDR::ArgInfoList SoapyPlutoSDR::getFrequencyArgsInfo(const int direction, const size_t channel) const
{

	SoapySDR::ArgInfoList freqArgs;

	return freqArgs;
}

std::vector<std::string> SoapyPlutoSDR::listFrequencies( const int direction, const size_t channel ) const
{
	std::vector<std::string> names;
	names.push_back( "RF" );
	return(names);
}

SoapySDR::RangeList SoapyPlutoSDR::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
	return(SoapySDR::RangeList( 1, SoapySDR::Range( 70000000, 6000000000ull ) ) );

}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/
void SoapyPlutoSDR::setSampleRate( const int direction, const size_t channel, const double rate )
{
	long long samplerate =(long long) rate;
#ifdef HAS_AD9361_IIO
	int const fir = 4; // assume ad9361_set_bb_rate() will load x4 FIR if needed
#else
	int const fir = 1;
#endif

	// note: sample rates below 25e6/12 need x8 decimation/interpolation or x4 FIR to 25e6/48,
	// below 25e6/96 need x8 decimation/interpolation and x4 FIR, minimum is 25e6/384
	// if libad9361 is available it will load an approporiate FIR.
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		decimation = false;
		if (samplerate < (25e6 / (12 * fir))) {
			if (samplerate * 8 < (25e6 / 48)) {
				SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
			}
			else if (samplerate * 8 < (25e6 / 12)) {
				SoapySDR_logf(SOAPY_SDR_NOTICE, "sample rate needs a FIR setting loaded.");
			}

			decimation = true;
			samplerate = samplerate * 8;
		}

		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),"sampling_frequency", samplerate);

		iio_channel_attr_write_longlong(iio_device_find_channel(rx_dev, "voltage0", false), "sampling_frequency", decimation?samplerate/8:samplerate);

		if(rx_stream)
			rx_stream->set_buffer_size_by_samplerate(decimation ? samplerate / 8 : samplerate);
	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		interpolation = false;
		if (samplerate < (25e6 / (12 * fir))) {
			if (samplerate * 8 < (25e6 / 48)) {
				SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
			}
			else if (samplerate * 8 < (25e6 / 12)) {
				SoapySDR_logf(SOAPY_SDR_NOTICE, "sample rate needs a FIR setting loaded.");
			}

			interpolation = true;
			samplerate = samplerate * 8;
		}


		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),"sampling_frequency", samplerate);
		iio_channel_attr_write_longlong(iio_device_find_channel(tx_dev, "voltage0", true), "sampling_frequency", interpolation?samplerate / 8:samplerate);

		if(tx_stream)
			tx_stream->set_samplerate(decimation ? samplerate / 8 : samplerate);
	}

#ifdef HAS_AD9361_IIO
	if(ad9361_set_bb_rate(dev,(unsigned long)samplerate))
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to set BB rate.");
#endif

}

double SoapyPlutoSDR::getSampleRate( const int direction, const size_t channel ) const
{
	long long samplerate = 0;

	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(rx_dev, "voltage0", false),"sampling_frequency",&samplerate )!=0)
			return 0;
	}

	else if(direction==SOAPY_SDR_TX){

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(tx_dev, "voltage0", true),"sampling_frequency",&samplerate)!=0)
			return 0;

	}

	return double(samplerate);

}

std::vector<double> SoapyPlutoSDR::listSampleRates( const int direction, const size_t channel ) const
{
	std::vector<double> options;

	options.push_back(65105);//25M/48/8+1
	options.push_back(1e6);
	options.push_back(2e6);
	options.push_back(3e6);
	options.push_back(4e6);
	options.push_back(5e6);
	options.push_back(6e6);
	options.push_back(7e6);
	options.push_back(8e6);
	options.push_back(9e6);
	options.push_back(10e6);
	return(options);

}

SoapySDR::RangeList SoapyPlutoSDR::getSampleRateRange( const int direction, const size_t channel ) const
{
	SoapySDR::RangeList results;

	// note that there are some gaps and rounding errors since we get truncated values form IIO
	// e.g. 25e6/12 = 2083333.333 is read as 2083333 but written as 2083334
#ifdef HAS_AD9361_IIO
	// assume ad9361_set_bb_rate(), if available, will load x4 FIR as needed
	// below 25e6/96 needs x8 decimation/interpolation and x4 FIR, minimum is 25e6/384
	results.push_back(SoapySDR::Range(25e6 / 384, 61440000));
#else
	// sample rates below 25e6/12 need x8 decimation/interpolation (or x4 FIR to 25e6/48)
	results.push_back(SoapySDR::Range(25e6 / 96, 61440000));
#endif

	return results;
}

void SoapyPlutoSDR::setBandwidth( const int direction, const size_t channel, const double bw )
{
	long long bandwidth = (long long) bw;
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),"rf_bandwidth", bandwidth);
	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),"rf_bandwidth", bandwidth);
	}

}

double SoapyPlutoSDR::getBandwidth( const int direction, const size_t channel ) const
{
    long long bandwidth = 0;

	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"rf_bandwidth",&bandwidth )!=0)
			return 0;

	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"rf_bandwidth",&bandwidth )!=0)
			return 0;
	}

	return double(bandwidth);

}

std::vector<double> SoapyPlutoSDR::listBandwidths( const int direction, const size_t channel ) const
{
	std::vector<double> options;
	options.push_back(0.2e6);
	options.push_back(1e6);
	options.push_back(2e6);
	options.push_back(3e6);
	options.push_back(4e6);
	options.push_back(5e6);
	options.push_back(6e6);
	options.push_back(7e6);
	options.push_back(8e6);
	options.push_back(9e6);
	options.push_back(10e6);
	return(options);

}

/*******************************************************************
 * Time API
 ******************************************************************/

bool SoapyPlutoSDR::hasHardwareTime(const std::string &what) const
{
	return ((timestamp_every_rx + timestamp_every_tx) > 0);
}

long long SoapyPlutoSDR::getHardwareTime(const std::string &what) const
{
	throw std::runtime_error("SoapyPlutoSDR::getHardwareTime() not yet implemented");
}

void SoapyPlutoSDR::setHardwareTime(const long long timeNs, const std::string &what)
{
	throw std::runtime_error("SoapyPlutoSDR::setHardwareTime() not yet implemented");
}

/*******************************************************************
 * Helpers
 ******************************************************************/

void SoapyPlutoSDR::handle_direct_args(const SoapySDR::Kwargs & args)
{
	std::string direct_val = "0";

	// Lookup legacy value followed by new value
	if (args.count("usb_direct") != 0) direct_val = args.at("usb_direct");
	if (args.count("direct") != 0) direct_val = args.at("direct");

	if (direct_val == "1") {
		if (0 == strcmp(iio_context_get_name(ctx), "usb")) {
			// Connected via usb, check if usb support is available to support direct mode
#ifdef HAS_LIBUSB1
			// Init libusb
			if (usb_ctx == nullptr) {
				int rc = libusb_init(&usb_ctx);
				if (rc < 0) {
					SoapySDR_logf(SOAPY_SDR_ERROR, "libusb init error (%d)", rc);
					throw std::runtime_error("libusb init error");
				}
			}

			if (!this->usb_sdr_dev) {
				// Open usb device
				this->open_sdr_usb_gadget();

				// Notify usb direct mode in use
				SoapySDR_logf(SOAPY_SDR_INFO, "USB direct mode enabled!");
			}
#else
			SoapySDR_logf(SOAPY_SDR_ERROR, "usb direct mode is only available when built with LIBUSB");
			throw std::runtime_error("usb direct mode is only available when built with LIBUSB");
#endif
		}
		else if (0 == strcmp(iio_context_get_name(ctx), "network")) {
			// Connected via network (hopefully physical ethernet)
			if ((-1 == this->ip_sdr_dev_control) || (-1 == this->ip_sdr_dev_data)) {
				// Open ip device
				this->open_sdr_ip_gadget();

				// Notify ip direct mode in use
				SoapySDR_logf(SOAPY_SDR_INFO, "IP direct mode enabled!");
			}

			// Grab UDP packet sizes to use if available
			int udp_packet_size = 1472;
			if (args.count("udp_packet_size") != 0) {
				// Retrieve size
				try {
					udp_packet_size = std::stoi(args.at("udp_packet_size"));
				} catch (...) {
					SoapySDR_logf(SOAPY_SDR_ERROR, "invalid value for udp_packet_size, expected number");
					throw std::runtime_error("invalid value for udp_packet_size, expected number");
				}
			}
			this->udp_packet_size = udp_packet_size;
		}
	}
	else if (direct_val == "0") {
		// default value
	} else {
		SoapySDR_logf(SOAPY_SDR_ERROR, "invalid value for direct, expected 0/1");
		throw std::runtime_error("invalid value for direct, expected 0/1");
	}
}

void SoapyPlutoSDR::handle_loopback_args(const SoapySDR::Kwargs & args)
{
	int new_loopback = -1;

	// Set / reset loopback mode
	if (args.count("loopback") != 0) {
		try {
			new_loopback = std::stoi(args.at("loopback"));
		} catch (...) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "invalid value for loopback, expected number");
			throw std::runtime_error("invalid value for loopback, expected number");
		}
		if (new_loopback < 0 || new_loopback > 2) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "invalid value for loopback, expected 0-2");
			throw std::runtime_error("invalid value for loopback, expected 0-2");
		}

		if (new_loopback != loopback) {
			// Update required
			switch (new_loopback) {
				case 1: {
					SoapySDR_logf(SOAPY_SDR_INFO, "digital loopback enabled");
					break;
				}
				case 2: {
					SoapySDR_logf(SOAPY_SDR_INFO, "analog loopback enabled");
					break;
				}
			}

			int rc = iio_device_debug_attr_write_longlong(dev, "loopback", new_loopback);
			if (rc < 0) {
				SoapySDR_logf(SOAPY_SDR_ERROR, "failed to set loopback mode (%d)", rc);
				throw std::runtime_error("failed to set loopback mode");
			}

			// Save new value
			loopback = new_loopback;
		}
	}
}

void SoapyPlutoSDR::handle_timestamp_every_arg(const SoapySDR::Kwargs & args, bool tx)
{
	uint32_t new_timestamp_every = 0;

	if (args.count("timestamp_every") != 0) {
		try {
			new_timestamp_every = std::stoul(args.at("timestamp_every"));
			SoapySDR_logf(SOAPY_SDR_INFO, "%s timestamping enabled, every %u samples", tx ? "TX" : "RX", new_timestamp_every);
		} catch (...) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "invalid value for %s timestamp_every, expected number", tx ? "TX" : "RX");
			throw std::runtime_error("invalid value for timestamp_every, expected number");
		}

		// Check timestamp every is only asserted with usb_direct for now
		if (new_timestamp_every > 0) {
			#ifdef HAS_LIBUSB1
			if (!this->usb_sdr_dev && -1 == this->ip_sdr_dev_control) {
			#else
			if (-1 == this->ip_sdr_dev_control) {
			#endif
				SoapySDR_logf(SOAPY_SDR_ERROR, "timestamp_every only currently supported with direct mode");
				throw std::runtime_error("timestamp_every only currently supported with direct mode");
			}
		}

		// Set appropriate timestamp_every
		if (tx) {
			timestamp_every_tx = new_timestamp_every;
		} else {
			timestamp_every_rx = new_timestamp_every;
		}
	}
}

#ifdef HAS_LIBUSB1
void SoapyPlutoSDR::open_sdr_usb_gadget(void)
{
	// Retrieve url and separate bus / device
	const char *uri = iio_context_get_attr_value(ctx, "uri");
	if (!uri) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to retrieve uri from iio");
		throw std::runtime_error("failed to retrieve uri from iio");
	}

	// Retrieve bus and device number from uri
	unsigned short int bus_num, dev_addr;
	if (2 != std::sscanf(uri, "usb:%hu.%hu", &bus_num, &dev_addr)) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to extract usb bus and device address from uri");
		throw std::runtime_error("failed to extract usb bus and device address from uri");
	}

	// Retrieve device list
	struct libusb_device **devs;
	int dev_count = libusb_get_device_list(usb_ctx, &devs);
	if (dev_count < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "libusb get device list error (%d)", dev_count);
		throw std::runtime_error("libusb get device list error");
	}

	// Iterate over devices
	for (int i = 0; i < dev_count; i++) {
		struct libusb_device *dev = devs[i];

		// Check device bus and address
		if (	(libusb_get_bus_number(dev) == bus_num)
			 && (libusb_get_device_address(dev) == dev_addr)
		   ) {
				// Found device, open it
				int rc = libusb_open(dev, &this->usb_sdr_dev);
				if (rc < 0) {
					// Failed to open device
					SoapySDR_logf(SOAPY_SDR_ERROR, "libusb failed to open device (%d)", rc);
					this->usb_sdr_dev = nullptr;
				}
				break;
		}
	}

	// Free list, reducing device reference counts
	libusb_free_device_list(devs, 1);

	// Check handle
	if (!this->usb_sdr_dev) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to open sdr_usb_gadget");
		throw std::runtime_error("failed to open sdr_usb_gadget");
	}

	// Retrieve active config descriptor
	struct libusb_config_descriptor *config;
	int rc = libusb_get_active_config_descriptor(libusb_get_device(this->usb_sdr_dev), &config);
	if (rc < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to get usb device descriptor (%d)", rc);
		throw std::runtime_error("failed to get usb device descriptor");
	}

	// Loop through interfaces and find one with the desired name
	int interface_num = -1;
	for (int i = 0; i < config->bNumInterfaces; i++) {
		const struct libusb_interface *iface = &config->interface[i];

		for (int j = 0; j < iface->num_altsetting; j++) {
			const struct libusb_interface_descriptor *desc = &iface->altsetting[j];

			// Get the interface name
			char name[128];
			rc = libusb_get_string_descriptor_ascii(this->usb_sdr_dev, desc->iInterface, (unsigned char*)name, sizeof(name));
			if (rc < 0) {
				SoapySDR_logf(SOAPY_SDR_ERROR, "failed to get usb device interface name (%d)", rc);
				throw std::runtime_error("failed to get usb device interface name");
			}

			if (0 == strcmp(name, "sdrgadget")) {
				// Capture interface number
				interface_num = desc->bInterfaceNumber;

				// Capture endpoint addresses
				for (int k = 0; k < desc->bNumEndpoints; k++) {
					const struct libusb_endpoint_descriptor *ep_desc = &desc->endpoint[k];
					if (ep_desc->bEndpointAddress & 0x80)
					{
						this->usb_sdr_ep_in = ep_desc->bEndpointAddress;
					}
					else
					{
						this->usb_sdr_ep_out = ep_desc->bEndpointAddress;
					}
				}

				// All done
				break;
			}
		}
	}

	// Free the configuration descriptor
	libusb_free_config_descriptor(config);
	config = nullptr;

	if (interface_num < 0)
	{
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to find usb device interface");
		throw std::runtime_error("failed to find usb device interface");
	}

	// Store interface number
	this->usb_sdr_intfc_num = (uint8_t)interface_num;

	// Claim the interface
	rc = libusb_claim_interface(this->usb_sdr_dev, this->usb_sdr_intfc_num);
	if (rc < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to claim usb device interface (%d)", rc);
		throw std::runtime_error("failed to claim usb device interface");
	}
}
#endif

void SoapyPlutoSDR::open_sdr_ip_gadget(void)
{
	struct sockaddr_in addr;

	// Retrieve ip address from context
	const char *ip = iio_context_get_attr_value(ctx, "ip,ip-addr");
	if (!ip) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to retrieve IP from iio");
		throw std::runtime_error("failed to retrieve IP from iio");
	}

	// Open sockets
	this->ip_sdr_dev_control = socket(AF_INET, SOCK_DGRAM, 0);
	if (-1 == this->ip_sdr_dev_control) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to open control socket");
		throw std::runtime_error("failed to open control socket");
	}
	this->ip_sdr_dev_data = socket(AF_INET, SOCK_DGRAM, 0);
	if (-1 == this->ip_sdr_dev_data) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to open data socket");
		throw std::runtime_error("failed to open data socket");
	}

	// Bind data socket, such that we can tell the server where to direct rx data to
	memset(&addr, 0x00, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = 0;  // Let the system choose an available port
	if (bind(this->ip_sdr_dev_data, (struct sockaddr*)&addr, sizeof(addr))) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to bind data socket");
		throw std::runtime_error("failed to bind data socket");
	}

	// "Connect" to direct server (saves needing to provide the address each time)
	memset(&addr, 0x00, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(ip);
	addr.sin_port = htons(DIRECT_IP_PORT_CONTROL);
	if (connect(this->ip_sdr_dev_control, (struct sockaddr*)&addr, sizeof(addr))) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to connect control socket");
		throw std::runtime_error("failed to connect control socket");
	}
	addr.sin_port = htons(DIRECT_IP_PORT_DATA);
	if (connect(this->ip_sdr_dev_data, (struct sockaddr*)&addr, sizeof(addr))) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to connect data socket");
		throw std::runtime_error("failed to connect data socket");
	}

	// Set receive timeout on data socket
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	if (setsockopt(this->ip_sdr_dev_data, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "failed to set data socket receive timeout");
		throw std::runtime_error("failed to set data socket receive timeout");
	}
}
