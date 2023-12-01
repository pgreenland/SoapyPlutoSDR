#include <cstdint>

#include <SoapySDR/Logger.hpp>
#include <iio.h>

namespace SoapyPlutoSDR_TimestampEvery
{
    inline void update_device_timestamp_every(const iio_device *dev, uint32_t value, size_t channel_count)
    {
        // Read current ADC GPIO output value, which controls decimator and ADC timestamp setting
        uint32_t temp_reg_val;
        if (iio_device_reg_read((iio_device*)dev, 0x800000BC, &temp_reg_val) < 0) {
            // Failed to read GPIO register
            SoapySDR_logf(SOAPY_SDR_ERROR, "failed to read timestamp_every setting");
            throw std::runtime_error("failed to read timestamp_every setting");
        }

        // ADC/DAC data bus in the pluto is 64-bit wide. With I/Q channels enabled for one channel
        // each sample is 2 * 16-bit. Therefore 2 samples will be carried in each 64-bit value.
        // Timestamps are applied every x samples on the 64-bit bus. Therefore we divide the
        // requested timestamp every value by 2.
        // If four channels are enabled, each sample is 4 x 16-bit. Therefore one sample will be carried
        // in each 64-bit value. We do not need to divide the requested timestamp value.
        if (channel_count == 2)
        {
            value /= 2;
        }

        // The 32-bit GPIO register is divided into timestamp_every [31:1] and decimator/interpolator enable [0:0]
        // therefore we mask off the old bits and or in the new bits, shifting them into place.
        temp_reg_val &= 0x1; // Mask off all but lowest bit
        temp_reg_val |= (value << 1);

        // Write new ADC GPIO output value.
        if (iio_device_reg_write((iio_device*)dev, 0x800000BC, temp_reg_val) < 0) {
            // Failed to write GPIO register
            SoapySDR_logf(SOAPY_SDR_ERROR, "failed to write timestamp_every setting");
            throw std::runtime_error("failed to write timestamp_every setting");
        }
    }
}
