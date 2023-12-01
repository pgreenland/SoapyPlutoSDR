#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h> //printf
#include <stdlib.h> //free
#include <stdint.h>

static void check_channel(size_t sample_count, size_t channel_count,
                          size_t channel_index,
                          long long *tx_timestamps, long long *rx_timestamps,
                          uint16_t** rx_buff,
                          size_t rx_mtu);
static long long extract_timestamp(uint16_t *rx_buff, size_t buff_size, size_t *inbound_ts_offset);

int main(void)
{
    //create device instance
    //args can be user defined or from the enumeration result
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    if (0) {
        SoapySDRKwargs_set(&args, "uri", "usb:");
    } else {
        SoapySDRKwargs_set(&args, "uri", "ip:pluto");
    }
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "1");
    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL)
    {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    //apply settings
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, 1.92e6) != 0)
    {
        printf("setSampleRate rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, 800e6, NULL) != 0)
    {
        printf("setFrequency rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, 1.92e6) != 0)
    {
        printf("setSampleRate tx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, 800e6, NULL) != 0)
    {
        printf("setFrequency tx fail: %s\n", SoapySDRDevice_lastError());
    }

    //setup streams
    size_t channels[] = {0, 1}; // {0} or {0, 1}
    size_t channel_count = sizeof(channels) / sizeof(channels[0]);
    SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (rxStream == NULL)
    {
        printf("setupStream rx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }
    SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (txStream == NULL)
    {
        printf("setupStream tx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    //get mtus
    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("MTU - TX: %lu, RX: %lu\n", tx_mtu, rx_mtu);

    //tx buffer could be made a lot shorter, the library *should* fill in the blank space
    tx_mtu = 6; // IQ samples (12 words)

    //sample count
    size_t sample_count = 30000;

    //create buffers for samples (unsigned signed int16's - although we're transmitting and receiving signed numbers)
    //double size for I and Q samples
    uint16_t* rx_buff[sample_count][channel_count];
    for (size_t i = 0; i < sample_count; i++)
    for (size_t j = 0; j < channel_count; j++)
    {
        rx_buff[i][j] = malloc(sizeof(uint16_t)*2*rx_mtu);
    }
    long long rx_timestamps[sample_count];
    uint16_t tx_buff[2*tx_mtu];
    long long tx_timestamps[sample_count];

    //prepare fixed bytes in transmit buffer
    //we transmit a pattern of FFFF FFFF [TS_0]00 [TS_1]00 [TS_2]00 [TS_3]00 [TS_4]00 [TS_5]00 [TS_6]00 [TS_7]00 FFFF FFFF
    //that is a flag (FFFF FFFF) followed by the 64 bit timestamp, split into 8 bytes and packed into the lsb of each of the DAC words.
    //DAC samples are left aligned 12-bits, so each byte is left shifted into place
    for(size_t i = 0; i < 2; i++)
    {
        tx_buff[0 + i] = 0xffff;
        // 8 x timestamp words
        tx_buff[10 + i] = 0xffff;
    }

    //activate streams
    SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0); //start streaming
    SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0); //start streaming

    //here goes
    printf("Start test...\n");

    //ensure buffers in device are empty
    for (size_t buffers_read = 0; buffers_read < 128; /* in loop */)
    {
        void *buffs[] = {rx_buff[0][0], rx_buff[0][1]}; //array of buffers
        int flags; //flags set by receive operation
        long long timeNs; //timestamp for receive buffer

        // Read samples
        int sr = SoapySDRDevice_readStream(sdr, rxStream, buffs, rx_mtu, &flags, &timeNs, 100000); // 100ms timeout
        if (sr < 0)
        {
            // Skip read on error (likely timeout)
            continue;
        }

        // Increment number of buffers read
        buffers_read++;
    }

    long long last_time = 0;
    for (size_t buffers_read = 0; buffers_read < sample_count; buffers_read++)
    {
        void *buffs[2]; //array of buffers
        int flags; //flags set by receive operation
        long long timeNs; //timestamp for receive buffer

        buffs[0] = rx_buff[buffers_read][0];
        buffs[1] = rx_buff[buffers_read][1];
        int sr = SoapySDRDevice_readStream(sdr, rxStream, buffs, rx_mtu, &flags, &timeNs, 100000);
        if (sr < 0)
        {
            // Skip read on error (likely timeout)
            continue;
        }
        rx_timestamps[buffers_read] = timeNs;

        // Dump info
        printf("Buffer: %lu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n", buffers_read, sr, flags, timeNs, timeNs - last_time);
        last_time = timeNs;

        // Calculate transmit time 4ms in future
        long long tx_time = timeNs + (4 * 1000 * 1000);

        // Push transmit time into queue
        tx_timestamps[buffers_read] = tx_time;

        // Set samples
        for(size_t i = 0; i < 8; i++)
        {
            // Extract byte from tx time
            uint8_t tx_time_byte = (tx_time >> (i * 8)) & 0xff;

            // Add byte to buffer
            tx_buff[2 + i] = tx_time_byte << 4;
        }

        // Send buffer
        buffs[0] = tx_buff;
        buffs[1] = tx_buff;
        flags = SOAPY_SDR_HAS_TIME;
        int st = SoapySDRDevice_writeStream(sdr, txStream, (const void * const*)buffs, tx_mtu, &flags, tx_time, 100000);
        if ((size_t)st != tx_mtu)
        {
            printf("TX Failed: %i\n", st);
        }
    }

    //stop streaming
    SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);

    //shutdown the stream
    SoapySDRDevice_closeStream(sdr, rxStream);
    SoapySDRDevice_closeStream(sdr, txStream);

    //cleanup device handle
    SoapySDRDevice_unmake(sdr);

    // Process each rx buffer, looking for transmitted timestamp
    for (size_t j = 0; j < channel_count; j++)
    {
        printf("Checking channel %zu\n", j);
        check_channel(sample_count, channel_count, j, tx_timestamps, rx_timestamps, (uint16_t**)rx_buff, rx_mtu);
    }

    //all done
    printf("test complete!\n");

    //free buffers
    for (size_t i = 0; i < sample_count; i++)
    for (size_t j = 0; j < channel_count; j++)
    {
        free(rx_buff[i][j]);
        rx_buff[i][j] = NULL;
    }

    return EXIT_SUCCESS;
}

static void check_channel(size_t sample_count, size_t channel_count,
                          size_t channel_index,
                          long long *tx_timestamps, long long *rx_timestamps,
                          uint16_t** rx_buff,
                          size_t rx_mtu)
{
    // Process each rx buffer, looking for transmitted timestamp
    size_t last_ts_index = 0;
    size_t last_buff_index = 0;
    for (size_t index = 0; index < sample_count; index++)
    {
        // Split buffer and timestamp
        long long curr_rx_buffer_ts = rx_timestamps[index];
        uint16_t* curr_rx_buffer_data = rx_buff[(index * channel_count) + channel_index];

        // Search for timestamp
        size_t inbound_ts_offset;
        long long inbound_ts_value = extract_timestamp(curr_rx_buffer_data, rx_mtu, &inbound_ts_offset);

        // Check timestamp
        if (inbound_ts_value > 0)
        {
            // Found timestamp, search tx timestamp queue
            size_t tx_times_index;
            for (tx_times_index = last_ts_index; tx_times_index < sample_count; tx_times_index++)
            {
                if (tx_timestamps[tx_times_index] == inbound_ts_value)
                {
                    /* Found entry */
                    break;
                }
            }

            if (tx_times_index < sample_count)
            {
                // Found TX entry
                printf("Buffer %lu contains TX TS after %lu buffers, TX TS queue index %lu, buffer word index %lu\n", index, index - last_buff_index, tx_times_index, inbound_ts_offset);
                if (curr_rx_buffer_ts != inbound_ts_value)
                {
                    printf("RX timestamp doesn't match TX timestamp....it should\n");
                }

                // Update last indexes
                last_buff_index = index;
                last_ts_index = tx_times_index;
            }
            else
            {
                // TX entry not found
                printf("Buffer %lu contains a TX timestamp which isn't in TX TS the queue\n", index);
            }
        }
        else
        {
            // Buffer doesn't contain timestamp
            printf("Buffer %lu doesn't contain timestamp\n", index);
        }
    }
}

//declare state machine states
typedef enum
{
    HEADER = 0,
    VALUE,
    FOOTER,
    DONE

} States;

static long long extract_timestamp(uint16_t *rx_buff, size_t buff_size, size_t *inbound_ts_offset)
{
    // Reset extracted timestamp and offset of timestamp within buffer
    long long extracted_value = 0;
    size_t extracted_offset = 0;

    // Search for timestamp in current buffer
    States state = HEADER;
    int count = 0;
    for (size_t i = 0; i < buff_size && (state != DONE); i++)
    {
        switch (state)
        {
            case HEADER:
            {
                // Looking for header words
                if (rx_buff[i] == 0xffff)
                {
                    // Found word
                    count++;
                }
                else
                {
                    // Reset count
                    count = 0;
                }

                if (count == 2)
                {
                    // Found both words, advance state
                    state = VALUE;
                    count = 0;
                }
                break;
            }
            case VALUE:
            {
                // Extract value
                extracted_value |= (unsigned long long)rx_buff[i] << (count * 8);
                if (count == 0)
                {
                    // Capture word offset of data
                    extracted_offset = i;
                }

                count++;

                if (count == 8)
                {
                    // Found all value bytes, advance state
                    state = FOOTER;
                    count = 0;
                }
                break;
            }
            case FOOTER:
            {
                // Looking for footer words
                if (rx_buff[i] == 0xffff)
                {
                    // Found word
                    count++;
                }
                else
                {
                    // Uh oh, shouldn't happen, too lazy to handle
                    count = 0;
                }

                if (count == 2)
                {
                    // Found both words, all done
                    state = DONE;
                }
                break;
            }
            case DONE:
            default:
            {
                /* Do nothing */
                break;
            }
        }
    }

    if (state != DONE)
    {
        // Failed to found timestamp in input buffer
        return 0;
    }

    // Return timestamp and word offset
    *inbound_ts_offset = extracted_offset;
    return extracted_value;
}