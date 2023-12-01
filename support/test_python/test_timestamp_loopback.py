"""
This script places the Pluto into digital loopback mode.
Before receiving and transmitting several buffers worth of data.
For each reception a transmission is scheduled 4ms in the future, with the transmission timestamp embedded in the datastream.
Calculated transmission timestamp and received buffered are captured for analysis after the stream completes.
Received buffers and transmitted timestamps are compared, ensuring timestamps are present in the correct buffers.

Note: For pluto may need to override its reference clock to ensure the timings below are calculated correctly:
Login via SSH / serial.
Run: fw_setenv ad936x_ext_refclk_override "<40000000>"
Reboot.

To reset:
Login via SSH / serial.
Run: fw_setenv ad936x_ext_refclk_override
Reboot.

To run script having installed in an alternative location:
export SRSRAN_INSTALL=${HOME}/srsRAN
LD_LIBRARY_PATH=${SRSRAN_INSTALL}/lib PYTHONPATH=${SRSRAN_INSTALL}/lib/python3.8/site-packages python3 test_timestamp_loopback.py
"""

import SoapySDR
from SoapySDR import * # SOAPY_SDR_ constants
import numpy # Use numpy for buffers
import enum

def main():
    """Perform timestamp test"""

    # Prepare device instance arguments
    if 0:
        args = dict(driver="plutosdr", uri="usb:", direct="1", timestamp_every="1920", loopback="1")
    else:
        args = dict(driver="plutosdr", uri="ip:pluto", direct="1", udp_packet_size="1472", timestamp_every="1920", loopback="1")

    # Enumerate devices using arguments as a search string
    results = SoapySDR.Device.enumerate(args)
    for result in results:
        print(f"Found device: {result}")

    # Create device
    sdr = SoapySDR.Device(args)

    # Apply settings
    sdr.setSampleRate(SOAPY_SDR_RX, 0, 1.92e6)
    sdr.setFrequency(SOAPY_SDR_RX, 0, 800.0e6)
    sdr.setSampleRate(SOAPY_SDR_TX, 0, 1.92e6)
    sdr.setFrequency(SOAPY_SDR_TX, 0, 800.0e6)

    # Setup a stream (signed int16's)
    channels = [0, 1] # [0] or [0, 1]
    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels)
    txStream = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels)

    # Get stream MTUs
    rx_mtu = int(sdr.getStreamMTU(rxStream))
    tx_mtu = int(sdr.getStreamMTU(txStream))
    print("MTU - TX: {}, RX: {}".format(tx_mtu, rx_mtu))
    if (tx_mtu != rx_mtu):
        raise Exception("MTU mismatch")

    # TX buffer could be made a lot shorter, the library *should* fill in the blank space
    tx_mtu = 6 # IQ samples (12 words)

    # Create a re-usable buffers for samples (unsigned signed int16's - although we're transmitting and receiving signed numbers)
    # double size for I and Q samples
    rx_buff0 = numpy.array([0]*2*rx_mtu, numpy.ushort)
    rx_buff1 = numpy.array([0]*2*rx_mtu, numpy.ushort)
    tx_buff = numpy.array([0]*2*tx_mtu, numpy.ushort)

    # Prepare fixed bytes in transmit buffer
    # We transmit a pattern of FFFF FFFF [TS_0]00 [TS_1]00 [TS_2]00 [TS_3]00 [TS_4]00 [TS_5]00 [TS_6]00 [TS_7]00 FFFF FFFF
    # that is a flag (FFFF FFFF) followed by the 64 bit timestamp, split into 8 bytes and packed into the lsb of each of the DAC words.
    # DAC samples are left aligned 12-bits, so each byte is left shifted into place
    for i in range(2):
        tx_buff[0 + i] = 0xffff
        # 8 x timestamp words
        tx_buff[10 + i] = 0xffff

    # Start streaming
    sdr.activateStream(rxStream)
    sdr.activateStream(txStream)

    # Open file
    f = open("rx_dump.txt", "w")

    # Here goes nothin'
    print("Start test...")

    # Ensure buffers in device are empty
    buffers_read = 0
    while buffers_read < 16:
        # Read samples
        sr = sdr.readStream(rxStream, [rx_buff0, rx_buff1], rx_mtu, timeoutUs=(100 * 1000)) # 100ms timeout
        if sr.ret < 0:
            # Skip read on error (likely timeout)
            continue

        # Increment number of buffers read
        buffers_read += 1

    # Receive some samples
    last_time = 0
    rx_buffers = []
    tx_times = []
    buffers_read = 0
    while buffers_read < 5000:
        # Read samples
        sr = sdr.readStream(rxStream, [rx_buff0, rx_buff1], rx_mtu, timeoutUs=(100 * 1000)) # 100ms timeout
        if sr.ret < 0:
            # Skip read on error (likely timeout)
            continue

        # Report samples
        print(f"Buffer: {buffers_read} - Samples: {sr.ret}, Flags: {sr.flags}, Time: {sr.timeNs}, TimeDiff: {sr.timeNs - last_time}")
        last_time = sr.timeNs

        # Increment number of buffers read
        buffers_read += 1

        # Push current rx timestamp and buffer into queue
        rx_buffers.append((sr.timeNs, [numpy.array(rx_buff0, copy=True), numpy.array(rx_buff1, copy=True)]))

        # Calculate transmit time 4ms in future
        tx_time = sr.timeNs + (4 * 1000 * 1000)

        # Push transmit time into queue
        tx_times.append(tx_time)

        # Set samples
        tx_time_hex = ""
        for i in range(8):
            # Extract byte from tx time
            tx_time_byte = (tx_time >> (i * 8)) & 0xff

            # Add byte to buffer
            tx_buff[2 + i] = tx_time_byte << 4

            # Convert byte to hex
            tx_time_hex += f"{tx_time_byte:02x}"

        # Send buffer
        res = sdr.writeStream(txStream, [tx_buff, tx_buff], tx_mtu, SOAPY_SDR_HAS_TIME, tx_time, timeoutUs=(100 * 1000)) # 100ms timeout
        if res.ret != tx_mtu:
            raise Exception('transmit failed %s' % str(res))

        # Dump samples in hex, with 64 bytes per line
        f.write(f"RX timeNs (DEC):{sr.timeNs}\n")
        f.write(f"TX timeNs (DEC):{tx_time}\n".format())
        f.write(f"TX timeNs (HEX):{tx_time_hex}\n")
        hex_str = rx_buff0.tobytes().hex('\n', 64)
        f.write(f"RX Data[0]:\n{hex_str}\n")
        hex_str = rx_buff1.tobytes().hex('\n', 64)
        f.write(f"RX Data[1]:\n{hex_str}\n")

    # Close file
    f.close()

    # Stop streaming
    sdr.deactivateStream(txStream)
    sdr.deactivateStream(rxStream)

    # Shutdown the stream
    sdr.closeStream(txStream)
    sdr.closeStream(rxStream)

    # Process each rx buffer, looking for transmitted timestamp
    for i in range(len(channels)):
        print(f"Checking channel {i}")
        check_channel(i, rx_buffers, tx_times)

    # All done
    print("test complete!")

def check_channel(channel_index, rx_buffers, tx_times):
    """Check samples for single channel"""

    last_index = 0
    for index in range(len(rx_buffers)):
        # Split buffer and timestamp
        curr_rx_buffer_ts, curr_rx_buffer_data = rx_buffers[index]

        # Select channel
        curr_rx_buffer_data = curr_rx_buffer_data[channel_index]

        # Search for timestamp
        inbound_ts_value, inbound_ts_offset = extract_timestamp(curr_rx_buffer_data)

        # Check timestamp
        if inbound_ts_value is not None:
            # Found timestamp, search tx timestamp queue
            try:
                tx_times_index = tx_times.index(inbound_ts_value)
            except ValueError:
                print(f"Inbound timestamp {inbound_ts_value} not found in outbound list")
                tx_times_index = -1

            if -1 != tx_times_index:
                # Found TX entry
                print(f"Buffer {index} contains TX TS after {index - last_index} buffers, TX TS queue index {tx_times_index}, buffer word index {inbound_ts_offset}")
                if (curr_rx_buffer_ts != inbound_ts_value):
                    print("RX timestamp doesn't match TX timestamp....it should")

                # Update last offset
                last_index = index
            else:
                # TX entry not found, search for closest match
                closest_tx_ts = min(tx_times, key=lambda x:abs(x-inbound_ts_value))
                print(f"Buffer {index} contains a TX timestamp which isn't in TX TS the queue, closest tx ts is {closest_tx_ts} away")
        else:
            # Buffer doesn't contain timestamp
            print(f"Buffer {index} doesn't contain timestamp")

# Declare state machine states
class States(enum.Enum):
    HEADER = enum.auto()
    VALUE = enum.auto()
    FOOTER = enum.auto()
    DONE = enum.auto()

# Extract timestamp from buffer
def extract_timestamp(rx_buff):
    """Extract timestamp from rx buffer, embedded in tx butter"""

    # Reset extracted timestamp and offset of timestamp within buffer
    extracted_value = 0
    extracted_offset = 0

    # Search for timestamp in current buffer
    state = States.HEADER
    count = 0
    for i in range(len(rx_buff)):
        if state == States.HEADER:
            # Looking for header words
            if rx_buff[i] == 0xffff:
                # Found word
                count += 1
            else:
                # Reset count
                count = 0

            if count == 2:
                # Found both words, advance state
                state = States.VALUE
                count = 0

        elif state == States.VALUE:
            # Extract value
            extracted_value |= int(rx_buff[i]) << (count * 8)

            if count == 0:
                # Capture word offset of data
                extracted_offset = i

            count += 1

            if count == 8:
                # Found all value bytes, advance state
                state = States.FOOTER
                count = 0

        elif state == States.FOOTER:
            # Looking for footer words
            if rx_buff[i] == 0xffff:
                # Found word
                count += 1
            else:
                # Uh oh, shouldn't happen, too lazy to handle
                break

            if count == 2:
                # Found both words, all done
                state = States.DONE
                break

    if state != States.DONE:
        # Failed to found timestamp in input buffer
        return (None, None)

    # Return timestamp and word offset
    return (extracted_value, extracted_offset)

if __name__ == "__main__":
    main()