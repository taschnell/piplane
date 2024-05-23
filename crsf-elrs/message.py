#!/usr/bin/env python3
import serial
import time
import argparse
from enum import IntEnum

CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A


def crc8_dvb_s2(crc, a) -> int:
    crc = crc ^ a
    for ii in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
    return crc & 0xFF


def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc


def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]


def signed_byte(b):
    return b - 256 if b >= 128 else b


def interpret_rc_channels(byte_data):
    """
    Handels byte Data from Rc_Channels_Packed
    """
    num = int.from_bytes(byte_data, byteorder="little")
    crsf_channel_values = []
    for i in range(16):
        shift_amount = 11 * i
        int_value = (num >> shift_amount) & 0x7FF
        crsf_channel_values.append(convert_number(int_value))
    return (crsf_channel_values)

def convert_number(input_number) -> int:
    """
    Converts CRSF_Channel int value to us values 
        points = [(172, 988), (992, 1500), (1811, 2012)]
    """
    if not 0 <= input_number <= 1984:
        raise ValueError("Input number must be in the range 0-1984.")

    # Define known input-output pairs
    points = [(172, 988), (992, 1500), (1811, 2012)]

    # Perform linear interpolation
    if input_number <= points[0][0]:
        # Extrapolate below the first point
        x0, y0 = 0, 0
        x1, y1 = points[0]
    elif input_number >= points[-1][0]:
        # Extrapolate above the last point
        x0, y0 = points[-1]
        x1, y1 = 1984, 2012 + (2012 - points[-2][1])
    else:
        # Interpolate between points
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            if x0 <= input_number <= x1:
                break

    # Calculate the interpolated value
    if x0 == x1:
        return y0  # Avoid division by zero if points are the same
    else:
        output_number = y0 + (input_number - x0) * (y1 - y0) / (x1 - x0)
        return int(output_number)


def channelsCrsfToChannelsPacket(channels) -> bytes:
    result = bytearray(
        [CRSF_SYNC, 24, PacketsTypes.RC_CHANNELS_PACKED]
    )  # 24 is packet length
    result += interpret_rc_channels(channels)
    result.append(crc8_data(result[2:]))
    return result


def handleCrsfPacket(ptype, data):
    if ptype == PacketsTypes.RADIO_ID and data[5] == 0x10:
        # print(f"OTX sync")
        pass
    elif ptype == PacketsTypes.LINK_STATISTICS:
        rssi1 = signed_byte(data[3])
        rssi2 = signed_byte(data[4])
        lq = data[5]
        snr = signed_byte(data[6])
        antenna = data[7]
        mode = data[8]
        power = data[9]
        # telemetry strength
        downlink_rssi = signed_byte(data[10])
        downlink_lq = data[11]
        downlink_snr = signed_byte(data[12])
        print(
            f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03} mode={mode}"
        )  # ant={antenna} snr={snr} power={power} drssi={downlink_rssi} dlq={downlink_lq} dsnr={downlink_snr}")
    elif ptype == PacketsTypes.ATTITUDE:
        pitch = int.from_bytes(data[3:5], byteorder="big", signed=True) / 10000.0
        roll = int.from_bytes(data[5:7], byteorder="big", signed=True) / 10000.0
        yaw = int.from_bytes(data[7:9], byteorder="big", signed=True) / 10000.0
        print(f"Attitude: Pitch={pitch:0.2f} Roll={roll:0.2f} Yaw={yaw:0.2f} (rad)")
    elif ptype == PacketsTypes.FLIGHT_MODE:
        packet = "".join(map(chr, data[3:-2]))
        print(f"Flight Mode: {packet}")
    elif ptype == PacketsTypes.BATTERY_SENSOR:
        vbat = int.from_bytes(data[3:5], byteorder="big", signed=True) / 10.0
        curr = int.from_bytes(data[5:7], byteorder="big", signed=True) / 10.0
        mah = data[7] << 16 | data[8] << 7 | data[9]
        pct = data[10]
        print(f"Battery: {vbat:0.2f}V {curr:0.1f}A {mah}mAh {pct}%")
    elif ptype == PacketsTypes.BARO_ALT:
        print(f"BaroAlt: ")
    elif ptype == PacketsTypes.DEVICE_INFO:
        packet = " ".join(map(hex, data))
        print(f"Device Info: {packet}")
    elif data[2] == PacketsTypes.GPS:
        lat = int.from_bytes(data[3:7], byteorder="big", signed=True) / 1e7
        lon = int.from_bytes(data[7:11], byteorder="big", signed=True) / 1e7
        gspd = int.from_bytes(data[11:13], byteorder="big", signed=True) / 36.0
        hdg = int.from_bytes(data[13:15], byteorder="big", signed=True) / 100.0
        alt = int.from_bytes(data[15:17], byteorder="big", signed=True) - 1000
        sats = data[17]
        print(
            f"GPS: Pos={lat} {lon} GSpd={gspd:0.1f}m/s Hdg={hdg:0.1f} Alt={alt}m Sats={sats}"
        )
    elif ptype == PacketsTypes.VARIO:
        vspd = int.from_bytes(data[3:5], byteorder="big", signed=True) / 10.0
        print(f"VSpd: {vspd:0.1f}m/s")
    elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
        #print(f"Channels: {len(interpret_rc_channels(data[3:25]))}")
        return interpret_rc_channels(data[3:25])
    else:
        packet = " ".join(map(hex, data))
        print(f"Unknown 0x{ptype:02x}: {packet}")


parser = argparse.ArgumentParser()
parser.add_argument("-P", "--port", default="/dev/ttyAMA0", required=False)
parser.add_argument("-b", "--baud", default=420000, required=False)
parser.add_argument(
    "-t",
    "--tx",
    required=False,
    default=False,
    action="store_true",
    help="Enable sending CHANNELS_PACKED every 20ms (all channels 1500us)",
)
args = parser.parse_args()

with serial.Serial(args.port, args.baud, timeout=2) as ser:
    input = bytearray()
    counter = 0
    start = time.time()
    try:
        while True:
            if ser.in_waiting > 0:
                input.extend(ser.read(ser.in_waiting))
            else:
                if args.tx:
                    ser.write(channelsCrsfToChannelsPacket([992 for ch in range(16)]))
                # time.sleep(0.002)

            if len(input) > 2:
                # This simple parser works with malformed CRSF streams
                # it does not check the first byte for SYNC_BYTE, but
                # instead just looks for anything where the packet length
                # is 4-64 bytes, and the CRC validates
                expected_len = input[1] + 2
                if expected_len > 64 or expected_len < 4:
                    input = []
                elif len(input) >= expected_len:
                    single = input[:expected_len]  # copy out this whole packet
                    input = input[expected_len:]  # and remove it from the buffer
                    if not crsf_validate_frame(single):  # single[-1] != crc:
                        packet = " ".join(map(hex, single))
                        print(f"crc error: {packet}")
                    else:
                        int_list = handleCrsfPacket(single[2], single)
                        counter +=1
                        print(int_list)
    except:
        print(time.time()-start)
        print(counter/(time.time()-start))
