import logging

logger = logging.getLogger(__name__)

import io
import queue
import string
import sys
import threading
import time
from enum import Enum, IntFlag, IntEnum, unique

import serial

# class for communicating with u-blox GPS receivers
# supports UBX, NMEA, and RTCM3 protocols

# this module (mostly) supports every protocol version released
# u-blox 5 series - protocol version 10.00 - 12.02
# u-blox 6 series - protocol version 12.00 - 13.03
# u-blox M6 series (GPS+GLONASS+QZSS) - protocol version 14.00
# ref: u-blox document GPS.G6-SW-10018
# u-blox 7 series - protocol version 14.00
# ref: u-blox document GPS.G7-SW-12001-B1 - https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
# u-blox 8 series - protocol version 15.00 - 23.01
# ref: u-blox document UBX-13003221 - https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
# u-blox 9 series - protocol version 32.01
# ref: u-blox document UBX-21022436 - https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
# u-blox 10 series - protocol version 34.10
# ref: u-blox document UBX-21035062 - https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf

# only tested with hardware I have access to, which is currently:
# MAX-M8Q
# SAM-M8Q
# MAX-7Q

class UbxCmd:

    @unique
    class PORT(IntEnum):
        I2C = 0
        UART1 = 1
        UART2 = 2
        USB = 3
        SPI = 4

    @unique
    class INOUT_PROTOCOL(IntFlag):
        NONE = 0
        UBX = 1<<0
        NMEA = 1<<1
        RTCM = 1<<2
        RTCM3 = 1<<5
        ALL = UBX|NMEA|RTCM|RTCM3

    @unique
    class GNSS_ID_AUGMENT(IntEnum):
        SBAS = 1
        IMES = 4
        QZSS = 5

    @unique
    class GNSS_ID_MAJOR(IntEnum):
        GPS = 0
        Galileo = 2
        BeiDou = 3
        GLONASS = 6

    @unique
    class SBAS_PRN(IntFlag):
        NONE = 0
        PRN120 = 1<<0
        PRN121 = 1<<1
        PRN122 = 1<<2
        PRN123 = 1<<3
        PRN124 = 1<<4
        PRN125 = 1<<5
        PRN126 = 1<<6
        PRN127 = 1<<7
        PRN128 = 1<<8
        PRN129 = 1<<9
        PRN130 = 1<<10
        PRN131 = 1<<11
        PRN132 = 1<<12
        PRN133 = 1<<13
        PRN134 = 1<<14
        PRN135 = 1<<15
        PRN136 = 1<<16
        PRN137 = 1<<17
        PRN138 = 1<<18
        PRN139 = 1<<19
        PRN140 = 1<<20
        PRN141 = 1<<21
        PRN142 = 1<<22
        PRN143 = 1<<23
        PRN144 = 1<<24
        PRN145 = 1<<25
        PRN146 = 1<<26
        PRN147 = 1<<27
        PRN148 = 1<<28
        PRN149 = 1<<29
        PRN150 = 1<<30
        PRN151 = 1<<31
        PRN152 = 1<<32
        PRN153 = 1<<33
        PRN154 = 1<<34
        PRN155 = 1<<35
        PRN156 = 1<<36
        PRN157 = 1<<37
        PRN158 = 1<<38
        ALL = PRN120|PRN121|PRN122|PRN123|PRN124|PRN125|PRN126|PRN127|\
              PRN128|PRN129|PRN130|PRN131|PRN132|PRN133|PRN134|PRN135|\
              PRN136|PRN137|PRN138|PRN139|PRN140|PRN141|PRN142|PRN143|\
              PRN144|PRN145|PRN146|PRN147|PRN148|PRN149|PRN150|PRN151|\
              PRN152|PRN153|PRN154|PRN155|PRN156|PRN157|PRN158

    def thread_tx(self) -> None:
    # thread that handles transmitting data frames to receiver (or file)
        time.sleep(0.001) # yield to make sure all threads have started

        while True:
            while self.write == False or self.stream == None:
                time.sleep(0.2)

            # wait forever for new data
            tx_item = self.tx_queue.get(block=True, timeout=None)

            if tx_item == None: # canary value to break out of infinite loop
                self.tx_queue.task_done()
                break

            self.stream.write(tx_item)
            self.tx_queue.task_done()
            if self.read == True:
                time.sleep(0.001) # yield to read threads after TX completes

    def thread_rx(self) -> None:
    # thread that handles receiving data frames from receiver (or file)
        time.sleep(0.001) # yield to make sure all threads have started

        # handles data input by stuffing into buffers, and doing minimal parsing to look for the end of the transmission before sending to queue to be fully parsed in another thread
        class rx_states(Enum):
            BEGIN = 1
            RX_NMEA = 2
            RX_UBX = 3
            RX_RTCM3 = 4

        RX_BUFF_SIZE = 128
        rx_buff = bytearray(RX_BUFF_SIZE)
        rx_buff_pos = 0
        rx_state = rx_states.BEGIN

        NMEA_BUFF_SIZE = 256 # usually NMEA sentences are max 82 chars, but ubx receivers can go longer if UBX-CFG-NMEA flags limit82 is unset (default)
        nmea_buff = bytearray(NMEA_BUFF_SIZE)
        nmea_buff_pos = 0

        UBX_BUFF_SIZE = 2 + 6 + 0xFFFF + 2 # max possible size for a UBX message µB + header + data + checksum
        ubx_buff = bytearray(UBX_BUFF_SIZE)
        ubx_buff_pos = 0

        RTCM3_BUFF_SIZE = 3 + 1023 + 3 # max possible size for a RTCM3 message header + data + checksum
        rtcm3_buff = bytearray(RTCM3_BUFF_SIZE)
        rtcm3_buff_pos = 0

        while True:
            while self.read == False or self.stream == None:
                time.sleep(0.2)

            try: rx_buff = self.stream.read(RX_BUFF_SIZE)
            except BlockingIOError: rx_count = 0
            else: rx_count = len(rx_buff)
            if rx_count == 0:
                continue

            rx_buff_pos = 0
            while rx_buff_pos < rx_count:
                if rx_state == rx_states.BEGIN:
                    if rx_buff[rx_buff_pos] == 0xB5: # ISO8859.1 for µ
                        rx_state = rx_states.RX_UBX
                        ubx_buff_pos = 0
                    elif rx_buff[rx_buff_pos] == 0x24: # ASCII for $
                        rx_state = rx_states.RX_NMEA
                        nmea_buff_pos = 0
                    # FIXME - add support for receiving RTCM3
                    # I don't have a high-precision receiver to test this, or access to the specification
                    # ref: https://www.ucalgary.ca/engo_webdocs/GL/06.20236.MinminLin.pdf
                    # ref: https://portal.u-blox.com/s/question/0D52p00009IthhBCAR/ublox-rtcm-wrapper-specification
                    elif rx_buff[rx_buff_pos] == 0xD3:
                        rx_state = rx_states.RX_RTCM3
                        rtcm3_buff_pos = 0

                if rx_state == rx_states.RX_NMEA:
                    nmea_buff[nmea_buff_pos] = rx_buff[rx_buff_pos]

                    # NMEA sentences start with $ and end with CRLF
                    if nmea_buff[nmea_buff_pos] == 0x0A and nmea_buff[nmea_buff_pos - 1] == 0x0D and nmea_buff[0] == 0x24:
                        self.rx_queue.put_nowait(bytes(nmea_buff[:nmea_buff_pos+1]))
                        nmea_buff_pos = 0
                        rx_state = rx_states.BEGIN
                    else:
                        nmea_buff_pos += 1
                        if nmea_buff_pos >= NMEA_BUFF_SIZE:
                            # either sentence is longer than buffer size, or we missed the \r\n
                            nmea_buff_pos = 0
                            rx_state = rx_states.BEGIN

                if rx_state == rx_states.RX_UBX:
                    ubx_buff[ubx_buff_pos] = rx_buff[rx_buff_pos]
                    ubx_msg_len = 0xFFFF
                    if ubx_buff_pos >= 5:
                        ubx_msg_len = int.from_bytes(ubx_buff[4:5], byteorder='little', signed=False)

                    # UBX messages start with µb, have a 6-byte header that contains the length, and a 2-byte checksum after length bytes
                    if ubx_buff_pos >= (2 + 6 + ubx_msg_len + 2) and ubx_buff[0] == 0xB5 and ubx_buff[1] == 0x62:
                        self.rx_queue.put_nowait(bytes(ubx_buff[:2 + 4 + ubx_msg_len + 2]))
                        ubx_buff_pos = 0
                        rx_state = rx_states.BEGIN
                    else:
                        ubx_buff_pos += 1

                if rx_state == rx_states.RX_RTCM3:
                    # FIXME - add support for receiving RTCM3
                    # I don't have a high-precision receiver to test this, or access to the specification
                    # ref: https://www.ucalgary.ca/engo_webdocs/GL/06.20236.MinminLin.pdf
                    # ref: https://portal.u-blox.com/s/question/0D52p00009IthhBCAR/ublox-rtcm-wrapper-specification
                    rtcm3_buff[rtcm3_buff_pos] = rx_buff[rx_buff_pos]
                    rtcm3_msg_len = 1023
                    if rtcm3_buff_pos >= 3:
                        rtcm3_msg_len = int.from_bytes(rtcm3_buff[2:3], byteorder='little', signed=False) & 0x03FF

                    # RTCM3 messages start with 0xD3, 6 bits reserved (set to 0), 10 bits length, and 3 byte checksum (CRC24Q)
                    if rtcm3_buff_pos >= (3 + rtcm3_msg_len + 3) and rtcm3_buff[0] == 0xD3 and (rtcm3_buff[1] & 0xFC) == 0:
                        self.rx_queue.put_nowait(bytes(ubx_buff[:3 + rtcm3_msg_len + 3]))
                        rtcm3_buff_pos = 0
                        rx_state = rx_states.BEGIN
                    else:
                        rtcm3_buff_pos += 1

                rx_buff_pos += 1

    def thread_parse(self) -> None:
    # thread that handles parsing of received data into NMEA, UBX or RTCM3 formats, verifying checksums, etc
        while True:
            time.sleep(0.001) # yield to read and application threads every cycle

            rx_data = self.rx_queue.get(block=True, timeout=None)

            # canary value to break out of infinite loop
            if rx_data == None:
                self.rx_queue.task_done()
                break

            rx_protocol = None

            if rx_data[0] == 0x24: # ASCII for $
                rx_protocol = self.INOUT_PROTOCOL.NMEA
                nmea_checksum = 0
                checksum_pos = 0
                checksum_start = 1
                checksum_len = len(rx_data) - 6
                rx_checksum = 0
                # make sure 5th-last character is a * (checksum delimiter), and ends with \r\n
                if rx_data[-5] == 0x2A and rx_data[-2] == 0x0D and rx_data[-1] == 0x0A: # ASCII for *, \r, \n

                    rx_checksum_str = str(rx_data[-4:-2], encoding="ascii").upper()
                    # check to make sure only hex digits in the checksum
                    if not all(c in string.hexdigits for c in rx_checksum_str):
                        self.rx_queue.task_done()
                        continue

                    rx_checksum = int(rx_checksum_str, base=16)

                else:
                    self.rx_queue.task_done()
                    continue

                # NMEA checksum is XOR of all data after start character
                while checksum_pos < checksum_len:
                    nmea_checksum ^= rx_data[checksum_start + checksum_pos]
                    checksum_pos += 1

                if nmea_checksum != rx_checksum:
                    self.rx_queue.task_done()
                    continue

            elif rx_data[0] == 0xB5 and rx_data[1] == 0x62: # ISO8859.1/ASCII for µb
                rx_protocol = self.INOUT_PROTOCOL.UBX
                ubx_ck_a = 0
                ubx_ck_b = 0
                checksum_pos = 0
                checksum_start = 2
                checksum_len = int.from_bytes(rx_data[4:5], byteorder='little', signed=False) + 4

                # UBX checksum is fletcher of all data after (excluding) start sequence
                while checksum_pos < checksum_len:
                    ubx_ck_a = (ubx_ck_a + rx_data[checksum_start + checksum_pos]) & 0xFF # don't have to & 0xFF if we had uint8_t :-(
                    ubx_ck_b = (ubx_ck_b + ubx_ck_a) & 0xFF
                    checksum_pos += 1

                if ubx_ck_a != rx_data[-2] or ubx_ck_b != rx_data[-1]:
                    self.rx_queue.task_done()
                    continue

            # FIXME - add support for receiving RTCM3
            # I don't have a high-precision receiver to test this...
            # ref: https://www.ucalgary.ca/engo_webdocs/GL/06.20236.MinminLin.pdf
            # ref: https://portal.u-blox.com/s/question/0D52p00009IthhBCAR/ublox-rtcm-wrapper-specification
            elif rx_data[0] == 0xD3 and (rx_data[1] & 0xFC) == 0:
                rx_protocol = self.INOUT_PROTOCOL.RTCM3
                rtcm3_crc24q = 0 # CRC24Q seed = 0
                checksum_pos = 0
                checksum_start = 0
                checksum_len = len(rx_data) - 3 # entire frame is checksummed

                # RTCM3 checksum is CRC24Q of entire frame
                crc24q_table = [
                    0x00000000,0x01864CFB,0x038AD50D,0x020C99F6,0x0793E6E1,0x0615AA1A,0x041933EC,0x059F7F17,
                    0x0FA18139,0x0E27CDC2,0x0C2B5434,0x0DAD18CF,0x083267D8,0x09B42B23,0x0BB8B2D5,0x0A3EFE2E,
                ]
                while checksum_pos < checksum_len:
                    rtcm3_crc24q ^= rx_data[checksum_pos + checksum_start] << 16
                    rtcm3_crc24q = (rtcm3_crc24q << 4) ^ crc24q_table[(rtcm3_crc24q >> 20) & 0x0F]
                    rtcm3_crc24q = (rtcm3_crc24q << 4) ^ crc24q_table[(rtcm3_crc24q >> 20) & 0x0F]

                rtcm3_crc24q &= 0xFFFFFF

                # CRC24Q sums to 0
                if rtcm3_crc24q != 0:
                    self.rx_queue.task_done()
                    continue

            self.parse_dest_lock.acquire(blocking=True)
            for dest in self.parse_dest_threads.keys():
                # check for protocol mask
                if self.parse_dest_threads[dest]['protocols'] & rx_protocol == 0:
                    continue

                if rx_protocol == self.INOUT_PROTOCOL.NMEA:
                    # check for NMEA sentence filter mask
                    if len(self.parse_dest_threads[dest]['nmea_filter']) > 0:
                        filter_match = False
                        for sentence,strmatch in self.parse_dest_threads[dest]['nmea_filter']:
                            if sentence != None:
                                if sentence.upper() != str(rx_data[3:5], encoding="ascii").upper():
                                    continue
                            if strmatch != None:
                                if strmatch.upper() not in str(rx_data[6:-6], encoding="ascii").upper():
                                    continue
                            filter_match = True
                            break
                        if filter_match == False:
                            continue

                elif rx_protocol == self.INOUT_PROTOCOL.UBX:
                    # check for UBX msgid/msgclass/offset/data filter mask
                    if len(self.parse_dest_threads[dest]['ubx_filter']) > 0:
                        filter_match = False
                        for msgclass,msgid,offset,data in self.parse_dest_threads[dest]['ubx_filter']:
                            if msgclass != None:
                                if msgclass != rx_data[2]:
                                    continue
                            if msgid != None:
                                if msgid != rx_data[3]:
                                    continue
                            if offset != None and data != None:
                                if data != rx_data[6 + offset]:
                                    continue
                            filter_match = True
                            break
                        if filter_match == False:
                            continue

                elif rx_protocol == self.INOUT_PROTOCOL.RTCM:
                    # FIXME - add support for receiving RTCM3
                    # I don't have a high-precision receiver to test this...
                    pass

                # filters passed, send it!
                dest.put_nowait(rx_data)

            # finished sending, release lock
            self.parse_dest_lock.release()
            self.rx_queue.task_done()

    def transmit(self, data: bytes) -> None:
        self.tx_queue.put(item=data, block=True, timeout=None)
        time.sleep(0.001) # force yield out of this thread so tx/rx threads can run

    def receive_queue_start(self,
        # empty filters = allow all
        protocols:INOUT_PROTOCOL|None, # protocols to allow: UBX or NMEA
        ubx_filters=[()], # list of UBX message class / message id / data offset / data value - matched equal, None = allow any
        nmea_filters=[()], # list of NMEA sentence type / string match
        rtcm3_filters=[()], # list of RTCM3 message type / string match
    ) -> queue.Queue:

        q = queue.Queue(0) # unlimited queue length, so that thread_parse() won't block

        self.parse_dest_lock.acquire(blocking=True)
        self.parse_dest_threads[q] = {}

        if protocols == None:
            protocols = self.INOUT_PROTOCOL.ALL
        self.parse_dest_threads[q]['protocols'] = protocols

        # check UBX filter is valid
        for filter in ubx_filters:
            if len(filter) == 0:
                continue
            if len(filter) != 4:
                self.parse_dest_lock.release()
                raise ValueError("Invalid UBX filter, must contain exactly 4 elements")
        self.parse_dest_threads[q]['ubx_filter'] = ubx_filters

        # check NMEA filter is valid
        for filter in nmea_filters:
            if len(filter) == 0:
                continue
            if len(filter) != 2:
                self.parse_dest_lock.release()
                raise ValueError("Invalid NMEA filter, must contain exactly 2 elements")
        self.parse_dest_threads[q]['nmea_filter'] = nmea_filters

        self.parse_dest_lock.release()
        return q

    def receive_queue_stop(self, queue: queue.Queue) -> None:
        self.parse_dest_lock.acquire(blocking=True)
        del self.parse_dest_threads[queue]
        self.parse_dest_lock.release()

    def ubx_msg_send(self, msgclass: int, msgid: int, data: bytes) -> None:
        # maximum data size = 0xFFFF (2 bytes) - ss32.2
        length = len(data)
        if length > 65535:
            raise ValueError("Maximum message length 65535 bytes exceeded")

        new_msg = bytearray(length + 8)

        # 2-byte preamble - ss32.2
        new_msg[0] = 0xB5 # ISO8859.1 for µ
        new_msg[1] = 0x62 # ASCII for b

        # 1-byte class, 1-byte message ID - ss32.2
        new_msg[2] = msgclass & 0xFF
        new_msg[3] = msgid & 0xFF

        # 2-byte length, little-endian - ss32.2
        new_msg[4:6] = length.to_bytes(length=2, byteorder='little')

        # payload - variable length (defined by length field)
        if length > 0:
            new_msg[6:(length + 6)] = data

        # 2-byte checksum - ss32.4
        new_msg[length + 6] = 0
        new_msg[length + 7] = 0

        # checksum spans from the message class and ID to the end of the data
        for byte in new_msg[2:(length + 6)]:
            new_msg[length + 6] = (new_msg[length + 6] + byte) & 0xFF
            new_msg[length + 7] = (new_msg[length + 7] + new_msg[length + 6]) & 0xFF

        self.transmit(bytes(new_msg))

    def ubx_msg_poll(self, msgclass: int, msgid: int, data=b'') -> bytes:
        response_queue = self.receive_queue_start(self.INOUT_PROTOCOL.UBX, [(msgclass, msgid, None, None)])
        self.ubx_msg_send(msgclass, msgid, data)
        response = b''

        response_count = 0
        while response_count < 5:
            response = b''
            try: response = response_queue.get(block=True, timeout=self.write_timeout)
            except queue.Empty:
                response_count += 1
                continue

            if len(response) > 0:
                if int.from_bytes(response[4:5], byteorder='little', signed=False) > 0:
                    response_queue.task_done()
                    break

            response_queue.task_done()
            response_count += 1

        self.receive_queue_stop(response_queue)

        if len(response) <= 8:
            return b''
        else:
            return bytes(response[6:-2])

    def ubx_msg_acknak(self, msgclass: int, msgid: int, data: bytes) -> bool:
        response_queue = self.receive_queue_start(self.INOUT_PROTOCOL.UBX, [(0x05, None, 0, msgclass), (0x05, None, 1, msgid)])
        self.ubx_msg_send(msgclass, msgid, data)
        response = b''
        acknak_count = 0
        while acknak_count < 5:
            response = b''
            try: response = response_queue.get(block=True, timeout=self.write_timeout)
            except queue.Empty:
                acknak_count += 1
                continue

            if len(response) > 0:
                if response[6] == msgclass and response[7] == msgid:
                    response_queue.task_done()
                    break

            response_queue.task_done()
            acknak_count += 1

        self.receive_queue_stop(response_queue)

        if len(response) == 0:
            return False
        if response[3] == 0:
            return False
        else:
            return True

    def ubx_ver_allowed(self, ver_min: float, ver_max: float) -> bool:
        if ver_min == 0 and ver_max == 0:
            return True
        if self.protocol_version >= ver_min and self.protocol_version <= ver_max:
            return True
        else:
            return False

    def ubx_mon_ver(self):
        extensions = []
        sw_version = ''
        hw_version = ''
        rom_version = ''

        response = self.ubx_msg_poll(0x0A, 0x04)
        response_pos = 0 # start of response data
        while response_pos < len(response):
            if response_pos == 0:
                sw_version = str(response[6:35], encoding='iso-8859-1').split(sep='\0', maxsplit=1)[0]
                response_pos += 30
            elif response_pos == 30:
                hw_version = str(response[36:45], encoding='iso-8859-1').split(sep='\0', maxsplit=1)[0]
                response_pos += 10
            elif response_pos == 40 and self.ubx_ver_allowed(10.00, 13.03):
                rom_version = str(response[46:75], encoding='iso-8859-1').split(sep='\0', maxsplit=1)[0]
                response_pos += 30
            else:
                extensions.append(str(response[response_pos:response_pos+29], encoding='iso-8859-1').split(sep='\0', maxsplit=1)[0])
                response_pos += 30

        if self.ubx_ver_allowed(10.00, 13.03):
            return (sw_version, hw_version, rom_version, extensions)
        elif self.ubx_ver_allowed(14.00, 34.10):
            return (sw_version, hw_version, extensions)

    def ubx_find_proto_ver(self) -> None:
        MAX_KNOWN_PROTO_VER=34.10

        extensions = []
        sw_version = ''
        hw_version = ''
        rom_version = ''

        if self.ubx_ver_allowed(10.00, 13.03):
            sw_version, hw_version, rom_version, extensions = self.ubx_mon_ver()
        elif self.ubx_ver_allowed(14.00, 34.10):
            sw_version, hw_version, extensions = self.ubx_mon_ver()
        for ext in extensions:
            if ext[:7] == 'PROTVER':
                # Protocol 17 and older (Gen5-7 and early Gen8) uses PROTVER VERSION
                # Protocol 18 and newer (most Gen8 and Gen9-10) uses PROTVER=VERSION
                # So just don't bother looking for the character between PROTVER and the version...
                self.protocol_version = float(ext[8:])
                if self.protocol_version > MAX_KNOWN_PROTO_VER:
                    raise ValueError("Receiver protocol version {} is newer than known protocol version {}".format(self.protocol_version, MAX_KNOWN_PROTO_VER))
                return

        # didn't find protocol version in extensions, this is probably a very old receiver?
        raise ValueError("Receiver protocol version not found in extensions")

    def set_logging(self, level) -> None:
        logger.setLevel(level)

    def set_stream(self, stream:serial.Serial|io.BufferedIOBase) -> None:
        if self.stream != None:
            if stream != self.stream:
                self.stream.close()

        # is our data stream actually a serial connection?
        if isinstance(stream, serial.Serial):
            # ensure serial connection is in correct mode, and hopefully correct speed
            # u-blox GPS receivers only support 8-N-1-N options on doublings of 4800 baud (4800 to 921600 baud)
            stream.baudrate = self.serial_baud
            stream.bytesize = serial.EIGHTBITS
            stream.parity = serial.PARITY_NONE
            stream.stopbits = serial.STOPBITS_ONE
            stream.timeout = 0.1 # 100 msec blocking timeout
            stream.xonxoff = False
            stream.rtscts = False
            stream.dsrdtr = False
            stream.write_timeout = self.write_timeout

        else:
            # set timeout for blocking io
            if self.read == True and self.write == True:
                raise ValueError("Unable to both read and write on a file, use a serial.Serial object instead")

        self.stream = stream

        # if read/write we are connected to the receiver using a serial stream
        # probe the receiver to determine protocol version
        if self.read == True and self.write == True:
            self.ubx_find_proto_ver()

    def set_read(self, read: bool) -> None:
        self.read = read

        # force-set read OR write on files
        # only serial connections can be R/W
        if isinstance(self.stream, io.RawIOBase) or isinstance(self.stream, io.BufferedIOBase):
            if self.stream.readable() and self.read == True:
                self.write = False
            else:
                self.read = False

    def set_write(self, write: bool) -> None:
        self.write = write

        # force-set read OR write on files
        # only serial connections can be R/W
        if isinstance(self.stream, io.RawIOBase) or isinstance(self.stream, io.BufferedIOBase):
            if self.stream.writable() and self.write == True:
                self.read = False
            else:
                self.write = False

    # defaults suitable for MAX-M8Q, override to match your own receiver
    def __init__(self, stream, serial_baud=9600, protocol_version=18.00, read=False, write=True, log_level=getattr(logging, 'INFO', None), write_timeout=1.0) -> None:
        self.set_logging(log_level)

        # init variables
        self.stream = None
        self.read = read
        self.write = write
        self.write_timeout = write_timeout

        # default speed for most u-blox GPS receivers is 9600 baud
        # the following (incomplete) list of receivers default to 38400 baud:
        # - NEO-M9N
        # - MIA-M10Q

        baud = 4800
        baud_ok = False
        while baud <= 921600:
            if baud == serial_baud:
                baud_ok = True
                break
            baud *= 2
        if baud_ok == False:
            raise ValueError("Serial baud rate must be a doubling of 4800, up to 921600")

        self.serial_baud = serial_baud

        # this module (mostly) supports every protocol version released
        # u-blox 5 series - protocol version 10.00 - 12.02
        # u-blox 6 series - protocol version 12.00 - 14.00
        # ref: u-blox document GPS.G6-SW-10018
        # u-blox 7 series - protocol version 14.00
        # ref: u-blox document GPS.G7-SW-12001-B1 - https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
        # u-blox 8 series - protocol version 15.00 - 23.01
        # ref: u-blox document UBX-13003221 - https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
        # u-blox 9 series - protocol version 32.01
        # ref: u-blox document UBX-21022436 - https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf
        # u-blox 10 series - protocol version 34.10
        # ref: u-blox document UBX-21035062 - https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf
        self.protocol_version = protocol_version

        self.tx_queue = queue.Queue(1024)
        self.tx_thread = threading.Thread(target=self.thread_tx, daemon=True)
        self.tx_thread.start()
        self.rx_queue = queue.Queue(0) # unbounded size, as rx_thread needs to be as non-blocking as possible
        self.rx_thread = threading.Thread(target=self.thread_rx, daemon=True)
        self.rx_thread.start()
        self.parse_thread = threading.Thread(target=self.thread_parse, daemon=True)
        self.parse_thread.start()
        self.parse_dest_threads = {}
        self.parse_dest_lock = threading.Lock()

        self.set_read(read)
        self.set_write(write)
        self.set_stream(stream)

    def ubx_cfg_cfg(self, data: bytes) -> None:
        # NOTE - UBX receiver implementaion has changed after protocol 23.01 (i.e. 9-series and later)
        # see receiver protocol specification (9- and 10-series) for details:
        # officially this is deprecated in 9- and 10-series receivers,
        # but it still clears/saves/loads entire configurations if any bits are set
        if not self.ubx_ver_allowed(10.00, 34.10):
            raise ValueError("ubx-cfg-cfg not supported in protocol version {}".format(self.protocol_version))

        if len(data) < 12 or len(data) > 13:
            raise ValueError("Data length must be 12 or 13 bytes")

        # apply bitfield masks
        new_data = bytearray(len(data))
        new_data[0:4] = (int.from_bytes(data[0:3], byteorder='little', signed=False) & 0b00000000000000000001111100011111).to_bytes(length=4, byteorder='little', signed=False)
        new_data[4:8] = (int.from_bytes(data[4:7], byteorder='little', signed=False) & 0b00000000000000000001111100011111).to_bytes(length=4, byteorder='little', signed=False)
        new_data[8:12] = (int.from_bytes(data[8:11], byteorder='little', signed=False) & 0b00000000000000000001111100011111).to_bytes(length=4, byteorder='little', signed=False)
        if len(data) == 13:
            new_data[12] = (data[12] & 0b00010111)

        # if this is a cfg reset, then the receiver may not respond
        self.ubx_msg_send(msgclass=0x06, msgid=0x09, data=bytes(new_data))
        time.sleep(0.5)

    def ubx_cfg_cfg_reset_all(self) -> None:
        data = bytearray(13)
        # clearMask
        data[0:4]  = (0b00000000000000000001111100011111).to_bytes(length=4, byteorder='little', signed=False)
        # saveMask
        data[4:8]  = (0b00000000000000000000000000000000).to_bytes(length=4, byteorder='little', signed=False)
        # loadMask
        data[8:12] = (0b00000000000000000001111100011111).to_bytes(length=4, byteorder='little', signed=False)

        # deviceMask
        data[12] = 0b00010111

        self.ubx_cfg_cfg(bytes(data))

    def ubx_cfg_cfg_save_all(self) -> None:
        data = bytearray(13)
        # clearMask
        data[0:4]  = (0b00000000000000000000000000000000).to_bytes(length=4, byteorder='little', signed=False)
        # saveMask
        data[4:8]  = (0b00000000000000000001111100011111).to_bytes(length=4, byteorder='little', signed=False)
        # loadMask
        data[8:12] = (0b00000000000000000000000000000000).to_bytes(length=4, byteorder='little', signed=False)

        # deviceMask
        data[12] = 0b00010111

        self.ubx_cfg_cfg(bytes(data))

    def ubx_cfg_prt(self, port: PORT, in_protocol: INOUT_PROTOCOL, out_protocol: INOUT_PROTOCOL, flags=b'\0\0', mode=b'\0\0\0\0', baud=0, txready_enable=False, txready_polarity_low=False, txready_pin=0, txready_threshold=0) -> None:
        if not self.ubx_ver_allowed(12.00, 23.01):
            raise ValueError("ubx-cfg-prt not supported in protocol version {}".format(self.protocol_version))

        # in_protocol limitations:
        # 6-series - UBX|NMEA
        # 7-series - UBX|NMEA|RTCM
        # 8-series P <20 - UBX|NMEA|RTCM
        # 8-series P>=20 - UBX|NMEA|RTCM|RTCM3
        # 9-series and later - UBX|NMEA|RTCM3
        if in_protocol != None:
            if self.ubx_ver_allowed(10.00, 13.03):
                # 6-series and earlier
                in_protocol = in_protocol & (self.INOUT_PROTOCOL.NMEA|self.INOUT_PROTOCOL.UBX)

            elif self.ubx_ver_allowed(14.00, 19.20):
                # M6, 7-series, some 8-series
                in_protocol = in_protocol & (self.INOUT_PROTOCOL.NMEA|self.INOUT_PROTOCOL.UBX|self.INOUT_PROTOCOL.RTCM)

            elif self.ubx_ver_allowed(20, 23.01):
                # some 8-series
                in_protocol = in_protocol & (self.INOUT_PROTOCOL.NMEA|self.INOUT_PROTOCOL.UBX|self.INOUT_PROTOCOL.RTCM|self.INOUT_PROTOCOL.RTCM3)

            elif self.ubx_ver_allowed(32.01, 34.10):
                # 9-series and later
                in_protocol = in_protocol & (self.INOUT_PROTOCOL.NMEA|self.INOUT_PROTOCOL.UBX|self.INOUT_PROTOCOL.RTCM3)

        # out_protocol limitations:
        # 6-series - UBX|NMEA
        # 7-series - UBX|NMEA
        # 8-series P <20 - UBX|NMEA
        # 8-series P>=20 - UBX|NMEA|RTCM3
        # 9-series and later - UBX|NMEA|RTCM3
        if out_protocol != None:
            if self.ubx_ver_allowed(10.00, 19.20):
                # 7-series and earlier, some 8-series
                out_protocol = out_protocol & (self.INOUT_PROTOCOL.NMEA|self.INOUT_PROTOCOL.UBX)

            elif self.ubx_ver_allowed(20.00, 34.10):
                # some 8-series, 9-series and later
                out_protocol = out_protocol & (self.INOUT_PROTOCOL.NMEA|self.INOUT_PROTOCOL.UBX|self.INOUT_PROTOCOL.RTCM3)

        data = bytearray(20)

        # port ID - see hardware integration guides for port ID numbers
        data[0] = int(port)

        # always reserved
        data[1] = 0

        # tx_ready flags are common across all ports
        # start filling the bit field
        tx_ready = 0

        # txReady only supported in firmware 7.01 (protocol 13.01) and later
        if self.ubx_ver_allowed(13.01, 23.01):
            # bit 0 = enable
            if txready_enable == True:
                tx_ready |= 1<<0
            else:
                tx_ready &= 0xFFFE

            # bit 1 = polarity: 0==high-enable, 1==low-enable
            if txready_polarity_low == True:
                tx_ready |= 1<<1
            else:
                tx_ready &= 0xFFFD

            # bit 2:6 = 5 bits for pin to signal txready on
            tx_ready |= (txready_pin & 0x1F) << 2

            # bit 7:15 = 9 bits for txready threshold (units of 8 bytes)
            tx_ready |= (txready_threshold & 0x1FF) << 7

        data[2:4] = tx_ready.to_bytes(length=2, byteorder='little', signed=False)

        # mode and flags are different across all ports, each port will generate its own bytes to fill
        # USB port mode and flags fields are reserved
        if port == self.PORT.USB:
            mode = b'\0\0\0\0'
            flags = b'\0\0'

        data[4:8] = mode

        # flags only supported in 7-series+ receivers (protocol 14.00+)
        if not self.ubx_ver_allowed(14.00, 23.01):
            flags = b'\0\0'

        data[16:18] = flags

        if port not in [self.PORT.UART1, self.PORT.UART2]:
            # baud field only used by UART1/2
            # other ports this field is reserved
            baud = 0

        data[8:12] = baud.to_bytes(length=4, byteorder='little', signed=False)

        # input protocol mask
        data[12:14] = int(in_protocol).to_bytes(length=2, byteorder='little', signed=False)

        # output protocol mask
        data[14:16] = int(out_protocol).to_bytes(length=2, byteorder='little', signed=False)

        data[18:20] = b'\0\0'

        # don't parse a response on uart 1, because serial settings may have changed
        if port == self.PORT.UART1 and isinstance(self.stream, serial.Serial):
            self.ubx_msg_send(0x06, 0x00, bytes(data))
            time.sleep(1)
        else:
            if self.ubx_msg_acknak(0x06, 0x00, data) == False:
                raise ValueError("Error in ubx-cfg-prt with data:\n{}".format(data.hex()))

    def ubx_cfg_prt_uart(self, uart=PORT.UART1, in_protocol=INOUT_PROTOCOL.NMEA|INOUT_PROTOCOL.UBX, out_protocol=INOUT_PROTOCOL.NMEA|INOUT_PROTOCOL.UBX, baud=9600, data_bits=8, parity_odd=None, stopbits=1.0, extended_tx_timeout=False) -> None:
        # !!! WARNING !!!
        # changes to the currently active port may result in the receiver being unable to communicate

        # any changes on port UART 1 will change the serial.Serial stream configuration to match

        # UART port can only be 1 or 2
        if self.ubx_ver_allowed(10.00, 34.10):
            if uart not in [self.PORT.UART1, self.PORT.UART2]:
                raise ValueError("UART must be PORT.UART1 or PORT.UART2 (default 1)")
        else:
            raise ValueError("ubx_cfg_prt_uart not supported in protocol version {}".format(self.protocol_version))


        # sanity-check for various firmware version limitations
        # baud rate limitations:
        # must be doublings of 4800
        # 6- and 7-series - 4800 to 115200
        # 8-series - 4800 to 460800
        # 9- and 10-series - 9600 to 921600
        test_baud = 4800
        while test_baud < 921600:
            if test_baud == baud:
                break
            elif test_baud > baud:
                # limit to next-lowest value from requested
                test_baud /= 2
                if test_baud == 2400:
                    test_baud = 4800
                logger.warning("Requested baud rate not a doubling of 4800, setting to {} from {}".format(test_baud, baud))
                break

            test_baud *= 2

        baud = int(test_baud)

        if self.ubx_ver_allowed(10.00, 13.03):
            # 7-series and earlier
            if baud < 4800:
                logger.warning("Requested baud rate of {} slower than low limit of 4800, setting to low limit".format(baud))
                baud = 4800
            elif baud > 115200:
                logger.warning("Requested baud rate of {} faster than high limit of 115200, setting to high limit".format(baud))
                baud = 115200

        elif self.ubx_ver_allowed(15.00, 23.01):
            # 8-series
            if baud < 4800:
                logger.warning("Requested baud rate of {} slower than low limit of 4800, setting to low limit".format(baud))
                baud = 4800
            elif baud > 460800:
                logger.warning("Requested baud rate of {} faster than high limit of 460800, setting to high limit".format(baud))
                baud = 460800

        elif self.ubx_ver_allowed(32.01, 34.10):
            # 9-series and later
            if baud < 9600:
                logger.warning("Requested baud rate of {} slower than low limit of 9600, setting to low limit".format(baud))
                baud = 9600
            elif baud > 921600:
                logger.warning("Requested baud rate of {} faster than high limit of 921600, setting to high limit".format(baud))
                baud = 921600

        if self.ubx_ver_allowed(10.00, 23.01):
            # 8-series and older use UBX-CFG-PRT to configure the port
            data = bytearray(self.ubx_msg_poll(0x06, 0x00, data=int(uart).to_bytes(length=1, byteorder='little', signed=False)))

            mode_bitfield = int.from_bytes(data[4:7], byteorder='little', signed=False)
            mode_bitfield &= 0x00003ED0 # bitfield reserved bits mask
            if self.ubx_ver_allowed(10.00, 14.00):
                mode_bitfield |= 1<<4 # reserved1 = default 1 for compatibility with A4 (for 7-series and earlier)
            mode_bitfield &= 0xFFFFFF3F # clear data bits setting
            mode_bitfield |= 1<<7 # 7-bit or 8-bit
            if data_bits == 8:
                mode_bitfield |= 1<<6 # 8-bit
            mode_bitfield &= 0xFFFFF1FF # clear parity bits setting
            if parity_odd == None:
                mode_bitfield |= 1<<11
            elif parity_odd == True:
                mode_bitfield |= 1<<9
            # else mode_bitfield set to 0, which is already done
            mode_bitfield &= 0xFFFFCFFF # clear stop bits setting
            if stopbits == 1.5:
                mode_bitfield |= 1<<12
            elif stopbits == 2:
                mode_bitfield |= 1<<13
            # else stopbits == 1, which is set to 00
            # stopbits == 0.5 is supported in the receiver (bits set 11) but not supported in serial.Serial

            if self.ubx_ver_allowed(14.00, 23.01):
                flags_bitfield = int.from_bytes(data[16:17], byteorder='little', signed=False)
                flags_bitfield &= 0x0002 # reserved bits mask
                flags_bitfield &= 0xFFFD # clear extended TX timeout bit
                if extended_tx_timeout == True:
                    flags_bitfield |= 1<<1

            else:
                flags_bitfield = 0

            self.ubx_cfg_prt(port=uart, in_protocol=in_protocol, out_protocol=out_protocol,
                flags=flags_bitfield.to_bytes(length=2, byteorder='little', signed=False),
                mode=mode_bitfield.to_bytes(length=4, byteorder='little', signed=False),
                baud=baud, txready_enable=False
            )

        elif self.ubx_ver_allowed(32.01, 34.10):
            # 9-series and newer use UBX-CFG-VALSET to configure the port
            raise ValueError("9-series+ configuration not implemented")

        if uart == self.PORT.UART1 and isinstance(self.stream, serial.Serial):
            self.stream.baudrate = baud

            if data_bits == 8:
                self.stream.bytesize = serial.EIGHTBITS

            elif data_bits == 7:
                self.stream.bytesize = serial.SEVENBITS
                if parity_odd == None:
                    self.stream.parity = serial.PARITY_NONE
                elif parity_odd == True:
                    self.stream.parity = serial.PARITY_ODD
                elif parity_odd == False:
                    self.stream.parity = serial.PARITY_EVEN

            # u-blox GPS receivers support 0.5 stop bits, but serial.Serial doesn't...
            if stopbits == 1:
                self.stream.stopbits = serial.STOPBITS_ONE
            elif stopbits == 1.5:
                self.stream.stopbits = serial.STOPBITS_ONE_POINT_FIVE
            elif stopbits == 2:
                self.stream.stopbits = serial.STOPBITS_TWO

    def ubx_cfg_prt_ddc(self, in_protocol=None, out_protocol=None):
        # DDC is what u-blox calls I2C (Philips/NXP trademark)
        # can usually use DDC in conjunction with UART1
        pass

    def ubx_cfg_prt_spi(self, in_protocol=None, out_protocol=None):
        # SPI usually takes over pins used for UART1 and DDC
        pass

    def ubx_cfg_prt_usb(self, in_protocol=None, out_protocol=None):
        # USB CDC-ACM port, emulates a serial port
        pass

    def ubx_cfg_rst(self, data: bytes):
        # reset may or may not be acknowledged by the receiver
        # earlier firmware does ack, or may partially ack
        self.ubx_msg_send(0x06, 0x04, data)
        time.sleep(0.5)

    def ubx_cfg_rst_hotstart(self):
        self.ubx_cfg_rst(b'\0\0\0\0')

    def ubx_cfg_rst_warmstart(self):
        self.ubx_cfg_rst(b'\x01\0\0\0')

    def ubx_cfg_rst_coldstart(self):
        self.ubx_cfg_rst(b'\xff\xff\0\0')

    def ubx_cfg_gnss(self, data: bytes):
        self.ubx_msg_acknak(0x06, 0x3E, data)

    def ubx_mon_gnss(self):
        gnss_supported = []
        gnss_major_enabled = []
        major_simultaneous = 0

        # 6-series and earlier: fixed GPS only
        # 7-series: fixed GPS+GLONASS
        # 8-series+: flexible GPS+GLONASS+Galileo+BeiDou
        if self.ubx_ver_allowed(10.00, 13.03):
            gnss_supported = [self.GNSS_ID_MAJOR.GPS, self.GNSS_ID_AUGMENT.SBAS]
            gnss_major_enabled = [self.GNSS_ID_MAJOR.GPS]
            major_simultaneous = 1
        elif self.ubx_ver_allowed(14.00, 14.00):
            gnss_supported = [self.GNSS_ID_MAJOR.GPS, self.GNSS_ID_AUGMENT.SBAS, self.GNSS_ID_AUGMENT.QZSS, self.GNSS_ID_MAJOR.GLONASS]
            gnss_major_enabled = [self.GNSS_ID_MAJOR.GPS, self.GNSS_ID_MAJOR.GLONASS]
            major_simultaneous = 1 # 7-series only supports GPS/SBAS/QZSS **OR** GLONASS
        elif self.ubx_ver_allowed(15.00, 34.10):
            # all augmentation systems supported 8-series and later
            gnss_supported = [self.GNSS_ID_AUGMENT.SBAS, self.GNSS_ID_AUGMENT.QZSS, self.GNSS_ID_AUGMENT.IMES]
            data = self.ubx_msg_poll(0x0A, 0x28)
            if not (data[0] == 0x00 or data[0] == 0x01):
                # message version 0 and 1 appear to be identical 🤷‍♂️
                # message version 1 only appears on 9-series receivers, it reverts back to version 0 on 10-series
                raise ValueError("ubx-mon-gnss unsupported message version {} received".format(data[0]))

            major_simultaneous = data[4]
            if (data[1] & 1<<0) != 0:
                gnss_supported.append(self.GNSS_ID_MAJOR.GPS)
            if (data[1] & 1<<1) != 0:
                gnss_supported.append(self.GNSS_ID_MAJOR.GLONASS)
            if (data[1] & 1<<2) != 0:
                gnss_supported.append(self.GNSS_ID_MAJOR.BeiDou)
            if (data[1] & 1<<3) != 0:
                gnss_supported.append(self.GNSS_ID_MAJOR.Galileo)

            if (data[3] & 1<<0) != 0:
                gnss_major_enabled.append(self.GNSS_ID_MAJOR.GPS)
            if (data[3] & 1<<1) != 0:
                gnss_major_enabled.append(self.GNSS_ID_MAJOR.GLONASS)
            if (data[3] & 1<<2) != 0:
                gnss_major_enabled.append(self.GNSS_ID_MAJOR.BeiDou)
            if (data[3] & 1<<3) != 0:
                gnss_major_enabled.append(self.GNSS_ID_MAJOR.Galileo)

        return (gnss_supported, gnss_major_enabled, major_simultaneous)

    def ubx_cfg_gnss_enabled(self, gnss:list[GNSS_ID_MAJOR|GNSS_ID_AUGMENT], channels=0):
        # receiver GNSS configuration rules:
        # 6-series and earlier - GPS/SBAS (no configuration of GNSS allowed)
        # 7-series - GPS/SBAS/QZSS+GLONASS (only GPS/SBAS/QZSS **OR** GLONASS + tracking channels limit)
        # 8-series+ - GPS/SBAS+GLONASS+QZSS+IMES+Galileo+BeiDou (major GNSS enabled limits per receiver + tracking channels limit)
        if len(gnss) == 0:
            raise ValueError("Must enable at least 1 GNSS")

        gnss_supported, gnss_enabled, major_simultaneous = self.ubx_mon_gnss()
        gnss_configured = []
        major_configured = 0
        minor_configured = 0
        for gnid in gnss:
            if gnid in gnss_supported:
                if gnid in self.GNSS_ID_MAJOR:
                    major_configured += 1
                    if major_configured > major_simultaneous:
                        raise ValueError("Receiver only supports {} major GNSS simultaneously".format(major_simultaneous))
                else:
                    minor_configured += 1
                gnss_configured.append(gnid)
            else:
                logger.warning("GNSS {} not supported in this receiver".format(gnid))

        if major_configured == 0:
            raise ValueError("At least 1 major GNSS must be configured")

        if self.ubx_ver_allowed(10.00, 13.03):
            # no GNSS enable/disable for 6-series and earlier
            return

        if self.ubx_ver_allowed(14.00, 14.00):
            # 7-series can configure GPS/QZSS/SBAS **OR** GLONASS
            if self.GNSS_ID_MAJOR.GLONASS in gnss and len(gnss) > 1:
                raise ValueError("7-series can only enable GLONASS without GPS, QZSS, or SBAS")

        if self.ubx_ver_allowed(14.00, 23.01):
            data = bytearray(self.ubx_msg_poll(0x06, 0x3E))
            if data[0] != 0x00:
                raise ValueError("ubx-cfg-gnss unsupported message version {} received".format(data[0]))
            if channels == 0:
                channels = data[1]

            config_pos = 0
            num_configs = data[3]
            channels_min = 0
            while config_pos < num_configs:
                gnss_id = data[4+(8*config_pos)]
                flags = int.from_bytes(data[8+(8*config_pos):11+(8*config_pos)], byteorder='little', signed=False)
                if self.ubx_ver_allowed(14.00, 14.00):
                    flags &= 0x00000001
                elif self.ubx_ver_allowed(15.00, 23.01):
                    flags &= 0x00FF0001
                if gnss_id in gnss_configured:
                    channels_min += data[5+(8*config_pos)]
                    flags |= 1<<0
                else:
                    flags &= 0xFFFFFFFE
                data[8+(8*config_pos):12+(8*config_pos)] = int.to_bytes(flags, length=4, byteorder='little', signed=False)
                config_pos += 1

            if not self.ubx_ver_allowed(23.01, 23.01):
                # number of configured channels is read-only on protocol version 23.01 and later
                if channels < channels_min:
                    raise ValueError("Not enough enabled channels to support GNSS configuration, {} required".format(channels_min))
                else:
                    data[2] = channels

            if self.ubx_cfg_gnss(data) == False:
                raise ValueError("Error in ubx-cfg-gnss with data:\n{}".format(data.hex()))

        elif self.ubx_ver_allowed(32.01, 34.10):
            raise ValueError("9-series+ configuration not implemented")

    def ubx_cfg_gnss_all(self, gnss:GNSS_ID_MAJOR|GNSS_ID_AUGMENT, enabled=True, min_channels=None, max_channels=None, sig_cfg_mask=None):
        gnss_supported, gnss_enabled, major_simultaneous = self.ubx_mon_gnss()
        if gnss not in gnss_supported:
            return
            raise ValueError("Receiver does not support GNSS {}".format(gnss))

        if self.ubx_ver_allowed(10.00, 13.03):
            # no GNSS enable/disable for 6-series and earlier
            return

        if self.ubx_ver_allowed(14.00, 23.01):
            data = bytearray(12)
            cur_data = self.ubx_msg_poll(0x06, 0x3E)
            if cur_data[0] != 0x00:
                raise ValueError("ubx-cfg-gnss unsupported message version {} received".format(data[0]))
            channels_avail = cur_data[1]

            data[:3] = cur_data[:3]
            data[3] = 1

            config_pos = 0
            num_configs = cur_data[3]
            while config_pos < num_configs:
                if cur_data[4+(8*config_pos)] == gnss:
                    data[4:] = cur_data[4+(8*config_pos):12+(8*config_pos)]
                    break
                config_pos += 1

            flags = int.from_bytes(data[8:12], byteorder='little', signed=False)
            enabled_cur = False
            if flags & 0x00000001 != 0:
                enabled_cur = True

            if self.ubx_ver_allowed(14.00, 14.00):
                flags &= 0x00000001
            elif self.ubx_ver_allowed(15.00, 23.01):
                flags &= 0x00FF0001
                if sig_cfg_mask != None:
                    flags &= 0x00000001
                    flags |= sig_cfg_mask << 16
            if enabled == True:
                flags |= 1<<0
            else:
                flags &= 0xFFFFFFFE
            data[8:] = int.to_bytes(flags, length=4, byteorder='little', signed=False)

            if not self.ubx_ver_allowed(23.01, 23.01):
                if min_channels != None:
                    if min_channels < 4 and gnss in self.GNSS_ID_MAJOR:
                        raise ValueError("Major GNSS must have minimum 4 channels, {} requested".format(min_channels))

                    data[5] = min_channels

                if max_channels != None:
                    if max_channels < data[5]:
                        raise ValueError("Max {} channels must be >= {} min channels".format(max_channels, data[5]))
                    elif max_channels > channels_avail:
                        logger.warning("{} channels requested, setting to {} available".format(max_channels, channels_avail))
                        max_channels = channels_avail

                    data[6] = max_channels

            if self.ubx_cfg_gnss(data) == False:
                raise ValueError("Error in ubx-cfg-gnss with data:\n{}".format(data.hex()))

            # cold-start if a GNSS system is enabled/disabled
            if enabled_cur != enabled:
                self.ubx_cfg_cfg_save_all()
                self.ubx_cfg_rst_coldstart()

        elif self.ubx_ver_allowed(32.01, 34.10):
            raise ValueError("9-series+ configuration not implemented")


    def ubx_cfg_gnss_gps(self, enabled=True, min_channels=None, max_channels=None, sig_cfg_mask=None):
        self.ubx_cfg_gnss_all(self.GNSS_ID_MAJOR.GPS, enabled, min_channels, max_channels, sig_cfg_mask)

    def ubx_cfg_gnss_glonass(self, enabled=True, min_channels=None, max_channels=None, sig_cfg_mask=None):
        self.ubx_cfg_gnss_all(self.GNSS_ID_MAJOR.GLONASS, enabled, min_channels, max_channels, sig_cfg_mask)

    def ubx_cfg_gnss_galileo(self, enabled=True, min_channels=None, max_channels=None, sig_cfg_mask=None):
        self.ubx_cfg_gnss_all(self.GNSS_ID_MAJOR.Galileo, enabled, min_channels, max_channels, sig_cfg_mask)

    def ubx_cfg_gnss_beidou(self, enabled=True, min_channels=None, max_channels=None, sig_cfg_mask=None):
        self.ubx_cfg_gnss_all(self.GNSS_ID_MAJOR.BeiDou, enabled, min_channels, max_channels, sig_cfg_mask)

    def ubx_cfg_gnss_imes(self, enabled=True, min_channels=None, max_channels=None, sig_cfg_mask=None):
        self.ubx_cfg_gnss_all(self.GNSS_ID_AUGMENT.IMES, enabled, min_channels, max_channels, sig_cfg_mask)

    def ubx_cfg_gnss_qzss(self, enabled=True, min_channels=None, max_channels=None, sig_cfg_mask=None):
        self.ubx_cfg_gnss_all(self.GNSS_ID_AUGMENT.QZSS, enabled, min_channels, max_channels, sig_cfg_mask)

    def ubx_cfg_sbas(self, data: bytes):
        if self.ubx_msg_acknak(0x06, 0x16, data) == False:
            raise ValueError("Error in ubx-cfg-sbas with data:\n{}".format(data.hex()))

    def ubx_cfg_gnss_sbas(self, enabled=True, min_channels=1, max_channels=None, sig_cfg_mask=None, test_mode=False, ranging=True, correction=True, integrity=False, prn_mask=0):
        # SBAS configuration is in 2 locations for 7-series and later
        # SBAS GNSS receiver portion in ubx-cfg-gnss and SBAS-specific configuration in ubx-cfg-sbas
        self.ubx_cfg_gnss_all(self.GNSS_ID_AUGMENT.SBAS, enabled, min_channels, max_channels, sig_cfg_mask)

        if self.ubx_ver_allowed(10.00, 23.01):
            data = bytearray(8)

            mode = 0
            if enabled == True:
                mode |= 1<<0
            if test_mode == True:
                mode |= 1<<1
            data[0] = mode

            usage = 0
            if ranging == True:
                usage |= 1<<0
            if correction == True:
                usage |= 1<<1
            if integrity == True:
                usage |= 1<<2
            data[1] = usage

            if self.ubx_ver_allowed(10.00, 13.03):
                # channels dedicated to SBAS configured in ubx-cfg-sbas in protocol < 14
                if min_channels > 3:
                    logger.warning("Maximum of 3 SBAS channels supported on 6-series receivers")
                    min_channels = 3
                data[2] = min_channels

            # PRN mask is 39 bits
            # the value is split across 5 bytes, in a bizarre order
            data[3] = (prn_mask & 0xFF00000000) >> 32
            data[4:8] = int.to_bytes(prn_mask & 0x00FFFFFFFF, length=4, byteorder='little', signed=False)

            self.ubx_cfg_sbas(bytes(data))

        elif self.ubx_ver_allowed(32.01, 34.10):
            raise ValueError("9-series+ configuration not implemented")

    def ubx_cfg_msg(self, msgclass:int, msgid:int, ports=[], rate=1):
        if self.ubx_ver_allowed(10.00, 23.01):
            data = bytearray(8)
            data[0] = msgclass & 0xFF
            data[1] = msgid & 0xFF
            if len(ports) == 0:
                # configure all ports with same rate
                for i in range(0,6):
                    data[i+2] = rate
            else:
                data[0:8] = self.ubx_msg_poll(0x06, 0x01, bytes(data[0:1]))
                for port in ports:
                    data[int(port) + 2] = rate

            if self.ubx_msg_acknak(0x06, 0x01, data) == False:
                raise ValueError("Error in ubx-cfg-msg with data:\n{}".format(data.hex()))

        elif self.ubx_ver_allowed(32.01, 34.10):
            raise ValueError("9-series+ configuration not implemented")
