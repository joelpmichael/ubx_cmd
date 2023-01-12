# Small dodgy library for configuring u-blox GPS receivers

This is a combination of my python coding practice, making python do things that I normally do in C, implementing a specification from a datasheet, and scratching a personal itch. It may be useful to others, so have at it! 

The eventual goal is to support every released u-blox GPS receiver, but I only have a small selection to test with. Hardware donations of different u-blox GPS receivers are welcome. I currently have access to:

- MAX-M8Q
- SAM-M8Q
- MAX-7Q

PRs to make it suck less or implement more of the spec are welcome - especially implementing 9-series configuration.

I may eventually add support for talking to u-blox receivers over I2C or SPI, currently only serial (UART or USB) connections are supported.

## Dependencies

Requires pyserial to talk interactively to a GPS receiver.

    pip install pyserial

## Basic Usage

### read/write on a serial connection

    from ubx_cmd import UbxCmd
    import serial

    s = serial.Serial("COM6")
    ubx = UbxCmd(s, read=True, write=True)

### Write UBX binary protocol to a file

    from ubx_cmd import UbxCmd

    f = open("FILE", "wb")
    ubx = UbxCmd(f, read=False, write=True)

### Read UBX binary protocol from STDIN

    from ubx_cmd import UbxCmd
    import sys

    ubx = UbxCmd(sys.stdin, read=True, write=False)

## Querying Data

`bytes = ubx_msg_poll(msgclass:int, msgid:int, data:bytes)` = creates a UBX binary protocol message that polls the receiver for the given message class and message ID data. If the poll request message requires additional data, set the data bytes, otherwise the poll request message length is set to 0. Returns: `bytes` object of the poll response payload, excluding headers and checksum.

`bool = ubx_msg_acknak(msgclass:int, msgid:int, data:bytes)` = creates a UBX binary protocol message that expects a `UBX-ACK-(N)ACK` response (e.g. UBX-CFG commands). Returns: `bool` = `True` if `UBX-ACK-ACK` response received, `False` if `UBX-ACK-NACK` response received.

## Sending Data

`ubx_msg_send(msgclass:int, msgid:int, data:bytes)` = creates a UBX binary protocol message of the given message class, message ID, and data. Generates the header and checksum, and sends the bytes to the stream.

`transmit(data:bytes)` = sends the data bytes to the stream exactly as passed.

## Receiving Data

`queue.Queue = receive_queue_start(protocols:UbxMsg.INOUT_PROTOCOL|None, ubx_filters:[(msgclass, msgid, offset, data)], nmea_filters:[(sentence, strmatch)])` = received messages that match filters are inserted into the returned queue. `protocols` is a bitwise mask of protocols to match, if set to `None` then will match all protocols. `ubx_filters` is a list of tuples that will match the UBX binary protocol, with `None` in a filter tuple element meaning match anything. `nmea_filters` is a list of tuples that will match the NMEA protocol, with `None` in a filter tuple element meaning match anything. Queue messages will be exactly as received from the stream, including headers and checksums.

`receive_queue_stop(queue:queue.Queue)` = stop and delete the passed receive queue.

## `UbxCmd` Class Initialisation Options

`stream` = an IO file handle that sends/receives bytes using `read()` and `write()` methods. Required - no default.

`serial_baud` = initial baud rate of the serial port - can be changed later using `ubx_cfg_prt_uart(baud=int)` but must be correct at init time when `read=True`. Most u-blox GPS receivers are set to 9600, however some newer receivers (e.g. NEO-M9N, MIA-M10Q) are set to 38400. Default: `9600`

`read` and `write` = enable reading or writing of the stream respectively. Stream can only be read+write if it is a `serial.Serial` stream. Default: `read=False` and `write=True`

`write_timeout` = seconds to wait after sending a write command for a response from the receiver. Default: `1.0`

`protocol_version` = manually-set protocol version, to ensure command compatibility. If the stream is read+write (i.e. a `serial.Serial` stream), the protocol version of the attached receiver will be probed and this parameter is ignored. Default: `18.00` (compatible with MAX-M8Q and SAM-M8Q)

`log_level` = specify a different `logging` log level for the module if necessary. Default: `INFO`

## Receiver Configuration Methods

`ubx_cfg_cfg_reset_all()` = revert all configuration to defaults. Uses `ubx_cfg_cfg` internally.

`ubx_cfg_cfg_save_all()` = save current configuration to all configuration locations. Uses `ubx_cfg_cfg` internally.

`ubx_cfg_cfg(data:bytes)` = send a `UBX-CFG-CFG` message with the provided data bytes. Does not wait for a `UBX-ACK-(N)ACK` response as communication may be interrupted, instead it waits 0.5 seconds.

## Receiver Reset Methods

`ubx_cfg_rst_hotstart()` = hot-start the receiver immediately using internal watchdog. Uses `ubx_cfg_rst` internally.

`ubx_cfg_rst_hotstart()` = warm-start the receiver immediately using internal watchdog, discarding received ephemeresis data only. Uses `ubx_cfg_rst` internally.

`ubx_cfg_rst_coldstart()` = cold-start the receiver immediately using internal watchdog, discarding all received data. Uses `ubx_cfg_rst` internally.

`ubx_cfg_rst(data:bytes)` = send a `UBX-CFG-RST` message with the provided data bytes. Does not wait for a `UBX-ACK-(N)ACK` response as the hardware reset may happen before the response message is fully transmitted in older receivers, and newer receivers do not send a response at all, instead it waits 0.5 seconds.

## Receiver GNSS Configuration Methods

`ubx_cfg_gnss_enabled(gnss:list[UbxCmd.GNSS_ID_MAJOR|UbxCmd.GNSS_ID_AUGMENT], channels:int)` = sets the enabled GNSS constellations to the list provided, and sets the maximum number of channels to use in the receiver. Refer to the u-blox Receiver Protocol Description for your receiver for the supported GNSS constellations, number of simultaneous GNSS supported, and constraints on enabling multiple GNSS simultaneously.

`ubx_cfg_gnss_gps(enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int)` = calls `ubx_cfg_gnss_all(gnss=UbxCmd.GNSS_ID_MAJOR.GPS)` and passes other arguments through.

`ubx_cfg_gnss_glonass(enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int)` = calls `ubx_cfg_gnss_all(gnss=UbxCmd.GNSS_ID_MAJOR.GLONASS)` and passes other arguments through.

`ubx_cfg_gnss_galileo(enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int)` = calls `ubx_cfg_gnss_all(gnss=UbxCmd.GNSS_ID_MAJOR.Galileo)` and passes other arguments through.

`ubx_cfg_gnss_beidou(enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int)` = calls `ubx_cfg_gnss_all(gnss=UbxCmd.GNSS_ID_MAJOR.BeiDou)` and passes other arguments through.

`ubx_cfg_gnss_imes(enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int)` = calls `ubx_cfg_gnss_all(gnss=UbxCmd.GNSS_ID_AUGMENT.IMES)` and passes other arguments through.

`ubx_cfg_gnss_qzss(enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int)` = calls `ubx_cfg_gnss_all(gnss=UbxCmd.GNSS_ID_AUGMENT.QZSS)` and passes other arguments through.

`ubx_cfg_gnss_sbas(enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int, test_mode:bool, ranging:bool, correction:bool, integrity:bool, prn_mask:UbxCfg.SBAS_PRN)` = calls `ubx_cfg_gnss_all(gnss=UbxCmd.GNSS_ID_AUGMENT.SBAS)` and passes other arguments through. Also configures SBAS settings.

`ubx_cfg_gnss_all(gnss:GNSS_ID_MAJOR|GNSS_ID_AUGMENT, enabled:bool, min_channels:int, max_channels:int, sig_cfg_mask:int)` = xxx

`ubx_cfg_sbas(data:bytes)` = xxx

## Receiver Port Configuration Methods

## Receiver Message Configuration Methods

`ubx_cfg_msg(msgclass:int, msgid:int, ports=list[UbxCfg.PORT], rate:int)` = set the receiver to automatically output the selected message class and message ID on the selected port(s) every `rate` seconds. If `ports` is an empty list, applies to all ports on the receiver. If `rate` is set to 0, disables the sending of the selected message. Refer to the u-blox Receiver Protocol Description for your receiver for valid message classes and IDs.

## Class Configuration Methods

`set_stream(stream:serial.Serial|io.BufferedIOBase)` = change the IO stream to a new stream. Closes the old stream. If the new stream is a serial.Serial stream, it is configured with the current baud rate.

`set_read(read:bool)` = enable or disable reading of the stream. If the stream is not serial.Serial, writing will be disabled if reading is set to True

`set_write(write:bool)` = enable or disable writing of the stream. If the stream is not serial.Serial, reading will be disabled if writing is set to True

`set_logging(level:logging._Level)` = change the `logging` log level
