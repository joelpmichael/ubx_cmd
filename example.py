#!/usr/bin/env python3
import serial

from ubx_cmd import UbxCmd

s = serial.Serial('COM6')
ubx = UbxCmd(s, read=True, write=True)
print(ubx)
ubx.ubx_cfg_cfg_reset_all()

# explicitly set protocols allowed on UART1
ubx.ubx_cfg_prt_uart(uart=ubx.PORT.UART1, in_protocol=ubx.INOUT_PROTOCOL.NMEA|ubx.INOUT_PROTOCOL.UBX, out_protocol=ubx.INOUT_PROTOCOL.NMEA|ubx.INOUT_PROTOCOL.UBX)

# configure GNSS systems:
# enable major GPS, Galileo, BeiDou; minor QZSS, SBAS
ubx.ubx_cfg_gnss_enabled(gnss=[ubx.GNSS_ID_MAJOR.GPS, ubx.GNSS_ID_MAJOR.Galileo, ubx.GNSS_ID_MAJOR.BeiDou, ubx.GNSS_ID_AUGMENT.QZSS, ubx.GNSS_ID_AUGMENT.SBAS])
ubx.ubx_cfg_gnss_gps(min_channels=8, max_channels=24)
ubx.ubx_cfg_gnss_galileo(min_channels=4, max_channels=12)
ubx.ubx_cfg_gnss_beidou(min_channels=8, max_channels=24)

# we can see QZSS here, use it
ubx.ubx_cfg_gnss_qzss(min_channels=2, max_channels=4, sig_cfg_mask=0x05)

# new SouthPAN SBAS system is currently in test mode using PRN122
ubx.ubx_cfg_gnss_sbas(min_channels=1, max_channels=2, test_mode=True, ranging=False, correction=True, integrity=False, prn_mask=ubx.SBAS_PRN.PRN122)

# disable NMEA-GSV spam
ubx.ubx_cfg_msg(0xF0, 0x03, rate=0)

# enable NMEA-ZDA date/time messages
ubx.ubx_cfg_msg(0xF0, 0x08, rate=1)

ubx.ubx_cfg_cfg_save_all()
