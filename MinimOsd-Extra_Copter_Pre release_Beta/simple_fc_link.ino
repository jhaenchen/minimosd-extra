#ifndef USE_MAVLINK

static bool fc_link_active = false;
static uint8_t crlf_count = 0;

static uint8_t readbyte(void) {
    while (Serial.available() < 1);
    return Serial.read();
}

int read_fc_link(void) {
    uint32_t param, i;
    int ret = 0;

    /* Grabbing data */
    while(Serial.available() > 0) {
        uint8_t c = readbyte();

        /*
         * Allow CLI to be started by hitting enter 3 times, if no
         * heartbeat packets have been received
         */
        if (!fc_link_active && millis() < 20000 && millis() > 5000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                uploadFont();
            }
        }

	ret = 1;

#define LINK_BARO_ALT 0x80
#define LINK_FLAGS    0x81
#define LINK_STATUS   0x82
#define LINK_ATTITUDE 0x83
#define LINK_ESCDATA  0x84

        switch (c) {
        case LINK_BARO_ALT:
            param = readbyte() << 8;
            param |= readbyte();
            osd_alt = param / 10.0f;
            param = readbyte();
            osd_climb = (int8_t) (uint8_t) param / 10.0f;
            break;

        case LINK_FLAGS:
            param = readbyte();
            motor_armed = (param >> 0) & 1;
            acc_valid = (param >> 1) & 1;
            mag_valid = (param >> 2) & 1;
            nosignal = (param >> 3) & 1;
            break;

        case LINK_STATUS:
            param = readbyte() << 8;
            param |= readbyte();
            osd_vbat_A = param / 1000.0f;
            param = readbyte();
            osd_throttle = param * 100 / 255;
            param = readbyte();
            osd_rssi = param;
            break;

        case LINK_ATTITUDE:
            param = readbyte() << 8;
            param |= readbyte();
            osd_pitch = (int16_t) (uint16_t) param * 180.0f / 32768;
            param = readbyte() << 8;
            param |= readbyte();
            osd_roll = (int16_t) (uint16_t) param * 180.0f / 32768;
            param = readbyte();
            osd_heading = (uint8_t) param * 360.0f / 256;
            break;

        case LINK_ESCDATA:
            i = readbyte();
            param = readbyte() << 8;
            param |= readbyte();
            if (i < 0 || i > 7) {
                readbyte();
                readbyte();
                break;
            }
            osd_rpm[i] = param;
            param = readbyte() << 8;
            param |= readbyte();
            osd_esctemp[i] = param;
            break;

/*
            case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    osd_gps_alt = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000.0f;
                    osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                    osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                    osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                    osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                    osd_cog = mavlink_msg_gps_raw_int_get_cog(&msg);
                }
                break;
*/
        default:
            ret = 0;
            continue;
        }

        fc_link_active = true;
    }

    return ret;
}
#endif
