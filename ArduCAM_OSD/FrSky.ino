#ifdef USE_FRSKY

#define Protocol_HeadTail    0x5E

// Data Ids (bp = before point; af = after point)
// Official data IDs
#define ID_GPS_Altitude_bp    0x01
#define ID_GPS_Altitude_ap    0x09
#define ID_Temperature1       0x02
#define ID_RPM                0x03
#define ID_Fuel_level         0x04
#define ID_Temperature2       0x05
#define ID_Volt               0x06
#define ID_Altitude_bp        0x10
#define ID_Altitude_ap        0x21
#define ID_GPS_speed_bp       0x11
#define ID_GPS_speed_ap       0x19
#define ID_Longitude_bp       0x12
#define ID_Longitude_ap       0x1A
#define ID_E_W                0x22
#define ID_Latitude_bp        0x13
#define ID_Latitude_ap        0x1B
#define ID_N_S                0x23
#define ID_Course_bp          0x14
#define ID_Course_ap          0x1C
#define ID_Date_Month         0x15
#define ID_Year               0x16
#define ID_Hour_Minute        0x17
#define ID_Second             0x18
#define ID_Acc_X              0x24
#define ID_Acc_Y              0x25
#define ID_Acc_Z              0x26
#define ID_Voltage_Amp_bp     0x3A
#define ID_Voltage_Amp_ap     0x3B
#define ID_Current            0x28
// User defined data IDs
#define ID_Gyro_X             0x40
#define ID_Gyro_Y             0x41
#define ID_Gyro_Z             0x42
#define ID_Angle_Roll        0x43
#define ID_Angle_Pitch       0x44
#define ID_Flags               0x45
#define ID_Throttle           0x46

#define PACKET_STEP_NONE 0
#define PACKET_STEP_HEAD 1
#define PACKET_STEP_VALUE 2

static bool fc_link_active = false;
static uint8_t crlf_count = 0;
static uint8_t last_packet_type = 0;
static uint8_t last_packet_step = PACKET_STEP_NONE;

static uint8_t readbyte() {
	while (Serial.available() < 1);
	return Serial.read();
}

static uint8_t decodeserializedbyte(uint8_t c) {
	if (c == 0x5D) {
		c = readbyte();
		return c ^ 0x60;
		}
	return c;
}

int ret_invalid_packet() {
	last_packet_step = PACKET_STEP_NONE;
	last_packet_type = 0;
	return 0;
}

int read_fc_link(void) {
	uint32_t param, i;
	int ret = 0;

	static uint16_t vbat_A_last = 0;
	static int16_t osd_heading_last = 0;

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

		if (last_packet_step == PACKET_STEP_NONE) {
			if (c != Protocol_HeadTail) return ret_invalid_packet();
			last_packet_step++;
			return 0;
		}
		if (last_packet_step == PACKET_STEP_HEAD) {
			if (c == Protocol_HeadTail) return 0; // first byte was tail, this is next bytes head
			last_packet_type = c;
			last_packet_step++;
			return 0;
		}
		if (last_packet_step == PACKET_STEP_VALUE) {
			c = decodeserializedbyte(c);

			uint8_t high = decodeserializedbyte(readbyte());
			uint16_t val = (high << 8) | c;

/*
osd_alt
osd_climb
motor_armed 1/0
acc_valid 1/0
mag_valid 1/0
nosignal  1/0
osd_vbat_A
osd_throttle (0-255)
osd_rssi = 0-100?
osd_pitch = param * 180.0f / 32768;
osd_roll = param * 180.0f / 32768;
osd_heading = param * 360.0f / 256;
osd_rpm[0-6] = param16byte
osd_esctemp[0-6] = param16byte
osd_fix_type // GPS lock 0-1=no fix, 2=2D, 3=3D
osd_satellites_visible
osd_lat
osd_lon
*/

			if (last_packet_type == ID_Altitude_bp)
				osd_alt = (float) ((int16_t) val);
			else if (last_packet_type == ID_Altitude_ap)
				osd_alt = (float) (((int16_t)osd_alt) + (val / 100.0f));

			else if (last_packet_type == ID_Voltage_Amp_bp)
				{
				osd_vbat_A = val * 100 * 21 / 1100.0f;
				vbat_A_last = val * 100;
				}
			else if (last_packet_type == ID_Voltage_Amp_ap)
				osd_vbat_A = (vbat_A_last + (val * 10)) * 21/1100.0f;
			else if (last_packet_type == ID_Voltage_Amp_bp)
				{
				osd_vbat_A = val * 100 * 21 / 1100.0f;
				vbat_A_last = val * 100;
				}
			else if (last_packet_type == ID_Voltage_Amp_ap)
				osd_vbat_A = (vbat_A_last + (val * 10)) * 21/1100.0f;
			else if (last_packet_type == ID_Course_bp)
			{
				osd_heading = (int16_t)(val * 100);
				osd_heading_last = osd_heading;
			}
			else if (last_packet_type == ID_Course_ap)
			{
				osd_heading = (osd_heading_last + val)/100.0f;
			}
			else if (last_packet_type == ID_Angle_Roll)
			{
				osd_roll = (int16_t) val;
			}
			else if (last_packet_type == ID_Angle_Pitch)
			{
				osd_pitch = (int16_t) val;
			}
			else if (last_packet_type == ID_Throttle)
			{
				osd_throttle = val; // 0-100
			}
			else if (last_packet_type == ID_Flags)
			{
				motor_armed = (ID_Flags & 1)  ? 1 : 0;
				osd_fix_type = (ID_Flags & 2)  ? 3 : 0;

				if (ID_Flags&4) osd_mode = 0; // Stab
				else if (ID_Flags&8) osd_mode = 2; // Alt Hold
				else if (ID_Flags&32) osd_mode = 6; // RTL
				else if (ID_Flags&64) osd_mode = 5; // Loiter
				else osd_mode = 1; // Acro
			}

			ret_invalid_packet(); // reset
			fc_link_active = true;
			return 1;
		}

		return 0;

/*
		case LINK_GPS:
			param = readbyte();
			osd_fix_type = param & 15;
			osd_satellites_visible = param >> 4;
			param = readbyte() << 16;
			param = readbyte() << 8;
			param = readbyte() << 0;
			osd_lat = param / (0x1000000 / 180.0);
			if (osd_lat > 90.0)
				osd_lat -= 180.0;
			param = readbyte() << 16;
			param = readbyte() << 8;
			param = readbyte() << 0;
			osd_lon = param / (0x1000000 / 360.0);
			if (osd_lat > 180.0)
				osd_lat -= 360.0;
*/
	}
	return 0;
}
#endif
