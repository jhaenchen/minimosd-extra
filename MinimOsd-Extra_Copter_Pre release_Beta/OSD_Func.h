
//------------------ Heading and Compass ----------------------------------------

#define ROSE_WIDTH 16
static char buf_show[ROSE_WIDTH + 1];
const char buf_Rule[] = {
    0x82, 0x80, 0x81, 0x80, 0x81, 0x80, 0x81, 0x80,
    0x84, 0x80, 0x81, 0x80, 0x81, 0x80, 0x81, 0x80,
    0x83, 0x80, 0x81, 0x80, 0x81, 0x80, 0x81, 0x80,
    0x85, 0x80, 0x81, 0x80, 0x81, 0x80, 0x81, 0x80,
};
void setHeadingPatern()
{
  int start = round(osd_heading * sizeof(buf_Rule) / 360) +
      sizeof(buf_Rule) * 2 - ROSE_WIDTH / 2 + 1;

  for (int x = 0; x < ROSE_WIDTH; x++)
    buf_show[x] = buf_Rule[start++ % sizeof(buf_Rule)];

  buf_show[ROSE_WIDTH] = '\0';
}

//------------------ Battery Remaining Picture ----------------------------------

char setBatteryPic(uint16_t bat_level)
{
  if(bat_level <= 100){
    return 0xb4;
  }
  else if(bat_level <= 300){
    return 0xb5;
  }
  else if(bat_level <= 400){
    return 0xb6;
  }
  else if(bat_level <= 500){
    return 0xb7;
  }
  else if(bat_level <= 800){
    return 0xb8;
  }
  else return 0xb9;
}

//------------------ Home Distance and Direction Calculation ----------------------------------

void setHomeVars(OSD &osd)
{
  float dstlon, dstlat;
  int bearing, now;
  static int osd_alt_millis = 0;
  static bool last_armed = 0;
  static float prev_lat = 0;
  static float prev_lon = 0;
  static float prev_alt = 0;
  static uint16_t prev_cog = 0;

  osd_alt_to_home = (osd_alt - osd_home_alt);

  //Check arm/disarm switching.
  if (motor_armed && !last_armed){
    //If motors armed, reset home in Arducopter version
    osd_got_home = !motor_armed;
    haltset = !motor_armed;
    last_armed = motor_armed;
  }

  // JRChange: osd_home_alt: check for stable osd_alt (must be stable for 3s)
  // calculate osd_heading if not available (0), or maybe do it in the proto
  if(!haltset && fabs(osd_alt) > 0.1f){
    if(fabs(osd_alt_prev - osd_alt) > 0.5f || osd_zerr > 5.0f){
      osd_alt_millis = millis();
      osd_alt_prev = osd_alt;
    } else if (millis() - osd_alt_millis >= 3000) {
      osd_home_alt = osd_alt;  // take this stable osd_alt as osd_home_alt
      haltset = 1;
    }
  }

  if(!osd_got_home && osd_fix_type > 1 && osd_err <= 5.0f){
    osd_home_lat = osd_lat;
    osd_home_lon = osd_lon;
    osd_got_home = 1;
  }
  else if(osd_got_home == 1){
    /* Skip the calculation if we've not moved or not received new coords */
    if (fabs(osd_lat - prev_lat) < 0.5f && fabs(osd_lon - prev_lon) < 0.5f)
      return;

    // shrinking factor for longitude going to poles direction
    float rads = fabs(osd_home_lat) * 0.0174532925f;
    float scaleLongDown = cos(rads);
    float scaleLongUp   = 1.0f / scaleLongDown;

    //DST to Home
    dstlat = osd_home_lat - osd_lat;
    dstlon = osd_home_lon - osd_lon;
    osd_home_distance = sqrt(sq(dstlat * 111319.5f) +
        sq(dstlon * 111319.5 * scaleLongDown));

    //DIR to Home
    bearing = 720 + 90 + atan2(dstlat * scaleLongUp, -dstlon) *
        57.295775f; // absolute home direction
    bearing -= 180; // absolute return direction
    bearing = bearing - osd_heading; // relative home direction
    osd_home_direction = (bearing * 16 + 7) / 360 % 16 + 1;//array of arrows =)

    /* If we're not receiving COG/speed information, calculate our own */
    if (osd_cog == prev_cog) {
      /* DST travelled */
      dstlat = osd_lat - prev_lat;
      dstlon = osd_lon - prev_lon;
      osd_groundspeed = sqrt(sq(dstlat * 111319.5f) +
          sq(dstlon * 111319.5f * scaleLongDown));

      /* DIR */
      osd_cog = (uint32_t) (72000.0f + 9000.0f +
          atan2(dstlat * scaleLongUp, -dstlon) * 5729.5775f) % 36000;
    }
    prev_cog = osd_cog;
    prev_lat = osd_lat;
    prev_lon = osd_lon;
  }
}

void setFdataVars(){
  //
  if (takeofftime == 0 && osd_throttle > 15 && motor_armed){
    takeofftime = 1;
    tdistance = 0;
    FTime = (millis()/1000);
    start_battery_reading = osd_battery_remaining_A;
    last_battery_reading = osd_battery_remaining_A;
  }

  //Check if is moving (osd_groundspeed > 1Km/h or osd_climb within ]-10, 10[ m/m
  //if((osd_groundspeed > 0.28) || (osd_climb < -0.17) || (osd_climb > 0.17)){
  if(osd_throttle > 15){
    not_moving_since = millis();
    landed_at_time = 4294967295; //Airborn
  }
  //If it is stoped for more than 10 seconds, declare a landing
  else if(((millis() - not_moving_since) > 10000) && (landed_at_time == 4294967295) && (takeofftime == 1)){
    landed_at_time = millis();
  }

  if (osd_groundspeed > 1.0) tdistance += (osd_groundspeed * (millis() - dt) / 1000.0);
  mah_used += (osd_curr_A * 10.0 * (millis() - dt) / 3600000.0);
  //tdistance += (millis() - dt) / 1000.0 * osd_groundspeed;
  //mah_used += (millis() - dt) / 3600000.0 * osd_curr_A * 10.0;
  dt = millis();

  if (takeofftime == 1){
    start_Time = (millis()/1000) - FTime;
    if (osd_home_distance > max_home_distance) max_home_distance = osd_home_distance;
    if (osd_airspeed > max_osd_airspeed) max_osd_airspeed = osd_airspeed;
    if (osd_groundspeed > max_osd_groundspeed) max_osd_groundspeed = osd_groundspeed;
    if (osd_alt_to_home > max_osd_home_alt) max_osd_home_alt = osd_alt_to_home;
    if (osd_windspeed > max_osd_windspeed) max_osd_windspeed = osd_windspeed;
  }
}

