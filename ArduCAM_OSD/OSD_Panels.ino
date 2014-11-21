/******* STARTUP PANEL *******/

void startPanels(){
  panLogo(); // Display our logo  
  do_converts(); // load the unit conversion preferences
}
//------------------ Panel: Startup ArduCam OSD LOGO -------------------------------

void panLogo(){
    osd.setPanel(5, 5);
    osd.openPanel();
    osd.printf_P(PSTR("\xb0\xb1\xb2\xb3\xb4|\xb5\xb6\xb7\xb8\xb9|MinimOSD-Extra Copter|Pre-Release r605"));
    osd.closePanel();
}


/******* PANELS - POSITION *******/

static boolean force_redraw = 1;

void writePanels() {
  static uint16_t last_redraw = 0;
  if ((uint16_t) (millis() - last_redraw) > 10000) {
    osd_clear = 1;
    last_redraw = millis();
  }
//  if(millis() < (lastMAVBeat + 2200))
//    waitingMAVBeats = 1;
  //if(ISd(panel,Warn_BIT)) panWarn(panWarn_XY[0][panel], panWarn_XY[1][panel]); // this must be here so warnings are always checked

  //Only show flight summary 10 seconds after landing and if throttle < 15
  if ((landed_at_time != 4294967295) && ((((millis() - landed_at_time) / 10000) % 2) == 0)){ 
    if (osd_clear == 0){
       osd.clear();
       osd_clear = 1;
       force_redraw = 1;
    }
    panFdata(); 
  }else{ 
    if(ISd(0,Warn_BIT)) panWarn(panWarn_XY[0][0], panWarn_XY[1][0]); // this must be here so warnings are always checked
    //Check for panel toggle
    if(ch_toggle > 3) panOff(); // This must be first so you can always toggle
    if (osd_clear == 1){
      osd.clear();
      osd_clear = 0;
      force_redraw = 1;
    }
    if(panel != npanels){
      //Testing bits from 8 bit register A 
      //if(ISa(panel,Cen_BIT)) panCenter(panCenter_XY[0][panel], panCenter_XY[1][panel]);   //4x2
      if(ISa(panel,Pit_BIT)) panPitch(panPitch_XY[0][panel], panPitch_XY[1][panel]); //5x1
      if(ISa(panel,Rol_BIT)) panRoll(panRoll_XY[0][panel], panRoll_XY[1][panel]); //5x1
      if(ISa(panel,BatA_BIT)) panBatt_A(panBatt_A_XY[0][panel], panBatt_A_XY[1][panel]); //7x1
      //if(ISa(panel,BatB_BIT)) panBatt_B(panBatt_B_XY[0], panBatt_B_XY[1][panel]); //7x1
      if(ISa(panel,GPSats_BIT)) panGPSats(panGPSats_XY[0][panel], panGPSats_XY[1][panel]); //5x1
      //if(ISa(panel,GPL_BIT)) panGPL(panGPL_XY[0][panel], panGPL_XY[1][panel]); //2x1
      if(ISa(panel,GPS_BIT)) panGPS(panGPS_XY[0][panel], panGPS_XY[1][panel]); //12x3
      if(ISa(panel,Bp_BIT)) panBatteryPercent(panBatteryPercent_XY[0][panel], panBatteryPercent_XY[1][panel]); //
      if(ISa(panel,COG_BIT)) panCOG(panCOG_XY[0][panel], panCOG_XY[1][panel]); //

      //Testing bits from 8 bit register B
      if(ISb(panel,Rose_BIT)) panRose(panRose_XY[0][panel], panRose_XY[1][panel]);        //13x3
      if(ISb(panel,Head_BIT)) panHeading(panHeading_XY[0][panel], panHeading_XY[1][panel]); //13x3
      //if(ISb(panel,MavB_BIT)) panMavBeat(panMavBeat_XY[0][panel], panMavBeat_XY[1][panel]); //13x3
      if(osd_got_home == 1){
        if(ISb(panel,HDis_BIT)) panHomeDis(panHomeDis_XY[0][panel], panHomeDis_XY[1][panel]); //13x3
        if(ISb(panel,HDir_BIT)) panHomeDir(panHomeDir_XY[0][panel], panHomeDir_XY[1][panel]); //13x3
      }
      if(ISb(panel,Time_BIT)) panTime(panTime_XY[0][panel], panTime_XY[1][panel]);
      if(ISb(panel,WDis_BIT)) panWPDis(panWPDis_XY[0][panel], panWPDis_XY[1][panel]); //??x??

      //Testing bits from 8 bit register C 
      //if(osd_got_home == 1){
      if(ISc(panel,Alt_BIT)) panAlt(panAlt_XY[0][panel], panAlt_XY[1][panel]); //
      if(ISc(panel,Halt_BIT)) panHomeAlt(panHomeAlt_XY[0][panel], panHomeAlt_XY[1][panel]); //
      if(ISc(panel,Vel_BIT)) panVel(panVel_XY[0][panel], panVel_XY[1][panel]); //
      if(ISc(panel,As_BIT)) panAirSpeed(panAirSpeed_XY[0][panel], panAirSpeed_XY[1][panel]); //
      //}
      if(ISc(panel,Thr_BIT)) panThr(panThr_XY[0][panel], panThr_XY[1][panel]); //
      if(ISc(panel,FMod_BIT)) panFlightMode(panFMod_XY[0][panel], panFMod_XY[1][panel]);  //
      if(ISc(panel,Hor_BIT)) panHorizon(panHorizon_XY[0][panel], panHorizon_XY[1][panel]); //14x5
      if(ISc(panel,CurA_BIT)) panCur_A(panCur_A_XY[0][panel], panCur_A_XY[1][panel]);

      //Testing bits from 8 bit register D 
      //if(ISd(Off_BIT)) panOff(panOff_XY[0], panOff_XY[1]);
      //For now we don't have windspeed in copter
      //if(ISd(panel,WindS_BIT)) panWindSpeed(panWindSpeed_XY[0][panel], panWindSpeed_XY[1][panel]);
      if(ISd(panel,Climb_BIT)) panClimb(panClimb_XY[0][panel], panClimb_XY[1][panel]);
      //if(ISd(panel,Tune_BIT)) panTune(panTune_XY[0][panel], panTune_XY[1][panel]);
      if(ISd(panel,RSSI_BIT)) panRSSI(panRSSI_XY[0][panel], panRSSI_XY[1][panel]); //??x??
      if(ISd(panel,Eff_BIT)) panEff(panEff_XY[0][panel], panEff_XY[1][panel]);
      //if(ISd(panel,CALLSIGN_BIT)) panCALLSIGN(panCALLSIGN_XY[0][panel], panCALLSIGN_XY[1][panel]);
      if(ISe(panel,TEMP_BIT)) panTemp(panTemp_XY[0][panel], panTemp_XY[1][panel]);
      //if(ISe(panel,Ch_BIT)) panCh(panCh_XY[0][panel], panCh_XY[1][panel]);
      if(ISe(panel,DIST_BIT)) panDistance(panDistance_XY[0][panel], panDistance_XY[1][panel]);
      if(ISe(panel,RPM_BIT)) panRPM(panRPM_XY[0][panel], panRPM_XY[1][panel]);
    }
    //else { //panel == npanels
      //if(ISd(0,Warn_BIT)) panWarn(panWarn_XY[0][0], panWarn_XY[1][0]); // this must be here so warnings are always checked
      //if(ISd(0,CALLSIGN_BIT)) panCALLSIGN(panCALLSIGN_XY[0][panel], panCALLSIGN_XY[1][panel]); //call sign even in off panel
    //}
  }
/*  } else { // if no mavlink update for 2 secs
    
        // this could be replaced with a No Mavlink warning so the last seen values still show

        osd.clear();
        force_redraw = 1;
        waitingMAVBeats = 1;
        // Display our logo and wait... 
    //    panWaitMAVBeats(5,10); //Waiting for MAVBeats...
    panLogo();
  }*/
  if(ISd(panel % npanels,CALLSIGN_BIT)) panCALLSIGN(panCALLSIGN_XY[0][panel], panCALLSIGN_XY[1][panel]); //call sign even in off panel
#if 0
  static uint16_t last_ts = 0;
  uint16_t new_ts = millis();
  uint16_t period = new_ts - last_ts;
  last_ts = new_ts;
  osd.setPanel(1, 13);
  osd.openPanel();
  if (period > 10.1f)
      osd.write_num(1000.0f / period, 1, 3, 0); /* FPS */
  osd.closePanel();
#endif
  force_redraw = 0;
  
    // OSD debug for development (Shown on top-middle panels) 
#ifdef membug
    osd.setPanel(13,4);
    osd.openPanel();
    osd.printf("%i",freeMem()); 
    osd.closePanel();
#endif
}

/******* PANELS - DEFINITION *******/
/* **************************************************************** */

#define REDRAW_CHECK(val) \
    static typeof(val) last_val = (typeof(val)) -1; \
\
    if (val == last_val && !force_redraw) \
        return; \
    last_val = val;

#define REDRAW_CHECKF(val) \
    static float last_val = -1; \
\
    if (fabs(val - last_val) < 0.001f && !force_redraw) \
        return; \
    last_val = val;

/* **************************************************************** */
// Panel  : COG Course Over Ground
// Needs  : X, Y locations
// Output : 
// Size   : 
// Staus  : done

void panCOG(int first_col, int first_line){
    int16_t off_course = (osd_cog / 100 - osd_heading) ; //[-360, 360]
    int8_t osd_COG_arrow_rotate_int = ((int) round((float) (off_course * (16.0f / 360.0f))) + 16) % 16 + 1; //[1, 16]

    if (off_course > 180){
       off_course += - 360;
    }else if (off_course < -180){
       off_course += + 360;
    }
    if (osd_groundspeed < 0.2f)
        osd_COG_arrow_rotate_int = 0;

    REDRAW_CHECK(osd_COG_arrow_rotate_int);

    osd.setPanel(first_col, first_line);
    osd.openPanel();

    if (osd_COG_arrow_rotate_int) {
        showArrow(osd_COG_arrow_rotate_int);
        osd.write_num(off_course, 0, 4, 0);
        osd.write(0x05);
    } else {
        osd.write(' ');
        osd.write(' ');
        osd.write(' ');
        osd.write(' ');
        osd.write(' ');
        osd.write(' ');
        osd.write(' ');
    }
    osd.closePanel();
}

// Panel  : ODO
// Needs  : X, Y locations
// Output : 
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panDistance(int first_col, int first_line){
    REDRAW_CHECKF(tdistance * converth);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //do_converts();
    if ((tdistance * converth) > 1000.0) {
    osd.printf("%c%5.2f%c", 0x8F, (double) ((tdistance * converth) / distconv), distchar);
    }else{
    osd.printf("%c%5.0f%c", 0x8F, (double) (tdistance * converth), high);
    }
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panFdata
// Needs  : X, Y locations
// Output : 
// Size   : 
// Staus  : done
void panFdata(){
     osd.setPanel(11, 4);
    osd.openPanel();                          
//    osd.printf("%c%3i%c%02i|%c%5.0f%c|%c%5.0f%c|%c%5.0f%c|%c%5.0f%c|%c%5.0f%c|%c%5.0f%c", 0x08,((int)start_Time/60)%60,0x3A,(int)start_Time%60, 0x0B, ((max_home_distance) * converth), high, 0x1B, ((tdistance) * converth), high, 0x13,(max_osd_airspeed * converts), spe,0x14,(max_osd_groundspeed * converts),spe,0x12, (max_osd_home_alt * converth), high,0x1D,(max_osd_windspeed * converts),spe);
//    osd.printf("%c%3i%c%02i|%c%5i%c|%c%5i%c|%c%5i%c|%c%5i%c|%c%5i%c|%c%5i%c|%c%10.6f|%c%10.6f", 0x08,((int)start_Time/60)%60,0x3A,(int)start_Time%60, 0x0B, (int)((max_home_distance) * converth), high, 0x8F, (int)((tdistance) * converth), high, 0x13,(int)(max_osd_airspeed * converts), spe,0x14,(int)(max_osd_groundspeed * converts),spe,0x12, (int)(max_osd_home_alt * converth), high,0x1D,(int)(max_osd_windspeed * converts),spe, 0x03, (double)osd_lat, 0x04, (double)osd_lon);
    osd.printf("%c%3i%c%02i|%c%5i%c|%c%5i%c|%c%5i%c|%c%5i%c|%c%10.6f|%c%10.6f", 0x08,((int)start_Time/60)%60,0x3A,(int)start_Time%60, 0x0B, (int)((max_home_distance) * converth), high, 0x8F, (int)((tdistance) * converth), high,0x14,(int)(max_osd_groundspeed * converts),spe,0x12, (int)(max_osd_home_alt * converth), high, 0x03, (double)osd_lat, 0x04, (double)osd_lon);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : pantemp
// Needs  : X, Y locations
// Output : 
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panTemp(int first_col, int first_line){
    REDRAW_CHECK(temperature);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //do_converts();
    osd.printf("%c%5.1f%c", 0x0a, (double) (float(temperature / 10 * tempconv + tempconvAdd) / 100), temps);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : efficiency
// Needs  : X, Y locations
// Output : 
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panEff(int first_col, int first_line){
    //Check takeoff just to prevent inicial false readings
    if (takeofftime){
        ////If in loiter should estimated remaining flight time
        //if ((osd_climb > -0.05) && (osd_climb < 0.05) && (osd_groundspeed * converts < 2)){ 
          if(osd_battery_remaining_A != last_battery_reading){
            remaining_Time = osd_battery_remaining_A * ((millis()/1000) - FTime) / (start_battery_reading - osd_battery_remaining_A);
            last_battery_reading = osd_battery_remaining_A;
          }
          REDRAW_CHECK((int) remaining_Time);
          osd.setPanel(first_col, first_line);
          osd.openPanel();
          osd.printf("%c%2i%c%02i", 0x17,((int)remaining_Time/60)%60,0x3A,(int)remaining_Time%60);
        //}
        //If in movement show mAh needed to fly a Km or a mile (depending on selected unit
//        else{
//          eff = (float(osd_curr_A * 10) / (osd_groundspeed * converts))* 0.5 + eff * 0.5;
//        eff = eff * 0.2 + eff * 0.8;
//          osd.setPanel(first_col, first_line);
//          osd.openPanel();
//          if (eff > 0 && eff <= 9999) {
//            osd.printf("%c%4.0f%c", 0x17, (double)eff, 0x82);
//          }else{
//          osd.printf_P(PSTR("\x17\x20\x20\x20\x20\x20")); 
//          }
        //}
    }
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panCh
// Needs  : X, Y locations
// Output : Scaled channel values from MAVLink
// Size   
// Staus  : done

//void panCh(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
    
//    osd.printf("%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i", 0x43, 0x31, chan1_raw, 0x43, 0x32, chan2_raw, 0x43, 0x33, chan3_raw, 0x43, 0x34, chan4_raw, 0x43, 0x35, chan5_raw, 0x43, 0x36, chan6_raw); 
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panRSSI
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panRSSI(int first_col, int first_line){
    rssi = (int16_t) osd_rssi;
    REDRAW_CHECK(rssi);

    //if (rssi > rssical) rssi = rssical;
    //else if (rssi < rssipersent) rssi = rssipersent;

    if(!rssiraw_on) rssi = (int16_t)((float)(rssi - rssipersent)/(float)(rssical-rssipersent)*100.0f);
//    if (rssi < -99) rssi = -99;

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3i%c", 0x09, rssi, 0x25);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panCALLSIGN
// Needs  : X, Y locations
// Output : Call sign identification
// Size   : 1 x 6Hea  (rows x chars)
// Staus  : done

void panCALLSIGN(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //osd.printf("%c%c%c%c%c%c", char_call[0], char_call[1], char_call[2], char_call[3], char_call[4], char_call[5]); 
    //During the first 1000 miliseconds of each minute show callsign
    if(((millis() / 1000) % 60) < 2)
      osd.printf("%s", char_call);
    else
//    osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20"));
      osd.printf("%s",strclear);
    
/*    if ((millis() - 60000) > CallSignBlink){
      if (millis() - 61000 > CallSignBlink){
        CallSignBlink = (millis() - 1000);
          }
    osd.printf("%s", char_call); 
    }else{
//    osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20"));
    osd.printf("%s",strclear);
    }*/
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panSetup
// Needs  : Nothing, uses whole screen
// Output : The settings menu
// Size   : 3 x ?? (rows x chars)
// Staus  : done

//void panSetup(){

//    if (millis() > text_timer){
//        text_timer = millis() + 500;

//        osd.clear();
//        osd.setPanel(5, 7);
//        osd.openPanel();

//        if (chan1_raw_middle == 0 && chan2_raw_middle == 0){
//            chan1_raw_middle = chan1_raw;
//            chan2_raw_middle = chan2_raw;
//        }

//        if ((chan2_raw - 100) > chan2_raw_middle ) setup_menu++;  //= setup_menu + 1;
//        else if ((chan2_raw + 100) < chan2_raw_middle ) setup_menu--;  //= setup_menu - 1;
//        if (setup_menu < 0) setup_menu = 0;
//        else if (setup_menu > 2) setup_menu = 2;


//        switch (setup_menu){
//        case 0:
//            {
//                osd.printf_P(PSTR("    Overspeed    "));
//                osd.printf("%3.0i%c", overspeed, spe);
//                overspeed = change_val(overspeed, overspeed_ADDR);
//                break;
//            }
//        case 1:
//            {
//                osd.printf_P(PSTR("   Stall Speed   "));
//                osd.printf("%3.0i%c", stall , spe);
//                //overwritedisplay();
//                stall = change_val(stall, stall_ADDR);
//                break;
//            }
//        case 2:
//            {
//                osd.printf_P(PSTR("Battery warning "));
//                osd.printf("%3.1f%c", float(battv)/10.0 , 0x76, 0x20);
//                battv = change_val(battv, battv_ADDR);
//                break;
//            }
            //      case 4:
            //        osd.printf_P(PSTR("Battery warning "));
            //        osd.printf("%3.0i%c", battp , 0x25);
            //        if ((chan1_raw - 100) > chan1_raw_middle ){
            //        battp = battp - 1;}
            //        if ((chan1_raw + 100) < chan1_raw_middle ){
            //        battp = battp + 1;} 
            //        EEPROM.write(208, battp);
            //        break;
//        }
//}
//    osd.closePanel();
//}

//int change_val(int value, int address)
//{
//    uint8_t value_old = value;
//    if (chan1_raw > chan1_raw_middle + 100) value--;
//    if (chan1_raw  < chan1_raw_middle - 100) value++;

//    if(value != value_old && setup_menu ) EEPROM.write(address, value);
//    return value;
//}

/* **************************************************************** */
// Panel  : pan wind speed
// Needs  : X, Y locations
// Output : Wind direction symbol (arrow) and velocity
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panWindSpeed(int first_col, int first_line){
    //osd_wind_arrow_rotate_int = round((osd_winddirection - osd_heading)/360.0 * 16.0) + 1; //Convert to int 1-16
    //if(osd_wind_arrow_rotate_int < 0 ) osd_wind_arrow_rotate_int += 16; //normalize
    //else if(osd_wind_arrow_rotate_int == 0 ) osd_wind_arrow_rotate_int = 1; //normalize
    //else if(osd_wind_arrow_rotate_int == 17 ) osd_wind_arrow_rotate_int = 1; //normalize
    
    osd_wind_arrow_rotate_int = ((int)round((osd_winddirection - osd_heading)/360.0 * 16.0) + 16) % 16 + 1; //[1, 16]
    nor_osd_windspeed = osd_windspeed * 0.010 + nor_osd_windspeed * 0.990;     

    REDRAW_CHECK((uint8_t) osd_wind_arrow_rotate_int);

    osd.setPanel(first_col, first_line);
    osd.openPanel();

    osd.printf("%c%3.0f%c", 0x1d, (double) (osd_windspeed * converts), spe);
    showArrow((uint8_t) osd_wind_arrow_rotate_int); // print data to OSD
    osd.printf("%2.0f%c", (double) (nor_osd_windspeed * converts), spe);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panOff
// Needs  : X, Y locations
// Output : OSD off
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panOff(){
  bool rotatePanel = 0;

  //If there is a warning force switch to panel 0
  if(foundWarning == 1){
    if(panel != 0){
      //osd.clear();
      osd_clear = 1;
    }
    panel = 0;
  }
  else{
    //Flight mode switching
    if (ch_toggle == 4){
      if ((osd_mode != 6) && (osd_mode != 7)){
        if (osd_off_switch != osd_mode){ 
          osd_off_switch = osd_mode;
            osd_switch_time = millis();
            if (osd_off_switch == osd_switch_last){
              rotatePanel = 1;
            }
        }
        if ((millis() - osd_switch_time) > 2000){
          osd_switch_last = osd_mode;
        }
      }
    }
    else {
      if(ch_toggle == 5) ch_raw = chan5_raw;
      else if(ch_toggle == 6) ch_raw = chan6_raw;
      else if(ch_toggle == 7) ch_raw = chan7_raw;
      else if(ch_toggle == 8) ch_raw = chan8_raw;

      //Switch mode by value
      if (switch_mode == 0){
        //First panel
        if (ch_raw < 1200 && panel != 0) {
          osd_clear = 1;
          //osd.clear();
          panel = 0;
        }
        //Second panel
        else if (ch_raw >= 1200 && ch_raw <= 1800 && panel != 1) { //second panel
          osd_clear = 1;
          //osd.clear();
          panel = 1;
        }
        //Panel off
        else if (ch_raw > 1800 && panel != npanels) {
          osd_clear = 1;
          //osd.clear();
          panel = npanels; //off panel
        }
      }
      //Rotation switch
      else{
        if (ch_raw > 1200)
          if (osd_switch_time + 1000 < millis()){
            rotatePanel = 1;
            osd_switch_time = millis();
        }
      }    
    }
    if(rotatePanel == 1){
      osd_clear = 1;
      //osd.clear();
      panel++;
      if (panel > npanels)
        panel = 0;
    }
  }
}
//* **************************************************************** */
// Panel  : panTune
// Needs  : X, Y locations
// Output : Current symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done
    
//  void panTune(int first_col, int first_line){
//  osd.setPanel(first_col, first_line);
//  osd.openPanel();

//  osd.printf("%c%c%2.0f%c|%c%c%2.0f%c|%c%c%4.0i%c|%c%c%4.0i%c|%c%c%3.0f%c|%c%c%3.0f%c|%c%c%3.0f%c", 0x4E, 0x52, (nav_roll), 0x05, 0x4E, 0x50, (nav_pitch), 0x05, 0x4E, 0x48, (nav_bearing), 0x05, 0x54, 0x42, (wp_target_bearing), 0x05, 0x41, 0x45, (alt_error * converth), high, 0x58, 0x45, (xtrack_error), 0x6D, 0x41, 0x45, ((aspd_error / 100.0) * converts), spe);

//  osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panCur_A
// Needs  : X, Y locations
// Output : Current symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panCur_A(int first_col, int first_line){
    REDRAW_CHECK(osd_curr_A);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%.1f%c", 0xBD, (float(osd_curr_A) * .01), 0x0E);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panAlt
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 2 x 7Hea  (rows x chars)
// Staus  : done

void panAlt(int first_col, int first_line){
    REDRAW_CHECK((int16_t) (osd_alt - osd_home_alt));

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //osd.printf("%c%5.0f%c",0x11, (double)(osd_alt - osd_home_alt), 0x0C);
    //osd.printf("%c%5.0f%c",0x11, (double)(osd_gps_alt * converth), high);
    osd.printf("%4.1f%c%c |+%3i%c%c ",
        (double) (osd_alt - osd_home_alt), 0x0c, 0x12,
        (int) osd_home_alt, 0x0c, 0x11);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panClimb
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panClimb(int first_col, int first_line){
    REDRAW_CHECK((int8_t) (10.0f * osd_climb));

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //vs = (osd_climb * converth * 60) * 0.1 + vs * 0.9;
    //osd.printf("%5.1f%c", (double) osd_climb, climbchar);
    osd.write_num(osd_climb, 1, 0, 0);
    osd.write(climbchar);
    osd.write(' ');
    osd.write(' ');
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHomeAlt
// Needs  : X, Y locations
// Output : Alt symbol and home altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panHomeAlt(int first_col, int first_line){
    REDRAW_CHECK((int16_t) osd_home_alt);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
//    osd.printf("+%i%c%c", (int) osd_home_alt, 0x0c, 0x11);
    osd.write_num(osd_home_alt, 0, 0, '+');
    osd.write(0x0c);
    osd.write(0x11);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panVel
// Needs  : X, Y locations
// Output : Velocity value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panVel(int first_col, int first_line){
    uint16_t val = osd_groundspeed * converts;
    if (val > 999)
        val = 999;

    REDRAW_CHECK(val);

    osd.setPanel(first_col, first_line);
    osd.openPanel();

    osd.write(0x14); /* GS symbol */
    osd.write_num(val, 0, 3, 0);
    osd.write(spe);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panAirSpeed
// Needs  : X, Y locations
// Output : Airspeed value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panAirSpeed(int first_col, int first_line){
    uint16_t val = osd_airspeed * converts;
    if (val > 999)
        val = 999;

    REDRAW_CHECK(val);

    osd.setPanel(first_col, first_line);
    osd.openPanel();

    osd.write(0x13); /* Airspeed symbol */
    osd.write_num(val, 0, 3, 0);
    osd.write(spe);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panWarn
// Needs  : X, Y locations
// Output : Warning strings depending on flags
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panWarn(int first_col, int first_line){
    int count = 0, warnings[5];
    static const char *texts[5] = {
        "  DISARMED  ",
        "  ACC RANGE ",
        "  MAG RANGE ",
        " !NOSIGNAL! ",
    };
    const char *val;

    if (!motor_armed)
        warnings[count++] = 0;
    if (!acc_valid)
        warnings[count++] = 1;
    if (!mag_valid)
        warnings[count++] = 2;
    if (nosignal)
        warnings[count++] = 3;

    if (count)
        val = texts[warnings[(millis() / 500) % count]];
    else
        val = "            ";

    REDRAW_CHECK(val); /* Pointer compare, but fine here */

    osd.setPanel(first_col, first_line);
    osd.openPanel();

    osd.printf(val);

    osd.closePanel();
}

  
/* **************************************************************** */
// Panel  : panThr
// Needs  : X, Y locations
// Output : Throttle value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panThr(int first_col, int first_line){
    REDRAW_CHECK(osd_throttle);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0i%c",0x02,osd_throttle,0x25);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panBatteryPercent
// Needs  : X, Y locations
// Output : Battery state from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panBatteryPercent(int first_col, int first_line){
    if (EEPROM.read(OSD_BATT_SHOW_PERCENT_ADDR) == 0) {
        REDRAW_CHECK((int) mah_used);
    } else {
        REDRAW_CHECK(osd_battery_remaining_A);
    }

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if (EEPROM.read(OSD_BATT_SHOW_PERCENT_ADDR) == 0){
        osd.printf("%c%4.0f%c",0x17, (double) mah_used, 0x01);
    }else{
        osd.printf("%c%3.0i%c", 0x17, osd_battery_remaining_A, 0x25);
    }
    osd.closePanel();
}
/* **************************************************************** */
// Panel : panTime
// Needs : X, Y locations
// Output : Time from start with symbols
// Size : 2 x 8 (rows x chars) //AJF
// Staus : done
void panTime(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    // The data is scaled in usec, and stored as a 64 bit value. We therefore need to divide by 1,000,000 to get to seconds.
    // Using the standard library 64 bit divide adds about 10kB to the code size and doesn't fit in the 328. Using this routine only 
    // adds about 1.3kB, and does fit (at least for this version of MinimOSD with a few panels disabled).
    //uint32_t local_osd_gps_time = Divide_1000000(osd_gps_time);
    //start_Time = (millis() / 1000) - FTime;
    //uint32_t local_time = (local_osd_gps_time + TIME_MAX_SECONDS + TIME_TIMEZONE * 3600) % TIME_MAX_SECONDS;
    //int t_sec = local_time % 60;
    //local_time /= 60;
    //int t_min = local_time % 60;
    //local_time /= 60;
    //int t_hr = local_time % 24;
    //osd.printf(" %c%2i%c%02i|", 0x08, ((int)start_Time / 60) % 60, 0x3A, (int)start_Time % 60);
    
    osd.printf("%llu", osd_gps_time);
    //if(local_osd_gps_time != 0) osd.printf("%02i%c%02i%c%02i", t_hr, 0x3A, t_min, 0x3A, t_sec);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHomeDis
// Needs  : X, Y locations
// Output : Home Symbol with distance to home in meters
// Size   : 1 x 6  (rows x chars)
// Staus  : done

void panHomeDis(int first_col, int first_line){
    REDRAW_CHECK((int) (osd_home_distance * converth));

    osd.setPanel(first_col, first_line);
    osd.openPanel();
//    osd.printf("%4i%c%c", (int)((osd_home_distance) * converth), high, 0x0b);
//    osd.printf("   0%c%c", high, 0x0b);
    if (osd_home_distance >= 5) {
        osd.write_num(osd_home_distance * converth, 0, 4, 0);
        osd.write(high);
        osd.write(0x0b);
    } else {
        int i;
        for (i = 0; i < 6; i++)
            osd.write(' ');
    }
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panCenter
// Needs  : X, Y locations
// Output : 2 row croshair symbol created by 2 x 4 chars
// Size   : 2 x 4  (rows x chars)
// Staus  : done

//void panCenter(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    osd.printf_P(PSTR("\x05\x03\x04\x05|\x15\x13\x14\x15"));
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panHorizon
// Needs  : X, Y locations
//// Output : 12 x 13 Horizon line surrounded by 2 cols (left/right rules)
// Size   : 14 x 13  (rows x chars)
// Staus  : done
#define AH_COLS			14			// number of artificial horizon columns
#define AH_ROWS			13			// number of artificial horizon rows
#define ILS_ROWS                8
void panHorizon(int first_col, int first_line) {
    int i, ilstop;
    static float last_pitch = -1, last_roll = -1;
    static uint8_t buffer2[AH_COLS * AH_ROWS];

    if (force_redraw || fabs(last_pitch - osd_pitch) > 0.1 ||
            fabs(last_roll - osd_roll) > 0.2) {
        uint8_t buffer1[AH_COLS * AH_ROWS];
        uint8_t j, line = first_line;

        for (j = 0; j < sizeof(buffer1); j++)
            buffer1[j] = 0;
        show_horizon(first_col, first_line, buffer1);

        for (j = 0; j < sizeof(buffer1); line++) {
            uint8_t k, prev = 0;
            
            for (k = 0; k < AH_COLS - 1; k++, j++)
                if (buffer1[j] != buffer2[j]) {
                    buffer2[j] = buffer1[j];
                    if (!prev) {
                        osd.setPanel(first_col + k, line);
                        osd.openPanel();
                        prev = j;
                    }
                    while (prev < j)
                        osd.write(buffer1[prev++]);
                    osd.write(buffer1[prev++]);
                }
            if (prev)
                osd.closePanel();
            j++;
        }

        last_pitch = osd_pitch;
        last_roll = osd_roll;
    }

    ilstop = min(first_line, (MAX7456_screen_rows - ILS_ROWS) / 2);
    i = min(ILS_ROWS, AH_ROWS);

    REDRAW_CHECK((int) ((osd_alt - osd_home_alt) * 10));

    showILS(first_col + AH_COLS - 1, ilstop, i);
}

/* **************************************************************** */
// Panel  : panPitch
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
// Size   : 1 x 6  (rows x chars)
// Staus  : done

void panPitch(int first_col, int first_line){
    REDRAW_CHECK((int8_t) osd_pitch);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4.0f%c%c",osd_pitch,0x05,0x07);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRoll
// Needs  : X, Y locations
// Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
// Size   : 1 x 6  (rows x chars)
// Staus  : done

void panRoll(int first_col, int first_line){
    REDRAW_CHECK((int8_t) osd_roll);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4.0f%c%c",osd_roll,0x05,0x06);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRPM
// Needs  : X, Y locations
// Output : RPM values for each motor
// Size   : 4 x 11  (rows x chars)
// Staus  : done

void panRPM(int first_col, int first_line){
    int i;
    static uint8_t last_val[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };

    for (i = 0; i < 4; i ++) {
        if (osd_rpm[i] - osd_esctemp[i] == last_val[i] && !force_redraw)
            continue;
        last_val[i] = osd_rpm[i] - osd_esctemp[i];

        osd.setPanel(first_col, first_line + i);
        osd.openPanel();
        osd.write_num(osd_rpm[i], 0, 5, 0);
        osd.write(0x0a);
        //osd.write(' ');
        osd.write_num((osd_esctemp[i] * tempconv + tempconvAdd * 10) *
            0.001f, 1, 0, 0);
        osd.write(temps);
        osd.closePanel();
    }
}

/* **************************************************************** */
// Panel  : panBattery A (Voltage 1)
// Needs  : X, Y locations
// Output : Voltage value as in XX.X and symbol of over all battery status
// Size   : 1 x 8  (rows x chars)
// Staus  : done

void panBatt_A(int first_col, int first_line){
    REDRAW_CHECKF(osd_vbat_A);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    /*************** This commented code is for the next ArduPlane Version
    if(osd_battery_remaining_A > 100){
        osd.printf(" %c%5.2f%c", 0xBD, (double)osd_vbat_A, 0xC9);
    else osd.printf("%c%5.2f%c%c", 0xBD, (double)osd_vbat_A, 0xC9, osd_battery_pic_A);
    */
//    osd.printf("%5.2f%c", (double)osd_vbat_A, 0x0D);
    osd.write_num(osd_vbat_A, 1, 3, 0);
    osd.write(0x0d);
    osd.closePanel();
}

//------------------ Panel: Waiting for MAVLink HeartBeats -------------------------------

//void panWaitMAVBeats(int first_col, int first_line){
//    panLogo();
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    osd.printf_P(PSTR("Waiting for|MAVLink heartbeats..."));
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panGPL
// Needs  : X, Y locations
// Output : 1 static symbol with changing FIX symbol
// Size   : 1 x 2  (rows x chars)
// Staus  : done

void panGPL(int first_col, int first_line){
    REDRAW_CHECK(osd_fix_type);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    const char *gps_str;
    if(osd_fix_type == 0 || osd_fix_type == 1) gps_str = "\x1F\x20"; 
        //osd.printf_P(PSTR("\x1F\x20"));
    else if(osd_fix_type == 2 || osd_fix_type == 3) gps_str = "\x0F\x20";
        //osd.printf_P(PSTR("\x0F\x20"));
    osd.printf("%s",gps_str);

    /*  if(osd_fix_type <= 1) {
    osd.printf_P(PSTR("\x1F"));
    } else {
    osd.printf_P(PSTR("\x0F"));
    }  */
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panGPSats
// Needs  : X, Y locations
// Output : 1 symbol and number of locked satellites
// Size   : 1 x 5  (rows x chars)
// Staus  : done

void panGPSats(int first_col, int first_line){
    REDRAW_CHECK((uint8_t) (osd_satellites_visible + osd_fix_type * 8));

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    
    char gps_chr;
    if (osd_fix_type == 0 || osd_fix_type == 1)
      gps_chr = 0x1f;       
    else if(osd_fix_type == 2 || osd_fix_type == 3)
      gps_chr = 0x0f;
    
    osd.write(gps_chr);
    osd.write_num(osd_satellites_visible, 0, 2, 0);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panGPS
// Needs  : X, Y locations
// Output : two row numeric value of current GPS location with LAT/LON symbols as on first char
// Size   : 2 x 12  (rows x chars)
// Staus  : done

void panGPS(int first_col, int first_line){
    /* TODO: check diff */
    int width = 6;
    if (osd_lat >= 10.0 || osd_lat < 0.0 || osd_lon >= 10.0 ||
            osd_lon < 0.0) {
        width += 1;
        if (osd_lat <= -10.0 || osd_lon >= 100.0 ||
                osd_lon <= -10.0) {
            width += 1;
            if (osd_lon <= -100.0)
                width += 1;
        }
    }
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.write(0x03);
    osd.write_num(osd_lat, 5, width, 0); /* FPS */
    osd.write('|');
    osd.write(0x04);
    osd.write_num(osd_lon, 5, width, 0); /* FPS */
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHeading
// Needs  : X, Y locations
// Output : Symbols with numeric compass heading value
// Size   : 1 x 5  (rows x chars)
// Staus  : not ready

void panHeading(int first_col, int first_line){
    REDRAW_CHECK((uint8_t) osd_heading);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.write_num(osd_heading, 0, 4, 0);
    osd.write(0x05);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRose
// Needs  : X, Y locations
// Output : a dynamic compass rose that changes along the heading information
// Size   : 2 x 16  (rows x chars)
// Staus  : done

void panRose(int first_col, int first_line){
    REDRAW_CHECK((uint16_t) (osd_heading * 10));

    setHeadingPattern();

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //osd_heading  = osd_yaw;
    //if(osd_yaw < 0) osd_heading = 360 + osd_yaw;
    //osd.printf("%s|%c%s%c", "\x20\x80\x80\x80\x80\x80\x81\x80\x80\x80\x80\x80\x20", 0xc3, buf_show, 0x87);
    osd.printf("%s|       ^", buf_show);
    osd.closePanel();
}


/* **************************************************************** */
// Panel  : panBoot
// Needs  : X, Y locations
// Output : Booting up text and empty bar after that
// Size   : 1 x 21  (rows x chars)
// Staus  : done

/*void panBoot(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("Booting up:\x88\x8D\x8D\x8D\x8D\x8D\x8D\x8D\x8E")); 
    osd.closePanel();
}*/

/* **************************************************************** */
// Panel  : panMavBeat
// Needs  : X, Y locations
// Output : 2 symbols, one static and one that blinks on every 50th received 
//          mavlink packet.
// Size   : 1 x 2  (rows x chars)
// Staus  : done

//void panMavBeat(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    if(mavbeat == 1){
//        osd.printf_P(PSTR("\xEA\xEC"));
//        mavbeat = 0;
//    }
//    else{
//        osd.printf_P(PSTR("\xEA\xEB"));
//    }
//    osd.closePanel();
//}


/* **************************************************************** */
// Panel  : panWPDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to next waypoint
// Size   : 1 x 2  (rows x chars)
// Staus  : not ready

//void panWPDir(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
   
//    wp_target_bearing_rotate_int = round(((float)wp_target_bearing - osd_heading)/360.0 * 16.0) + 1; //Convert to int 0-16 
//    if(wp_target_bearing_rotate_int < 0 ) wp_target_bearing_rotate_int += 16; //normalize  

//    showArrow((uint8_t)wp_target_bearing_rotate_int);
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panWPDis
// Needs  : X, Y locations
// Output : W then distance in Km - Distance to next waypoint
// Size   : 1 x 2  (rows x chars)
// Staus  : not ready TODO - CHANGE the Waypoint symbol - Now only a W!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void panWPDis(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    
    //wp_target_bearing_rotate_int = (int)round(((float)wp_target_bearing - osd_heading)/360.0 * 16.0); //[-16, 16]
    //if(wp_target_bearing_rotate_int < 0 ) wp_target_bearing_rotate_int += 16; //normalize [0, 16]
    //wp_target_bearing_rotate_int = wp_target_bearing_rotate_int % 16 + 1; //Convert to [1, 16] 
    wp_target_bearing_rotate_int = ((int)round(((float)wp_target_bearing - osd_heading)/360.0 * 16.0) + 16) % 16 + 1; //[1, 16]
    
    if (xtrack_error > 999) xtrack_error = 999;
    else if (xtrack_error < -999) xtrack_error = -999;

    osd.printf("%c%c%2i%c%4.0f%c|",0x57, 0x70, wp_number,0x0,(double)((float)(wp_dist) * converth),high);
    showArrow((uint8_t)wp_target_bearing_rotate_int);
    if (osd_mode == 10){
        osd.printf("%c%c%c%4.0f%c", 0x20, 0x58, 0x65, (double) (xtrack_error* converth), high);
    }else{
        osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20"));
    }
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHomeDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to home
// Size   : 1 x 2  (rows x chars)
// Status : not tested

void panHomeDir(int first_col, int first_line){
    uint8_t arrow_idx;
    int rel;

    if (!osd_got_home)
        return;

    rel = osd_home_direction + 360 - osd_heading; // relative home direction
    arrow_idx = (rel * 16 + 7) / 360 % 16 + 1;//array of arrows =)
    if (osd_home_distance < 5)
        arrow_idx = 0;

    REDRAW_CHECK(arrow_idx);

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if (arrow_idx)
        showArrow(arrow_idx);
    else {
        osd.write(' ');
        osd.write(' ');
    }
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panFlightMode 
// Needs  : X, Y locations
// Output : 2 symbols, one static name symbol and another that changes by flight modes
// Size   : 1 x 2  (rows x chars)
// Status : done

void panFlightMode(int first_col, int first_line){
    REDRAW_CHECK((uint8_t) (osd_mode + motor_armed * 16));

    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //char c1 = 0xE0 ;//"; char c2; char c3; char c4; char c5; 
    const char* mode_str="";
    if (osd_mode == 0) mode_str = "stab"; //Stabilize
    else if (osd_mode == 1) mode_str = "acro"; //Acrobatic
    else if (osd_mode == 2) mode_str = "alth"; //Alt Hold
    else if (osd_mode == 3) mode_str = "auto"; //Auto
    else if (osd_mode == 4) mode_str = "guid"; //Guided
    else if (osd_mode == 5) mode_str = "loit"; //Loiter
    else if (osd_mode == 6) mode_str = "retl"; //Return to Launch
    else if (osd_mode == 7) mode_str = "circ"; //Circle
    else if (osd_mode == 8) mode_str = "posi"; //Position
    else if (osd_mode == 9) mode_str = "land"; //Land
    else if (osd_mode == 10) mode_str = "oflo"; //OF_Loiter
    osd.printf("%c%s%c", 0x7F, mode_str, motor_armed * 0x86);
    osd.closePanel();
}


// ---------------- EXTRA FUNCTIONS ----------------------
// Show those fancy 2 char arrows
void showArrow(uint8_t rotate_arrow) {
    int arrow_set1 = 0x90;
    // We trust that we receive rotate_arrow [1, 16] so
    // it's no needed (rotate_arrow <= 16) in the if clause
    arrow_set1 += rotate_arrow * 2 - 2;
    osd.write(arrow_set1);
    osd.write(arrow_set1 + 1);
}

// Calculate and shows Artificial Horizon
// For using this, you must load a special mcm file with the new staggered artificial horizon chars!
// e.g. AH_BetterResolutionCharset002.mcm
							// with different factors we can adapt do different cam optics
#define CHAR_COLS		12			// number of MAX7456 char columns
#define CHAR_ROWS		18			// number of MAX7456 char rows
#define CHAR_SPECIAL		9			// number of MAX7456 special chars for the artificial horizon
#define AH_TOTAL_LINES		AH_ROWS * CHAR_ROWS	// helper define
#define CHAR_SPECIAL_HORIZ	18                      // special case for the horizontal bar char

#define LINE_SET_STRAIGHT	(0xbe)		        // code of the first MAX7456 straight char
#define LINE_SET_P___STAG_1	(0xD1 - 1)		// code of the first MAX7456 positive staggered set 1 char -1
#define LINE_SET_P___STAG_2	(0xDA - 1)		// code of the first MAX7456 positive staggered set 2 char -1
#define LINE_SET_N___STAG_1	(0xE3 - 1)		// code of the first MAX7456 negative staggered set 1 char -1
#define LINE_SET_N___STAG_2	(0xEC - 1)		// code of the first MAX7456 negative staggered set 2 char -1
#define LINE_SET_P_O_STAG_1	(0xF5 - 2)		// code of the first MAX7456 positive overflow staggered set 1 char -2
#define LINE_SET_P_O_STAG_2	(0xF9 - 1)		// code of the first MAX7456 positive overflow staggered set 2 char -1
#define LINE_SET_N_O_STAG_1	(0xF7 - 2)		// code of the first MAX7456 negative overflow staggered set 1 char -2
#define LINE_SET_N_O_STAG_2	(0xFC - 1)		// code of the first MAX7456 negative overflow staggered set 2 char -1


#define OVERFLOW_CHAR_OFFSET	6			// offset for the overflow subvals


#define ANGLE_1			9			// angle above we switch to line set 1
#define ANGLE_2			25			// angle above we switch to line set 2
#define PITCH_0_Y		(MAX7456_screen_rows / 2 - 1)

static void inline ah_print_char(int col, int row,
        uint8_t *cache, uint8_t ch) {
    row = AH_ROWS - row - 1;
    cache[col + AH_COLS * row] = ch;
}

void horiz_line(int pitch_offset, int len, int start_col, int start_row,
        int line_set, int line_set_overflow, int subval_overflow,
        uint8_t *cache) {
    float rsin = sin(osd_roll / 180.0 * 3.1415);
    float rcos = cos(osd_roll / 180.0 * 3.1415);
    int x0 = (MAX7456_screen_cols - 2 * start_col) * CHAR_COLS / 2 +
        3.5 * (osd_pitch + pitch_offset) * rsin;
    float y0 = (start_row + AH_ROWS - PITCH_0_Y) * CHAR_ROWS -
        3.5 * (osd_pitch + pitch_offset) * rcos;
    int x1 = x0 - len * rcos;
    int x2 = x0 + len * rcos;
    float y1 = y0 - len * rsin;
    float yd = rsin / rcos;
    int minx = min(x1, x2);
    int maxx = max(x1, x2) / CHAR_COLS;

    for (int col = minx / CHAR_COLS; col <= maxx; col++) {
        if (col < 0 || col >= AH_COLS - 1)
            continue;
        int y = y1 + (col * CHAR_COLS - minx + CHAR_COLS / 2) * yd;
        if (y < 0 || y >= AH_TOTAL_LINES)
            continue;

        int row = y / CHAR_ROWS;

        if (line_set == LINE_SET_STRAIGHT) {
          int subval = (y - (row * CHAR_ROWS)) / (CHAR_ROWS / CHAR_SPECIAL_HORIZ);

	  // print the line char
          ah_print_char(col, row, cache, line_set + subval);

	  // check if we have to print an overflow line char
          if (subval >= CHAR_SPECIAL_HORIZ - 1 && row < AH_ROWS - 1) {
              ah_print_char(col, row + 1, cache,
                      line_set + subval - CHAR_SPECIAL_HORIZ);
          } else if (subval <= 0 && row) {
              ah_print_char(col, row - 1, cache,
                      line_set + subval + CHAR_SPECIAL_HORIZ);
          }
        } else {
          int subval = (y - (row * CHAR_ROWS)) / (CHAR_ROWS / CHAR_SPECIAL);

	  // print the line char
          ah_print_char(col, row, cache, line_set + subval + 1);

	  // check if we have to print an overflow line char
          if (subval >= subval_overflow && row < AH_ROWS - 1) {	// only if it is a char which needs overflow and if it is not the upper most row
              ah_print_char(col, row + 1, cache,
                      line_set_overflow + subval + 1 - OVERFLOW_CHAR_OFFSET);
          }
        }
    }
}

// Calculate and show artificial horizon
void show_horizon(int start_col, int start_row, uint8_t *cache) {
    int roll;
    int line_set = LINE_SET_STRAIGHT;
    int line_set_overflow = 0;
    int subval_overflow = 9;

    // preset the line char attributes
    roll = osd_roll;
    if ((roll >= 0 && roll < 90) || (roll >= -179 && roll < -90)) {	// positive angle line chars
	roll = roll < 0 ? roll + 179 : roll;
        if (abs(roll) > ANGLE_2) {
	    line_set = LINE_SET_P___STAG_2;
	    line_set_overflow = LINE_SET_P_O_STAG_2;
            subval_overflow = 7;
	} else if (abs(roll) > ANGLE_1) {
	    line_set = LINE_SET_P___STAG_1;
	    line_set_overflow = LINE_SET_P_O_STAG_1;
            subval_overflow = 8;
	}
    } else {					// negative angle line chars
	roll = roll > 90 ? roll - 179 : roll;
        if (abs(roll) > ANGLE_2) {
	    line_set = LINE_SET_N___STAG_2;
	    line_set_overflow = LINE_SET_N_O_STAG_2;
            subval_overflow = 7;
	} else if (abs(roll) > ANGLE_1) {
	    line_set = LINE_SET_N___STAG_1;
	    line_set_overflow = LINE_SET_N_O_STAG_1;
            subval_overflow = 8;
	}
    }

    horiz_line(0, 32, start_col, start_row, line_set, line_set_overflow,
            subval_overflow, cache);
    horiz_line(-22.5, 16, start_col, start_row, line_set, line_set_overflow,
            subval_overflow, cache);
    horiz_line(22.5, 16, start_col, start_row, line_set, line_set_overflow,
            subval_overflow, cache);
    horiz_line(-45, 16, start_col, start_row, line_set, line_set_overflow,
            subval_overflow, cache);
    horiz_line(45, 16, start_col, start_row, line_set, line_set_overflow,
            subval_overflow, cache);
}

// Calculates and shows verical speed aid
void showILS(int start_col, int start_row, int height) {
    char str[20], len;
    static char prev_len = 0;
#define UNIT_PER_CHAR 2.5f
#define CH_PER_LONGCHAR 4
#define ZERO_ALT_OFFSET (18 - 7) /* Pixel height of the cur.alt. marker triangle tip */
    uint16_t alt = (fmod(osd_alt - osd_home_alt, 100.0f) + 100.0f) /
        UNIT_PER_CHAR * CHAR_SPECIAL_HORIZ;

    for (int i = 0; i < height; i++) {
        uint16_t ch_alt = (MAX7456_screen_rows / 2 -
            (start_row + i) + 100) * CHAR_SPECIAL_HORIZ + alt +
            ZERO_ALT_OFFSET;
        uint8_t ltype =
            (ch_alt % (CH_PER_LONGCHAR * CHAR_SPECIAL_HORIZ)) <
            CHAR_SPECIAL_HORIZ ? 0xbe : 0x6e;
        osd.openSingle(start_col, start_row + i);
        osd.write(ltype + CHAR_SPECIAL_HORIZ - 1 -
            (ch_alt % CHAR_SPECIAL_HORIZ));
    }
    len = osd.write_num_s(str, osd_alt - osd_home_alt, 1, 0, 0);
#if 0
    osd.openSingle(start_col + 1, MAX7456_screen_rows / 2);
    osd.write(0x6d);
    osd.openSingle(start_col + 1, MAX7456_screen_rows / 2 + 1);
    osd.write(0x6c);
    for (int i = 0; i < len; i++) {
        osd.openSingle(start_col + 2 + i, MAX7456_screen_rows / 2 - 1);
        osd.write(0xbe);
        osd.openSingle(start_col + 2 + i, MAX7456_screen_rows / 2);
        osd.write(str[i]);
        osd.openSingle(start_col + 2 + i, MAX7456_screen_rows / 2 + 1);
        osd.write(0xcb);
    }
    osd.openSingle(start_col + 2 + len, MAX7456_screen_rows / 2 - 1);
    osd.write(0x69);
    osd.openSingle(start_col + 2 + len, MAX7456_screen_rows / 2);
    osd.write(0x6a);
    osd.openSingle(start_col + 2 + len, MAX7456_screen_rows / 2 + 1);
    osd.write(0x6b);
    for (int i = len + 1; i < 6; i++) {
        osd.openSingle(start_col + 2 + i, MAX7456_screen_rows / 2 - 1);
        osd.write(' ');
        osd.openSingle(start_col + 2 + i, MAX7456_screen_rows / 2);
        osd.write(' ');
        osd.openSingle(start_col + 2 + i, MAX7456_screen_rows / 2 + 1);
        osd.write(' ');
    }
#else
    osd.setPanel(start_col + 1, MAX7456_screen_rows / 2 - 1);
    osd.openPanel();
    /* Row 1 */
    osd.write(' ');
    for (int i = 0; i < len; i++)
        osd.write(0xbe);
    osd.write(0x69);
    for (int i = len; i < prev_len; i++)
        osd.write(' ');
    osd.write('|');
    /* Row 2 */
    osd.write(0x6d);
    for (int i = 0; i < len; i++)
        osd.write(str[i]);
    osd.write(0x6a);
    for (int i = len; i < prev_len; i++)
        osd.write(' ');
    osd.write('|');
    /* Row 2 */
    osd.write(0x6c);
    for (int i = 0; i < len; i++)
        osd.write(0xcb);
    osd.write(0x6b);
    for (int i = len; i < prev_len; i++)
        osd.write(' ');
    osd.closePanel();
#endif
    prev_len = len;
}

void do_converts()
{
    if (EEPROM.read(measure_ADDR) == 0) {
        converts = 3.6;
        converth = 1.0;
        spe = 0x10;
        high = 0x0C;
        temps = 0xBA;
        tempconv = 10;
        tempconvAdd = 0;
        distchar = 0x1B;
        distconv = 1000;
        climbchar = 0x1A;
    } else {
        converts = 2.23;
        converth = 3.28;
        spe = 0x19;
        high = 0x66;
        temps = 0xBB;
        tempconv = 18;
        tempconvAdd = 3200;
        distchar = 0x1C;
        distconv = 5280;
        climbchar = 0x1E;
    }
}
