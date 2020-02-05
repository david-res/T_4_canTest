const long loopDelay1 = 10;

unsigned long timeNow1 = 0;
float canData [7]; //place up to date CAN data into this array
int boost_percent;
float boost_pressuer, boost_final;

void loop(void) {
  
  if(millis() > timeNow1 + loopDelay1){
    timeNow1 = millis();
    elapsedMillis em = 0; 
    canData [0] = PID_01(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE); //BOOST
    canData [2] = PID_01(ABSOLULTE_BAROMETRIC_PRESSURE);
    Serial.printf("elapsed PID loop poll %u\n", (uint32_t)em);
    em = 0;

}

elapsedMillis em2 = 0; 

boost_pressuer = (canData [0] - 102);
boost_final = boost_pressuer * 0.145038;
    if (boost_pressuer < 0 ){
    boost_percent = map(boost_pressuer, -110, -0.99,0,249);
    }

    if (boost_pressuer >= 0.1) {
    boost_percent = map(boost_pressuer, 0.1, 200.0, 250, 750);
    }

    if (canData [1] > 22){
    canData [1] = 22;
    }

    int16_t min_x = g_needle_rect_min_x;
    int16_t min_y = g_needle_rect_min_y;
    int16_t max_x = g_needle_rect_max_x;
    int16_t max_y = g_needle_rect_max_y;
    tft.writeRect(g_offset_x, g_offset_y, 150, 150, (const uint16_t*)defi);
    

    drawNeedle(boost_percent, COLOR_RED);
    //tft.setTextColor(ILI9341_WHITE);
    //tft.setTextSize(2);
    //tft.setCursor(90, 220);
    //tft.print (boost_final, 1);
    
    min_x = min(min_x, g_needle_rect_min_x);
    min_y = min(min_y, g_needle_rect_min_y);
    max_x = max(max_x, g_needle_rect_max_x);
    max_y = max(max_y, g_needle_rect_max_y);
    //tft.setClipRect(5, 45, 150, 150);
    tft.setClipRect(min_x, min_y, max_x - min_x + 1, max_y - min_y +1);
    tft.updateScreen();
    tft.setClipRect();
    Serial.printf("elapsed Screen Update %u\n", (uint32_t)em2);
    em2 = 0;
}

//MODE 01 PID'S
float PID_01(uint8_t pid) 
{
  elapsedMillis em1 = 0; 

  CAN_message_t can_MsgRx,can_MsgTx;
  
  can_MsgTx.buf[0] = 0x02;  // Two bytes in the request
  can_MsgTx.buf[1] = 0x01; // OBD mode 1
  can_MsgTx.buf[2] = pid;  
  can_MsgTx.buf[3] = 0;
  can_MsgTx.buf[4] = 0;  
  can_MsgTx.buf[5] = 0;
  can_MsgTx.buf[6] = 0;  
  can_MsgTx.buf[7] = 0;
  can_MsgTx.len = 8; // number of bytes in request
  //can_MsgTx.ext = 0; 
  can_MsgTx.flags.extended = 0; // 11 bit header, not 29 bit
  can_MsgTx.flags.remote = 0;
  can_MsgTx.id = 0x7E0; // request header for OBD
//  can_MsgTx.timeout = 500;
  Can0.write(can_MsgTx);  

  Serial.printf("elapsed TX %u\n", (uint32_t)em1);
  em1 = 0;

  elapsedMillis waiting;     // "waiting" starts at zero
  
  while (waiting < 20) {   //Check for timeout

    if(Can0.read(can_MsgRx)) { 


                    #define A can_MsgRx.buf[3]
                    #define B can_MsgRx.buf[4]
                    #define C can_MsgRx.buf[5]
                    #define D can_MsgRx.buf[6]

        if((can_MsgRx.id == 0x7E8) && (can_MsgRx.buf[2] == pid))
       {
                    //uint16_t pid_n = (uint16_t)(message.data[2] << 8) | message.data[3];
                    switch (pid){

                    //case FUEL_SYSTEM_STATUS: // raw
                   case RUN_TIME_SINCE_ENGINE_START:
                    //case DISTANCE_TRAVELED_WITH_MIL_ON:
                    //case DISTANCE_TRAVELED_SINCE_CODES_CLEARED:
                    //case TIME_RUN_WITH_MIL_ON:
                    //case TIME_SINCE_TROUBLE_CODES_CLEARED:
                        return (A * 256.0 + B);
                        break;

                    //case CALCULATED_ENGINE_LOAD:
                    case THROTTLE_POSITION:
                    //case COMMANDED_EGR:
                    //case COMMANDED_EVAPORATIVE_PURGE:
                    //case FUEL_TANK_LEVEL_INPUT:
                    case RELATIVE_THROTTLE_POSITION:
                    //case ABSOLUTE_THROTTLE_POSITION_B:
                    //case ABSOLUTE_THROTTLE_POSITION_C:
                    //case ABSOLUTE_THROTTLE_POSITION_D:
                    //case ABSOLUTE_THROTTLE_POSITION_E:
                    //case ABSOLUTE_THROTTLE_POSITION_F:
                    //case COMMANDED_THROTTLE_ACTUATOR:
                    //case ETHANOL_FUEL_PERCENTAGE:
                    //case RELATIVE_ACCELERATOR_PEDAL_POSITTION:
                    //case HYBRID_BATTERY_PACK_REMAINING_LIFE:
                        return (A / 2.55);
                        break;

                    //case COMMANDED_SECONDARY_AIR_STATUS:                    // raw
                    //case OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO:            // raw
                    //case OXYGEN_SENSORS_PRESENT_IN_2_BANKS:                 // raw
                    //case OXYGEN_SENSORS_PRESENT_IN_4_BANKS:                 // raw
                    //case AUXILIARY_INPUT_STATUS:                            // raw
                    //case FUEL_TYPE:                                         // raw
                    //case EMISSION_REQUIREMENT_TO_WHICH_VEHICLE_IS_DESIGNED: // raw
                        //return (A);

                    case OXYGEN_SENSOR_1_SHORT_TERM_FUEL_TRIM:
                    case OXYGEN_SENSOR_2_SHORT_TERM_FUEL_TRIM:
                    case OXYGEN_SENSOR_3_SHORT_TERM_FUEL_TRIM:
                    case OXYGEN_SENSOR_4_SHORT_TERM_FUEL_TRIM:
                    case OXYGEN_SENSOR_5_SHORT_TERM_FUEL_TRIM:
                    case OXYGEN_SENSOR_6_SHORT_TERM_FUEL_TRIM:
                    case OXYGEN_SENSOR_7_SHORT_TERM_FUEL_TRIM:
                    case OXYGEN_SENSOR_8_SHORT_TERM_FUEL_TRIM:
                        return ((B / 1.28) - 100.0);
                        break;

                    case ENGINE_COOLANT_TEMPERATURE:
                    case AIR_INTAKE_TEMPERATURE:
                    case AMBIENT_AIR_TEMPERATURE:
                    //case ENGINE_OIL_TEMPERATURE:
                        return (A - 40.0);
                        break;
/*
                    case SHORT_TERM_FUEL_TRIM_BANK_1:
                    case LONG_TERM_FUEL_TRIM_BANK_1:
                    case SHORT_TERM_FUEL_TRIM_BANK_2:
                    case LONG_TERM_FUEL_TRIM_BANK_2:
                    case EGR_ERROR:
                        return ((A / 1.28) - 100.0);
*/
                    case FUEL_PRESSURE:
                        return (A * 3.0);
                        break;

                    case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
                    //ase VEHICLE_SPEED:
                    //case WARM_UPS_SINCE_CODES_CLEARED:
                    case ABSOLULTE_BAROMETRIC_PRESSURE:
                        return (A);
                        break;

                    case ENGINE_RPM:
                        return ((A * 256.0 + B) / 4.0);
                        break;

                    case TIMING_ADVANCE:
                        return ((A / 2.0) - 64.0);
                        break;

                    //case MAF_AIR_FLOW_RATE:
                      //  return ((A * 256.0 + B) / 100.0);

                    //case FUEL_RAIL_PRESSURE:
                      //  return ((A * 256.0 + B) * 0.079);

                    //case FUEL_RAIL_GAUGE_PRESSURE:
                    //case FUEL_RAIL_ABSOLUTE_PRESSURE:
                      //  return ((A * 256.0 + B) * 10.0);

                    case OXYGEN_SENSOR_1_FUEL_AIR_EQUIVALENCE_RATIO:
                    case OXYGEN_SENSOR_2_FUEL_AIR_EQUIVALENCE_RATIO:
                    case OXYGEN_SENSOR_3_FUEL_AIR_EQUIVALENCE_RATIO:
                    case OXYGEN_SENSOR_4_FUEL_AIR_EQUIVALENCE_RATIO:
                    case OXYGEN_SENSOR_5_FUEL_AIR_EQUIVALENCE_RATIO:
                    case OXYGEN_SENSOR_6_FUEL_AIR_EQUIVALENCE_RATIO:
                    case OXYGEN_SENSOR_7_FUEL_AIR_EQUIVALENCE_RATIO:
                    case OXYGEN_SENSOR_8_FUEL_AIR_EQUIVALENCE_RATIO:
                        return (((A * 256.0 + B) * 2.0) / 65536.0);
                        break;

                   // case EVAP_SYSTEM_VAPOR_PRESSURE:
                     //   return (((int16_t)(A * 256.0 + B)) / 4.0);

                    case CATALYST_TEMPERATURE_BANK_1_SENSOR_1:
                    case CATALYST_TEMPERATURE_BANK_2_SENSOR_1:
                    case CATALYST_TEMPERATURE_BANK_1_SENSOR_2:
                    case CATALYST_TEMPERATURE_BANK_2_SENSOR_2:
                        return (((A * 256.0 + B) / 10.0) - 40.0);
                        break;

                    case CONTROL_MODULE_VOLTAGE:
                        return ((A * 256.0 + B) / 1000.0);
                        break;

                    //case ABSOLUTE_LOAD_VALUE:
                      //  return ((A * 256.0 + B) / 2.55);

                    case FUEL_AIR_COMMANDED_EQUIVALENCE_RATE:
                        return (2.0 * (A * 256.0 + B) / 65536.0);
                        break;

                    //case ABSOLUTE_EVAP_SYSTEM_VAPOR_PRESSURE:
                      //  return ((A * 256.0 + B) / 200.0);

                    //case 0x54:
                      //  return ((A * 256.0 + B) - 32767.0);

                    //case FUEL_INJECTION_TIMING:
                      //  return (((A * 256.0 + B) / 128.0) - 210.0);

                    //case ENGINE_FUEL_RATE:
                      //  return ((A * 256.0 + B) / 20.0);
                    }
                }

                Serial.printf("elapsed RX %u\n", (uint32_t)em1);
                em1 = 0;

                if (can_MsgRx.buf[1] == 0x7F)
                {
                    return NAN;
                }  

                
         }
         

 
  } // while
  
  return 0;

}
