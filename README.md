For an aggracultural application of LoraWAN, I found that multiple sensor types are required, and rather than 
building PCBs to handle each case, I wanted to have a sensor board that could manage all the uses that I needed, 
understanding that no single node would use all resources.  

re-485 : isolated with an isolated 5 v power source.

I2C : for temp and  humidity sensors that favor this.

current loop : for lake depth sensors that use 4-20 ma 12 or 24V current loops.

reed counter : water flow sensors to monitor flow volume and rate, using reed switch output, this is monitored in the 
Ultra Low Power (ULP) processor and continues in deep sleep of the MCU.

DC latching solenoids : motor drivers are used to actuate 9v or 12v DC latching solenoids to manage valves.  Two valves per sensor board.  

ADC : used to monitor water line pressure or for the current loop.  18 bit precision on a 2 v range with i2c data. 

solar charge controller for 6v (small) solar panel to maintain the battery.

I am using the Heltec e290 board, as the eink screen is very low power and in deep sleep with the ULP
running uses about 150 uAmp, reduced to 25 uAmp if the ULP is not  started.

I used JLCBCB.  The EasyEdaPro files for this board are included.  The board is connected to the Helted e290 
with a 40 pin male/female 20 cm cable.  

Board configuration:
The RS-485 including the 5v isolated supply is engagued by shorting its pins.
The voltage of the boost regulator is set with shorts to pins, 24v, 12v, or 9v.
Non populated resistor pads can be shorted to change the time constant of the reed debouncer from 10 ms as built to 0.9 ms by shorting that pad.  
The RS-485 termination resistor can be bypassed by shorting the center pad of the RS-485 region.  
RS-485 bias resistors (1kOhm) can be included by shorting the lateral pads of the RS-485 region.  

To deploy:


1100 mAmp battery
6v 120 mAmp solar panel.
Heltec-e290 with antenna (I used the standard SMC cable to the micro adapter to penetrate the project box).
Project box with see though lid to see the eink display.
Grommet to penetrate the box with the sensor cable.
Web serever running chirpstack 4 in docker as well as flask, mySQL.

RS-485 soil sensors:
1.  temp/moisture:  two sensors per node, RS-485 addresses 0x01 (shallow), 0x02(deep).
| Parameter | Value   | Units   |
|-----------|---------|---------|
| red       | 5v     | isolated    |
| black     | gnd    | isolated  |
| white     | A      | shallow sensor |
| yellow    | B      | shallow sensor |
| green     | A      | deep sensor |
| orange    | B      | deep sensor |
|-----------|---------|---------|

appPort

REED_NODE    8
#ifdef REED_NODE
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_REED_DELTA] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_REED_DELTA] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_FLOW_RATE] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_FLOW_RATE] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_LO] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_LO] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //  
  #endif
  
VALVE_NODE    9
#ifdef VALVE_NODE
  appData[appDataSize++] = wPress[0];       //  msb
  appData[appDataSize++] = xPress[1];    //  lsb
  appData[appDataSize++] = (uint8_t)(valveA -> time);       //  valve A
  appData[appDataSize++] = (uint8_t)(valveB -> time);       //  valve B 
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //
  #endif

SOIL_SENSOR_NODE    10
#ifdef SOIL_SENSOR_NODE
  appData[appDataSize++] = (uint8_t)(soilSensorOut[0]);       //  moist shallow
  appData[appDataSize++] = (uint8_t)(soilSensorOut[1]);       //  temp C shallow
  appData[appDataSize++] = (uint8_t)(soilSensorOut[2]);    //  pH shallow
  appData[appDataSize++] = (uint8_t)(soilSensorOut[3]);       //  moist deep
  appData[appDataSize++] = (uint8_t)(soilSensorOut[4]);       //  temp C deep
  appData[appDataSize++] = (uint8_t)(soilSensorOut[5]);    //  pH deep
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //  battery pct
  #endif

appPort map (udata from node), and target table in MySQL (active_devices is the device table)


1:  (2) lora_e5 soil probes, no pH   :  soil_air_data
            dOut.update({'soilTempCS': int.from_bytes(byteDataIn[0:2], 'big') / 10})
            dOut.update({'soilMoistS': int.from_bytes(byteDataIn[2:4], 'big') / 10})
            dOut.update({'soilTempCD': int.from_bytes(byteDataIn[4:6], 'big') / 10})
            dOut.update({'soilMoistD': int.from_bytes(byteDataIn[6:8], 'big') / 10})

2:  valve node and pressure  :  valve_data
            dOut.update({'hFlow': int.from_bytes(byteDataIn[0:2], 'big') / 10})
            dOut.update({'hPress': int.from_bytes(byteDataIn[2:4], 'big') / 10})
            dOut.update({'batPct': int.from_bytes(byteDataIn[4:6], 'big') / 10})
            dOut.update({'vlvStatus': int.from_bytes(byteDataIn[6:8], 'big') / 10})
3:  lake level  :  lake_data LAKE_NODE:  Uncaibrated 16 bit lake pressure for conversion to depth
            dOut.update({'lakePressUcal': int.from_bytes(byteDataIn[0:2], 'big')})
            dOut.update({'batPct': int.from_bytes(byteDataIn[2:3], 'big')})

4:

5:  lora_e5 (2) soil one air  :  soil_air_data
            dOut.update({'soilTempCS': int.from_bytes(byteDataIn[0:2], 'big') / 10})
            dOut.update({'soilMoistS': int.from_bytes(byteDataIn[2:4], 'big') / 10})
            dOut.update({'soilTempCD': int.from_bytes(byteDataIn[4:6], 'big') / 10})
            dOut.update({'soilMoistD': int.from_bytes(byteDataIn[6:8], 'big') / 10})
            dOut.update({'airTempC': round(int.from_bytes(byteDataIn[8:10], 'big') / 1000, 2)})
            dOut.update({'airMoist': round(int.from_bytes(byteDataIn[10:12], 'big') / 1000, 2)})
            dOut.update({'batPct': round(int.from_bytes(byteDataIn[12:13], 'big') / 2.55, 2)})

6:  SOIL_SENSOR_NODE
            dOut.update({'soilTempCS': int.from_bytes(byteDataIn[0:2], 'big') / 10})
            dOut.update({'soilMoistS': int.from_bytes(byteDataIn[2:4], 'big') / 10})
            dOut.update({'soilTempCD': int.from_bytes(byteDataIn[4:6], 'big') / 10})
            dOut.update({'soilMoistD': int.from_bytes(byteDataIn[6:8], 'big') / 10})
            dOut.update({'airTempC': round(int.from_bytes(byteDataIn[8:10], 'big') / 1000, 2)})
            dOut.update({'airMoist': round(int.from_bytes(byteDataIn[10:12], 'big') / 1000, 2)})
            dOut.update({'batPct': round(int.from_bytes(byteDataIn[12:13], 'big'), 2)})

7: 

8:  reed node  :  water_meter_data
        case 8:
            dOut.update({'reedDelta': int.from_bytes(byteDataIn[0:2], 'big')})
            dOut.update({'flowRate': int.from_bytes(byteDataIn[2:4], 'big')})
            dOut.update({'reedCount': int.from_bytes(byteDataIn[4:8], 'big')})
            dOut.update({'batPct': int.from_bytes(byteDataIn[8:9], 'big')})  

9:  valve node and pressure  :  valve_data
            dOut.update({'wPress': int.from_bytes(byteDataIn[0:2], 'big') / 10})
            dOut.update({'valveA': byteDataIn[2]})
            dOut.update({'valveB': byteDataIn[3]})
            dOut.update({'batPct': byteDataIn[4]})

10:  seeed weather station  :  s1000_data
            dOut.update({'airTemp': int.from_bytes(byteDataIn[0:2], 'big') / 100})
            dOut.update({'airHumid': int.from_bytes(byteDataIn[2:4], 'big') / 100})
            dOut.update({'airPresBar': int.from_bytes(byteDataIn[4:6], 'big') / 10000})
            dOut.update({'lightLux': int.from_bytes(byteDataIn[6:8], 'big') * 10})
            dOut.update({'minWindDir': int.from_bytes(byteDataIn[8:10], 'big')})
            dOut.update({'MaxWindDir': int.from_bytes(byteDataIn[10:12], 'big')})
            dOut.update({'avgWindDir': int.from_bytes(byteDataIn[12:14], 'big')})
            dOut.update({'minWindSp': int.from_bytes(byteDataIn[14:16], 'big') / 100})
            dOut.update({'maxWindSp': int.from_bytes(byteDataIn[16:18], 'big') / 100})
            dOut.update({'avgWindSp': int.from_bytes(byteDataIn[18:20], 'big') / 100})
            dOut.update({'accRain': int.from_bytes(byteDataIn[20:22], 'big') * 10})
            dOut.update({'accRainDur': int.from_bytes(byteDataIn[22:24], 'big') * 100})
            dOut.update({'rainInten': int.from_bytes(byteDataIn[24:26], 'big') / 10})
            dOut.update({'maxRain': int.from_bytes(byteDataIn[26:28], 'big') / 10})
            dOut.update({'pm_2_5': int.from_bytes(byteDataIn[28:30], 'big') / 10})
            dOut.update({'pm_10': int.from_bytes(byteDataIn[30:32], 'big') / 10})
            dOut.update({'c02': int.from_bytes(byteDataIn[32:34], 'big') / 10})

11:  two rs-485 soil probes include pH  :  soil_air_data
            dOut.update({'soilMoistS': int.from_bytes(byteDataIn[0:1], 'big')})
            dOut.update({'soilTempCS': int.from_bytes(byteDataIn[1:2], 'big')})
            dOut.update({'soilpHS': int.from_bytes(byteDataIn[2:3], 'big')/10})
            dOut.update({'soilMoistD': int.from_bytes(byteDataIn[3:4], 'big')})
            dOut.update({'soilTempCD': int.from_bytes(byteDataIn[4:5], 'big')})
            dOut.update({'soilpHD': int.from_bytes(byteDataIn[5:6], 'big')/10})
            dOut.update({'batPct': int.from_bytes(byteDataIn[6:7], 'big')})
            

12:  

            
appPort map (download to node)
5:  uint16_t loraWAN device cycle time in min
6:  uint32_t valve data -- to program valves 
7:  char(12) set device name

I have used the v4 of chirpstack as the docker image on EC2, and added flask and mySQL to manage the data.  The data is then served using flask.  I will upload that once modifications are complete.  
#   v a l v e _ d e p l o y  
 