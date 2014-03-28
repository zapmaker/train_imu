/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
November  2013     V2.3
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>

Modified by zapmaker for z-axis train curve detection, March 2014.
Visit zapmaker.org for specific circuit configuration. Currently it
requires a Rainbowduino coupled with a GY80 IMU.

GPL3 license as noted above.

*/

#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "train_imu.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "LCD.h"
#include "Sensors.h"
#include "Serial.h"
#include "Protocol.h"

#include <avr/pgmspace.h>
#include <Rainbowduino.h>

/*********** RC alias *****************/

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
  #if ACC
    "ANGLE;"
    "HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    "BARO;"
  #endif
  #ifdef VARIOMETER
    "VARIO;"
  #endif
  #if MAG
    "MAG;"
    "HEADFREE;"
    "HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    "CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    "CAMTRIG;"
  #endif
  #if GPS
    "GPS HOME;"
    "GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    "PASSTHRU;"
  #endif
  #if defined(BUZZER)
    "BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    "LEDMAX;"
    "LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    "LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    "CALIB;"
  #endif
  #ifdef GOVERNOR_P
    "GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    "OSD SW;"
  #endif
;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  #if ACC
    1, //"ANGLE;"
    2, //"HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    3, //"BARO;"
  #endif
  #ifdef VARIOMETER
    4, //"VARIO;"
  #endif
  #if MAG
    5, //"MAG;"
    6, //"HEADFREE;"
    7, //"HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    8, //"CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    9, //"CAMTRIG;"
  #endif
  #if GPS
    10, //"GPS HOME;"
    11, //"GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    12, //"PASSTHRU;"
  #endif
  #if defined(BUZZER)
    13, //"BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    14, //"LEDMAX;"
    15, //"LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    16, //"LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    17, //"CALIB;"
  #endif
  #ifdef GOVERNOR_P
    18, //"GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    19, //"OSD_SWITCH;"
  #endif
};


uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint32_t previousDebugTime = 0;
uint32_t previousRainbowTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG;
int16_t  magHold,headFreeModeHold; // [-180;+180]
uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];
int32_t  AltHold; // in cm
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0,0,0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

#if defined(ARMEDTIMEWARNING)
  uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

int16_t  debug[4];

flags_struct_t f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  uint16_t cycleTimeMax = 0;       // highest ever cycle timen
  uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
  int32_t  BAROaltMax;             // maximum value
  uint16_t GPS_speedMax = 0;       // maximum speed from gps
  uint16_t powerValueMaxMAH = 0;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  uint32_t armedTime = 0;
#endif

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;



// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
  uint16_t InflightcalibratingA = 0;
  int16_t AccInflightCalibrationArmed;
  uint16_t AccInflightCalibrationMeasurementDone = 0;
  uint16_t AccInflightCalibrationSavetoEEProm = 0;
  uint16_t AccInflightCalibrationActive = 0;
#endif


// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SPEKTRUM)
  volatile uint8_t  spekFrameFlags;
  volatile uint32_t spekTimeLast;
#endif

#if defined(OPENLRSv2MULTI)
  uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif


// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;

#ifdef LOG_PERMANENT
  plog_t plog;
#endif

  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  int16_t  nav[2];
  int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

  uint8_t alarmArray[16];           // array
 
#if BARO
  int32_t baroPressure;
  int32_t baroTemperature;
  int32_t baroPressureSum;
#endif

int prevPixelGX = 0;
int prevPixelGY = 0;
int prevPixelAX = 0;
int prevPixelAY = 0;


const int interruptNumber = 0;// usually connects to pin 2
const int pwmPin = 3;
const int dir1Pin = A2;//4;
const int dir2Pin = A3;//5;

volatile int state = LOW;
volatile int lastTime = 0;

int last = 0;
int motorTime = 0;
int serialTime = 0;
int speedAdjTime = 0;
int delta = 0;
int pwmOut = 0;
int lastDir1 = 0;
int lastDir2 = 0;
int lastFSAdcRead = 0;
int lastSCAdcRead = 0;
float detectedSpeed = 0;
boolean useBrakingOnReversal = false;
int lastswaptime = 0;
#define SVAL 390
int dirX = SVAL;
#define SDELT 3000

#define BUFSIZE 200
char buf[BUFSIZE];
int speedAdjustIncrementer = 2;
int correctedMotorPwmValue = 0;

void stop()
{
  // decide if we are braking on stop or
  // letting motor coast 
  if (useBrakingOnReversal) {
    lastDir1 = 0;
    lastDir2 = 0;
  }
  else {
    lastDir1 = 1;
    lastDir2 = 1;
  }
}

void blink()
{
  lastTime = millis();
  
  state = !state;
}


void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
// TODO maybe add something here
}

void setup() {
  #if !defined(GPS_PROMINI)
    SerialOpen(0,SERIAL0_COM_SPEED);
    #if defined(PROMICRO)
      SerialOpen(1,SERIAL1_COM_SPEED);
    #endif
    #if defined(MEGA)
      SerialOpen(1,SERIAL1_COM_SPEED);
      SerialOpen(2,SERIAL2_COM_SPEED);
      SerialOpen(3,SERIAL3_COM_SPEED);
    #endif
  #endif
  
  readGlobalSet();
  #ifndef NO_FLASH_CHECK
    #if defined(MEGA)
      uint16_t i = 65000;                             // only first ~64K for mega board due to pgm_read_byte limitation
    #else
      uint16_t i = 32000;
    #endif
    uint16_t flashsum = 0;
    uint8_t pbyt;
    while(i--) {
      pbyt =  pgm_read_byte(i);        // calculate flash checksum
      flashsum += pbyt;
      flashsum ^= (pbyt<<8);
    }
  #endif
  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    global_conf.currentSet=2;
  #else
    global_conf.currentSet=0;
  #endif
  while(1) {                                                    // check settings integrity
  #ifndef NO_FLASH_CHECK
    if(readEEPROM()) {                                          // check current setting integrity
      if(flashsum != global_conf.flashsum) update_constants();  // update constants if firmware is changed and integrity is OK
    }
  #else
    readEEPROM();                                               // check current setting integrity
  #endif  
    if(global_conf.currentSet == 0) break;                      // all checks is done
    global_conf.currentSet--;                                   // next setting for check
  }
  readGlobalSet();                              // reload global settings for get last profile number
  #ifndef NO_FLASH_CHECK
    if(flashsum != global_conf.flashsum) {
      global_conf.flashsum = flashsum;          // new flash sum
      writeGlobalSet(1);                        // update flash sum in global config
    }
  #endif
  readEEPROM();                                 // load setting data from last used profile
  blinkLED(2,40,global_conf.currentSet+1);          
  #if defined(OPENLRSv2MULTI)
    initOpenLRS();
  #endif
  initSensors();
  previousTime = micros();
  #if defined(GIMBAL)
   calibratingA = 512;
  #endif
  calibratingG = 512;
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
  
  #if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) || defined(OLED_DIGOLE) || defined(LCD_TELEMETRY_STEP)
    initLCD();
  #endif
  #ifdef LCD_TELEMETRY_DEBUG
    telemetry_auto = 1;
  #endif
  #ifdef LCD_CONF_DEBUG
    configurationLoop();
  #endif
  #ifdef FASTER_ANALOG_READS
    ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  #endif
  f.SMALL_ANGLES_25=1; // important for gyro only conf
  #ifdef LOG_PERMANENT
    // read last stored set
    readPLog();
    plog.lifetime += plog.armed_time / 1000000;
    plog.start++;         // #powercycle/reset/initialize events
    // dump plog data to terminal
    #ifdef LOG_PERMANENT_SHOW_AT_STARTUP
      dumpPLog(0);
    #endif
    plog.armed_time = 0;   // lifetime in seconds
    //plog.running = 0;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  #endif

  Rb.init();

  pinMode(2, INPUT);
  attachInterrupt(interruptNumber, blink, RISING);
  digitalWrite(pwmPin, pwmOut);
  pinMode(pwmPin, OUTPUT);
  digitalWrite(dir1Pin, lastDir1);
  pinMode(dir1Pin, OUTPUT);
  digitalWrite(dir2Pin, lastDir2);
  pinMode(dir2Pin, OUTPUT);
  
  
#if defined (PROMINI)
debugmsg_append_str("initialization completed (promini)\n");  
#else
debugmsg_append_str("initialization completed\n");  
#endif
}

// ******** Main Loop *********
void loop () {
  #if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
    #if defined(GPS_PROMINI)
      if(GPS_Enable == 0) {serialCom();}
    #else
      serialCom();
    #endif
  #endif

  #if defined(SPEKTRUM)
    if (spekFrameFlags == 0x01) readSpektrum();
  #endif
  
  #if defined(OPENLRSv2MULTI) 
    Read_OpenLRS_RC();
  #endif 

// TODO future:    #if defined(INFLIGHT_ACC_CALIBRATION)

// not in rc loop
  static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
  if(taskOrder>4) taskOrder-=5;
  switch (taskOrder) {
    case 0:
      taskOrder++;
      #if MAG
        if (Mag_getADC()) break; // max 350 µs (HMC5883) // only break when we actually did something
      #endif
    case 1:
      taskOrder++;
      #if BARO
        if (Baro_update() != 0 ) break;
      #endif
    case 2:
      taskOrder++;
      #if BARO
        if (getEstimatedAltitude() !=0) break;
      #endif    
    case 3:
      taskOrder++;
      #if GPS
        if(GPS_Enable) GPS_NewData();
        break;
      #endif
    case 4:
      taskOrder++;
      #if SONAR
        Sonar_update(); //debug[2] = sonarAlt;
      #endif
      #ifdef LANDING_LIGHTS_DDR
        auto_switch_landing_lights();
      #endif
      #ifdef VARIOMETER
        if (f.VARIO_MODE) vario_signaling();
      #endif
      break;
  } 
 
  computeIMU();
  // Measure loop rate just afer reading the sensors
  
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
  
  if ((currentTime - previousDebugTime) > 500000) {
//    String msg = "I2C Error Count:";
//    msg += i2c_errors_count;
//    msg += "\n";
 
//    debugmsg_append_str(msg.c_str());
    /*
    String msg = "IMU a=";
    msg += imu.gyroData[0];
    msg += " b=";
    msg += imu.gyroData[1];
    msg += " c=";
    msg += imu.gyroData[2];
    msg += "\n";
    
    debugmsg_append_str(msg.c_str());
    */
    /*
    String msg = "IMU r=";
    msg += att.angle[ROLL];
    msg += " p=";
    msg += att.angle[PITCH];
    msg += "\n";
    
    debugmsg_append_str(msg.c_str());
    *//*
      String msg = "IMU ax=";
      msg += imu.accADC[1];
      msg += " ay=";
      msg += imu.accADC[0];
      msg += "\n";
      
      debugmsg_append_str(msg.c_str());*/
    
    previousDebugTime = currentTime; 
  }

/*
  if ((currentTime - previousRainbowTime) > 50000) {
    int gx = 3 - (imu.gyroData[1] / 60);
    int gy = 3 - (imu.gyroData[0] / 60); 
    
    if (gx < 0) gx = 0;
    if (gx > 7) gx = 7;
    if (gy < 0) gy = 0;
    if (gy > 7) gy = 7;
    
    int setGX = -1;
    int setGY = -1;
    int setAX = -1;
    int setAY = -1;    
    
    if (gx != prevPixelGX || gy != prevPixelGY) {
      //Rb.setPixelXY(gx, gy, 0xFF, 0, 0);
      setGX = gx;
      setGY = gy;
    }

    int ax = 3 - (imu.accSmooth[1] / 20);
    int ay = 3 - ((imu.accSmooth[0] + 5000) / 20); 
    
    if (ax < 0) ax = 0;
    if (ax > 7) ax = 7;
    if (ay < 0) ay = 0;
    if (ay > 7) ay = 7;
    
    if (ax != prevPixelAX || ay != prevPixelAY) {
      setAX = ax;
      setAY = ay;
        
    
      //String msg = "IMU ax=";
      //msg += ax;
      //msg += " ay=";
      //msg += ay;
      //msg += "\n";
      
      //debugmsg_append_str(msg.c_str());

    }
    
    if (setAX != -1 && setAY != -1 && setGX != -1 && setGY != -1
       && setAX == setGX && setAY == setGY) {
      Rb.setPixelXY(setAX, setAY, 0xFF, 0xFF, 0xFF);

      Rb.setPixelXY(prevPixelAX, prevPixelAY, 0, 0, 0);
      Rb.setPixelXY(prevPixelGX, prevPixelGY, 0, 0, 0);
      
      prevPixelAX = ax;
      prevPixelAY = ay;

      prevPixelGX = gx;
      prevPixelGY = gy;

    }
    else if (setAX != -1 && setAY != -1) {
      Rb.setPixelXY(setAX, setAY, 0, 0, 0xFF);    
      
      Rb.setPixelXY(prevPixelAX, prevPixelAY, 0, 0, 0);
      
      prevPixelAX = ax;
      prevPixelAY = ay;

    }
    else if (setGX != -1 && setGY != -1) {
      Rb.setPixelXY(setGX, setGY, 0xFF, 0, 0);      
      
      Rb.setPixelXY(prevPixelGX, prevPixelGY, 0, 0, 0);
      
      prevPixelGX = gx;
      prevPixelGY = gy;

    }
    
    
    previousRainbowTime = currentTime; 
  }
*/

  int curr = millis();
  if (last != lastTime) {
    int tmp = lastTime - last;
    if (tmp > 30) {
      delta = tmp;
    }
    last = lastTime;
  }
  else if ((curr - lastTime) > 3000) {
    delta = 0;
  }  
    
  if ((curr - motorTime) > 100) {
    // scaling to match detector wheel speed
    lastFSAdcRead = analogRead(1) / 4;
    lastSCAdcRead = lastFSAdcRead / 2 + 64;
    /*
    //Serial.println(a);
    //int a = dirX;
    a /= 4;
    // split the potentiometer in half, incrementing
    // the speed in opposite motor direction from the
    // middle of the pot
    if (a <= 127) {
      // bottom half of pot
      if (lastDir1 == 0 && lastDir2 == 1) {
        // reversal occurred, stop motor to protect 
        // motor and controller
        stop();
        // ensure no pwm signal, just dc
        a = 255;
      }
      else {
        // change direction
        lastDir1 = 1;
        lastDir2 = 0;
        a = (127 - a) * 2;
      }
    }
    else {
      // top half of pot
      if (lastDir1 == 1 && lastDir2 == 0) {
        // reversal occurred, stop motor to protect 
        // motor and controller
        stop();
        // ensure no pwm signal, just dc
        a = 255;
      }
      else {
        // change direction
        lastDir1 = 0;
        lastDir2 = 1;
        a = (a - 128) * 2;
      }
     
    }

    // motor direction
    digitalWrite(dir1Pin, lastDir1);
    digitalWrite(dir2Pin, lastDir2);
*/
    // motor direction
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);

    float d = delta;
    detectedSpeed = 0;
    if (d > 0)
      detectedSpeed = 10000 / d;

    int cmpdelta = 200;
    if (detectedSpeed < 20)
      cmpdelta = 50;

    boolean correctingViaGyro = false;
    if ((curr - speedAdjTime) > cmpdelta)
    {
      if (imu.gyroData[2] > 180 || imu.gyroData[2] < -180)
      {
        correctedMotorPwmValue -= 4;
        if (correctedMotorPwmValue < 0)
          correctedMotorPwmValue = 0;
        correctingViaGyro = true;
      }  
      else if (detectedSpeed > lastSCAdcRead)
      {
        //speedAdjustIncrementer
        correctedMotorPwmValue -= 2;
        if (correctedMotorPwmValue < 0)
          correctedMotorPwmValue = 0;
      }
      else if (detectedSpeed < lastSCAdcRead)
      {
        correctedMotorPwmValue += 4;
        if (correctedMotorPwmValue > 255)
          correctedMotorPwmValue = 255;
      }
      speedAdjTime = curr;    
    }
        
    pwmOut = correctedMotorPwmValue;  

    //String msg = "IMU a=";
    //msg += imu.gyroData[0];
    //msg += " b=";
    //msg += imu.gyroData[1];
    //msg += " c=";
    //msg += imu.gyroData[2];
    //msg += "\n";
    
    //debugmsg_append_str(msg.c_str());
    
    int gy = 3 - (imu.gyroData[1] / 60);
    int gz = 3 - (imu.gyroData[2] / 60);
    
    if (gz < 0) gz = 0;
    if (gz > 7) gz = 7;

    //if (gx < 0) gx = 0;
    //if (gx > 7) gx = 7;
    if (gy < 0) gy = 0;
    if (gy > 7) gy = 7;

    int matrixPwm = pwmOut / 4;
    int matrixSpeed = detectedSpeed / 20;
    int matrixSetSpeed = lastFSAdcRead / 4;
    Rb.blankDisplay();

    Rb.setPixelXY(gy, gz, 0, 0xFF, 0);    
    
    int count = 0;
    for (int n = 0; n < 8; n++)
    {
      for (int m = 0; m < 8; m++)
      {
        if (count == matrixSetSpeed)
        {
          Rb.setPixelXY(n, m, 0, 0x2, 0x2);
        }
        if (count == matrixSpeed)
        {
          Rb.setPixelXY(n, m, 0, 0, 0xFF);      
        }
        if (count == matrixPwm)
        {
          int r = 0xFF;
          int g = 0;
          int b = 0;
          if (correctingViaGyro)
          {
            r = 0xFF;
            g = 0;
            b = 0xFF;
          }
          else if ((correctedMotorPwmValue == 255) && (lastSCAdcRead > detectedSpeed))
          {
            r = 0xFF;
            g = 0xFF;
            b = 0;
          }

          Rb.setPixelXY(n, m, r, g, b);      
        }
        
        count++;
      }
    }

    //pwmOut = a;
    // pwm 'speed'
    analogWrite(pwmPin, pwmOut);

    
    motorTime = curr;
  } 
  
  if ((curr - lastswaptime) > SDELT) {
    if (dirX < 512)
      dirX = 512 + SVAL;
    else
      dirX = 512 - SVAL;

    lastswaptime = curr;
  }
  
  /*
  if ((curr - serialTime) > 1000) {  
    String msg = "L1: ";
    msg += lastDir1;
    msg += " L2: ";
    msg += lastDir2;
    msg += " A: ";
    msg += pwmOut;
    msg += "\n";
    msg.toCharArray(buf, BUFSIZE); 
    //Serial.write(buf);
    
    msg = "Pulse Time: ";
    msg += delta;
    msg += " Speed: ";
    msg += (int)detectedSpeed;
    msg += "\n";
    msg.toCharArray(buf, BUFSIZE); 
    //Serial.write(buf);
    
    serialTime = curr;   
  }
  */
  

//*********************************** 
 
  #if MAG
    if (abs(rcCommand[YAW]) <70 && f.MAG_MODE) {
      int16_t dif = att.heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif*conf.pid[PIDMAG].P8>>5;
    } else magHold = att.heading;
  #endif

}


