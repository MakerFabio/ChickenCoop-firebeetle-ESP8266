// BOARD: select DFRobot Firebeetle ESP8266 
// Open Arduino IDE, File->Preferences, find Additional Boards Manager URLs, copy the below link, and paste in the blank.
// For domestic users: http://download.dfrobot.top/boards/package_DFRobot_index.json
// For overseas users: https://raw.githubusercontent.com/DFRobot/FireBeetle-ESP8266/master/package_firebeetle8266_index.json

// WARNING BOARD: WIFI was going better by selcting this baord: ESP8266 => NODE MCU 1.0 (ESDP12E) but it has ISSUES with sleep (it consume low only for 1/2 of the time)!!
// for firebeetle 8266 (which is an ESP-12F which is identical to the 12E beside the antenna) , add this json on preferences ESP8266 http://arduino.esp8266.com/stable/package_esp8266com_index.json 
// then on tool board manager install esp8266 by esp8266community, then select board, ESP8266, nodeMCU 1.0 (ESDP12E)    
// https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json)

// MEMO LOW POWER: on firebeetle ESP8266 E -select 80MHz clock speed on tool->CPU frequency

#include <ESP8266WiFi.h>
#include "c_types.h"

extern "C" {
#include "user_interface.h"// Required for LIGHT_SLEEP_T delay mode
}

#include "Arduino.h"

#include <Servo.h> // servo library  
#include "Print.h"
#include <BH1750.h> // install library BH1750 By Christopher Law
#include <Wire.h>
#include "gpio.h"
#define VIN_RATIO (4131/949)     // A0 Vin voltage divider firebeetle 8266 (populate R33 0Ohm)
#define VIN_WAIT_FOR_CHARGE 3300 // if Vin below this threshold, reamian in IDLe, with low power consumption until the battery is at least a little charged (otherwise the servo cosumption may reset the system)

#define SERVO_PIN 13 //  D2 on 8266 on firebeetle using firebeetle driver, pin 13 (marked IO13/D2) on standard ESP8266 module

#define LIGHT_TRESH_OPEN       70 //[LUX] if the luxmeter measures above this threshold the door will open
#define LIGHT_TRESH_CLOSE      6  //[LUX] if the luxmeter measures below this threshold (for more than 30 minutes) the door will close
#define LIGHT_TIMEOUT_MS       (10*1000) 
#define OPEN_CLOSE_DURATION_MS 14000 //[ms] how many ms will the 360 degree servo spin to close/ open the door
#define SLEEP_DURAT_S          3
#define WAIT_FOR_CLOSE_MIN     30  //[min] how much time the luxmeter has to be below the close threshold , before the coop door starts closing
#define WAIT_FOR_CLOSE_MS      (WAIT_FOR_CLOSE_MIN*60*1000) //[ms] as above, but in ms
#define OPEN_SERVO_ANGLE       170 // Angle sent to the servo to OPEN , thereis a 360 deg servo connected , so big angle will produce full speed forward
#define CLOSE_SERVO_ANGLE      10  // Angle sent to the servo to CLOSE , thereis a 360 deg servo connected , so small angle will produce full speed backward
#define IDLE_SERVO_ANGLE       90  // Angle sent to the servo to STAY STILL , thereis a 360 deg servo connected , so 90° is central position=> stop
#define PRINT_INTERVAL_MS      (6*1000) // every how many ms will printout the chicken coop status on the Serial monitor

enum coopStates {
  COOP_IDLE, // wake up, check for light level
  COOP_OPENING,
  COOP_OPENED, // door is OPEN because light is strong
  COOP_WAIT_FOR_CLOSE, // it's dark, wait for 1 hour before closing
  COOP_CLOSING,
  COOP_CLOSED // 
};

//****  GLOBAL VARIABLES  ***********************************
int    light;
Servo  servo;
BH1750 lightMeter(0x23);
enum coopStates coopState  = COOP_IDLE;

static unsigned long waitingStartMs, waitingTimeMs; 
static unsigned long previousPrintMillis;
bool butPres;
bool lightCheck = false;

//***  FUNCTION DECLARATION  *******************************
void showLight();
void readLight();

unsigned long sleepExtraMillis=0;
unsigned long myMillis(){
  return millis() + sleepExtraMillis;
}

// ESP8266 light sleep function. The firebeetle ESP8266 sinke 484uA in light sleep more (measured without sensors or servo attached)
// WARNING: this sleeps consumes 15mA if I select Board ESP8266 => NODE MCU 1.0 (ESDP12E) 
// This function consumes 930uA if you selcect board DFRobot Firebeetle ESP8266(no servo no sensor)
void lightSleep(uint32_t sleep_time_in_ms){
  // for timer-based light sleep to work, the os timers need to be disconnected
  extern os_timer_t *timer_list;
  timer_list = nullptr;
  
  //wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func); // optional callback called when the Boards wake up
  Serial.println("start to sleep");
  delay(1);
  
   // Here all the code to put con light sleep.03
  wifi_station_disconnect(); // new
  
  wifi_set_opmode(NULL_MODE);
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func); // Set wakeup callback
  wifi_fpm_do_sleep(sleep_time_in_ms  * 1000 );// debug restore
  // timed light sleep is only entered when the sleep command is followed by a delay() that is at least 1ms longer than the sleep
  delay(sleep_time_in_ms +1 );// also delay (in addition to millis) is doubled by 2 ?? when the device is sleeping
  Serial.println("waking up");
  delay(1);
}

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(150); 

  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output

  // Initialize BH1750: init the I2C bus for the BH1750 light meter (BH1750 library doesn't do this automatically)
  Wire.begin(); // on M5stack fire , there is only one SDA and SCL on PORT A, In Firebeetle ESP32, there is only one SDA and SCL wrtten in the silk connect there the LUXMETER which is address 0x23
 
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3); begin returns a boolean that can be used to detect setup problems.
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
  } else {
    Serial.println(F("Error initialising BH1750"));
  }
  
  Serial.println("ChickenCoopESP32_V1.21");
  
  servo.detach();
}


// main loop contains battery save functions and call user-sun state machine when needed
void loop()
{
  static unsigned long startMoveTime;
  uint32_t vin_mV=0;
  
  readLight();
  vin_mV=analogRead(A0) * VIN_RATIO;

  switch (coopState)
  {
  case COOP_IDLE:
    blinkLED(1);
    lightSleep(SLEEP_DURAT_S * 1000);
    if (vin_mV > VIN_WAIT_FOR_CHARGE )
    {
      if (lightCheck){
        if ( light < LIGHT_TRESH_CLOSE){
          servo.attach(SERVO_PIN); 
          coopState=COOP_CLOSING;
          servo.write(CLOSE_SERVO_ANGLE);
          startMoveTime = millis();
        }        
        if ( light > LIGHT_TRESH_OPEN){
          coopState=COOP_OPENING;
          servo.attach(SERVO_PIN); 
          servo.write(IDLE_SERVO_ANGLE);
          startMoveTime = millis();
        }
      } 
    }
    break;
  
  case COOP_OPENING:
    servo.write(OPEN_SERVO_ANGLE);
    if ((millis() - startMoveTime)>OPEN_CLOSE_DURATION_MS){
      servo.write(IDLE_SERVO_ANGLE);
      delay(50);      // wait a little to ensure at least a IDLE angle pulse is sent to the servo
      servo.detach(); // detach servo to avoid weird long pulses during sleep (servo library ios not working dutrring sleep)  
      coopState = COOP_OPENED;    
    }
    break;
  
  case COOP_OPENED:
    lightSleep(SLEEP_DURAT_S * 1000);
    if (lightCheck && ( light < LIGHT_TRESH_CLOSE) ) {
      coopState = COOP_WAIT_FOR_CLOSE;
      waitingStartMs = millis();
    }
    break;

  case COOP_WAIT_FOR_CLOSE: // THINK TWICE BEFORE CLOSING: light is low , wait for some time before closing the door, so chickens will be in
    waitingTimeMs = millis() - waitingStartMs;
    if ( lightCheck && (light < LIGHT_TRESH_CLOSE) ) {
      blinkLED(1);
      delay(500);
      Serial.println("waitStart, Millis ");
      Serial.println(waitingTimeMs/1000);Serial.println(waitingStartMs/1000);
      if ( waitingTimeMs > WAIT_FOR_CLOSE_MS){
          waitingTimeMs = 0;
          coopState = COOP_CLOSING;
          servo.attach(SERVO_PIN); 
          servo.write(IDLE_SERVO_ANGLE);// provide immediately a sIDLE servo when the servo wakes up
          startMoveTime = millis();
        }
    }
    else{
      coopState = COOP_OPENED;
      waitingTimeMs = 0;
    }
    break;

  case COOP_CLOSING:
    servo.write(CLOSE_SERVO_ANGLE);
    if ((millis() - startMoveTime)>OPEN_CLOSE_DURATION_MS){
      servo.write(IDLE_SERVO_ANGLE);
      delay(50);      // wait a little to ensure at least a IDLE angle pulse is sent to the servo
      servo.detach(); // detach servo to avoid weird long pulses during sleep (servo library ios not working dutrring sleep)  
      coopState = COOP_CLOSED;
    }
    break;
  
  case COOP_CLOSED:
    lightSleep(SLEEP_DURAT_S * 1000);
    if (lightCheck && ( light > LIGHT_TRESH_OPEN)){
        coopState = COOP_OPENING;
        servo.attach(SERVO_PIN); 
        servo.write(IDLE_SERVO_ANGLE);
        startMoveTime = millis();
    }
    break;

  default:
    break;
  }

  if ( (millis() - previousPrintMillis) > PRINT_INTERVAL_MS)
  {  
    previousPrintMillis = millis();
    Serial.println("***************************");
    Serial.print("millis = ");
    Serial.println(millis());
    
    Serial.print("Vin = ");
    Serial.println(vin_mV);

    showCoopState(); // blink and print coop state
    showLight();    
    delay(5);// extend a little bit for a longer LED pulse
  }

}

// optional callback called when the Boards wake up
void fpm_wakup_cb_func(void) {
  Serial.println("Light sleep callback"); //light sleep is over, either because timeout or external interrupt
  Serial.flush();
  wifi_fpm_close(); // disable force sleep function
  wifi_set_opmode(STATION_MODE); // set station mode
  wifi_station_connect(); // connect to AP

}

void readLight() {
static unsigned long prevLightUpdate = millis();
  if ( lightMeter.measurementReady()) { // debug , returning dummy -2 in case the sensor is disconnected, try to filter out
    light = lightMeter.readLightLevel();
    lightCheck = true;
    prevLightUpdate = millis();
  }else{ // if long time without a good light meas , tryto restart the sensor
    if ((millis()-prevLightUpdate )>LIGHT_TIMEOUT_MS){ 
      prevLightUpdate = millis();
      Serial.println("restartingLightSensor");
      lightCheck = false;
      lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE); // attempt to initialize again the sensor
    }
  }
 }

void blinkLED(int blinkCount )
{  
    
    for (int i = 0 ; i < blinkCount ; i++)
    {
      digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level but actually the LED is on; this is because it is active low on the ESP-01)
      delay(50);// extend a little bit for a longer LED pulse
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
    }
}

void showLight()
{
      Serial.print("light ");  // Print text on the screen (string)
      Serial.println( light);  // Print text on the screen (string)
}

void showCoopState()
{
  Serial.print("openclosedur=");Serial.println(OPEN_CLOSE_DURATION_MS);
  switch (coopState)
  {
  case COOP_IDLE:
    Serial.println("IDLE");
    break;
  case COOP_OPENING:
    Serial.println("OPENING");
    break;
  case COOP_OPENED:
    Serial.println("OPENED");
    blinkLED(1);
    break;    
  case COOP_WAIT_FOR_CLOSE:
    Serial.println("WAITxCLOSE ");
    break;    
  case COOP_CLOSING:
    Serial.println("COOP_CLOSING    ");
    break;    
  case COOP_CLOSED:
    Serial.println("COOP_CLOSED   ");
    blinkLED(2);
    break;    
  default:
    break;
  }
}
