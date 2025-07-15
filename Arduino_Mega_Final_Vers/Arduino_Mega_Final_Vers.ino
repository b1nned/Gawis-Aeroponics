#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL6367CD3o0"
#define BLYNK_TEMPLATE_NAME "Aeroponics with pest detection"
#define BLYNK_AUTH_TOKEN "diCKwLcqroTJ-A83Rye39oa5mGojQpSL"

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <Wire.h>
#include <DS3231.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "GravityTDS.h"
#include "DHT.h"

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "aeronetwork";
char pass[] = "Aeroponics12345";

const unsigned long TIME_CHECK_BLYNK_CONNECTION = 60000;  // (ms) check device connection to blynk every this time
const unsigned long TIME_UPDATE_SENSORS = 5000; // (ms) update sensors every 5 seconds for real-time monitoring

//pH normal range
const float NORMAL_RANGE_PH_MIN = 5.5; // minimum pH for normal
const float NORMAL_RANGE_PH_MAX = 7.5; // maximum pH for normal

const unsigned int TIME_DEBOUNCE_ABNORMAL_PH = 5000; // ms time, when pH is out of normal range for this duration pump is turned ON
const unsigned long TIME_ABNORMAL_PH_DISPENSE_SOLUTION = 5000; // ms time to dispense solution to normalize water
const unsigned long TIME_ABNORMAL_PH_WAITING_FOR_STABLE = 10000; // ms time to wait for water to stable after dispsensing solution before checking if water is normal

//water level
const unsigned int TIME_DEBOUNCE_WATER_LEVEL = 2000; // milliseconds time to check water stability
const byte ULTRASONIC_SENSOR_MAX_DISTANCE = 31; // cm, measurement from ultrasonic sensor to bottom of container
const byte ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_LOW = 7; //cm, measurement from bottom container to water level low grid
const byte ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_MEDIUM = 16; //cm, measurement from bottom container to water level medium grid
const byte ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_HIGH = 24; //cm, measurement from bottom container to water level high grid

//temperature threshold for turning ON fan
const float AIR_TEMPERATURE_THRESHOLD = 25.0; // celcius temperature threshold to turn on FAN
const unsigned int TIME_DEBOUCNE_AIR_TEMPERATURE_FAN = 5000; // millisecond time to debounce air temperature before turning ON fan

//mist ON OFF intervals
const unsigned long TIME_INTERVAL_MIST_ON_DURATION = 5000; // ms time MIST is ON during schedule
const unsigned long TIME_INTERVAL_MIST_OFF_DURATION = 10000; // ms time MIST is OFF during schedule

//pest signal validity duration
const unsigned int TIME_ESPCAM_PEST_SIGNAL_VALIDITY = 10000; // espcam command DETECT and NONE are valid for this milisecond time

//this is the schedule the light is turned on to off time, 24 hrour format
const byte TIME_SCHEDULE_LIGHT_ON_HR = 12; // hour time the light should turn on
const byte TIME_SCHEDULE_LIGHT_ON_MM = 37; // minute time the light should turn on
const byte TIME_SCHEDULE_LIGHT_ON_SS = 0; // second time the light should turn on
const byte TIME_SCHEDULE_LIGHT_OFF_HR = 12; // hour time the light should turn off
const byte TIME_SCHEDULE_LIGHT_OFF_MM = 36; // minute time the light should turn off
const byte TIME_SCHEDULE_LIGHT_OFF_SS = 0; // second time the light should turn off

const unsigned int TIME_BT_RECV_MANUAL_SIGNAL = 5000; // ms time, if no manual signal received from bluetooth for this duration, mode is switched to auto

//pH calibration variables
const unsigned int PH_ADC_READING_4 = 676;    // probe analogRead average when in solution 4.01
const unsigned int PH_ADC_READING_7 = 594;    // probe analogRead average when in solution 7
const unsigned int PH_ADC_READING_9 = 493;    // probe analogRead average when in solution 9.18
const float PH_CALIBRATION_MULTIPLIER = 1.0;  // multiply the computed pH value to increase or decrease it
const float PH_CALIBRATION_OFFSET = 0.0;      // added value after the multiplier is applied

//ramping variables
const byte L298N_DRIVER_RAMPING_VALUE = 5; // increment ramping number for motor
const byte L298N_DRIVER_RAMPING_TIMING = 5; // milliseconds time to do ramping up/down
const byte HIGH_POWER_DRIVER_RAMPING_VALUE = 5; // increment ramping number for motor
const byte HIGH_POWER_DRIVER_RAMPING_TIMING = 5; // milliseconds time to do ramping up/down

// Data history constants
const byte DATA_HISTORY_SIZE = 24; // Store 24 hours of data (1 per hour)
const unsigned long TIME_DATA_HISTORY_INTERVAL = 3600000; // 1 hour in milliseconds

//Blynk data stream list
#define VPORT_SENSOR_PH V0
#define VPORT_SENSOR_TDS V1
#define VPORT_SENSOR_TEMP V2
#define VPORT_SENSOR_HUMIDITY V4
#define VPORT_ALARM_FUNGAL_INFECTION V3
#define VPORT_ALARM_WATER_LEVEL V5
#define VPORT_BUTTON_SENSOR_REFRESH V7
#define VPORT_BUTTON_RELAY_MANUAL V6
#define VPORT_BUTTON_RELAY_PH_UP V8
#define VPORT_BUTTON_RELAY_FAN V9
#define VPORT_BUTTON_RELAY_PESTICIDE V10
#define VPORT_BUTTON_RELAY_PH_DOWN V11
#define VPORT_BUTTON_RELAY_LIGHT V12
#define VPORT_BUTTON_RELAY_MISTER V13
#define VPORT_TIME_MISTER_SCHEDULE V14
#define VPORT_WATER_LEVEL_PERCENT V15  // New virtual pin for water level percentage

//Blynk LED color
#define BLYNK_GREEN     "#23C48E"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"

// Arduino PIN assignments
const byte PIN_US_TRIG = A0; // ultrasonic sensor for water level
const byte PIN_US_ECHO = A1; 
const byte PIN_PH = A2; // pH sensor for water pH level
const byte PIN_TDS = A4; // TDS sensor for measuring TDS value of water
const byte PIN_DHT = A5; // measure temperature and humidity
const byte PIN_L298N_ENA = 3; // pump motor pH UP
const byte PIN_L298N_IN1 = 2;
const byte PIN_L298N_IN2 = 4;
const byte PIN_L298N_ENB = 6; // pump motor pH DOWN
const byte PIN_L298N_IN3 = 5;
const byte PIN_L298N_IN4 = 7;
const byte PIN_2CHRELAY_IN1_FAN = 8; // relay for FAN
const byte PIN_2CHRELAY_IN2_LIGHT = 9; // relay for Light
const byte PIN_HIGHPOWERDRIVER_MISTER_R_EN = A14; // mister pump
const byte PIN_HIGHPOWERDRIVER_MISTER_L_EN = A15;
const byte PIN_HIGHPOWERDRIVER_MISTER_R_PWM = 12;
const byte PIN_HIGHPOWERDRIVER_MISTER_L_PWM = 13;
const byte PIN_HIGHPOWERDRIVER_PEST_R_EN = A6; // pest pump
const byte PIN_HIGHPOWERDRIVER_PEST_L_EN = A7;
const byte PIN_HIGHPOWERDRIVER_PEST_R_PWM = 10;
const byte PIN_HIGHPOWERDRIVER_PEST_L_PWM = 11;
const byte PIN_BUZZER = A11;
const byte PIN_LED_WATER_LEVEL_HIGH = A9;
const byte PIN_LED_WATER_LEVEL_MEDIUM = A8;
const byte PIN_LED_WATER_LEVEL_LOW = A10;
const byte PIN_LED_HB = A12;
const byte PIN_LED_BLYNK_RECV = A13;

#define DHTTYPE DHT11

#define RELAY_ON LOW
#define RELAY_OFF HIGH

// Data history structures
struct SensorData {
  float ph;
  unsigned int tds;
  float temperature;
  byte humidity;
  byte water_level_percent;
  unsigned long timestamp;
};

SensorData dataHistory[DATA_HISTORY_SIZE];
byte dataHistoryIndex = 0;
unsigned long lastDataHistorySave = 0;

const byte BT_BUF_SEND_LEN = 15;
char btbuf_send_pH[BT_BUF_SEND_LEN + 1] = "";
char btbuf_send_tds[BT_BUF_SEND_LEN + 1] = "";
char btbuf_send_humid[BT_BUF_SEND_LEN + 1] = "";
char btbuf_send_temp[BT_BUF_SEND_LEN + 1] = "";
char btbuf_send_pest[BT_BUF_SEND_LEN + 1] = "";
char btbuf_send_water_level[BT_BUF_SEND_LEN + 1] = "";

char btbuf_led_pH_up[BT_BUF_SEND_LEN + 1] = "";
char btbuf_led_pH_down[BT_BUF_SEND_LEN + 1] = "";
char btbuf_led_pest[BT_BUF_SEND_LEN + 1] = "";
char btbuf_led_mist[BT_BUF_SEND_LEN + 1] = "";
char btbuf_led_fan[BT_BUF_SEND_LEN + 1] = "";
char btbuf_led_light[BT_BUF_SEND_LEN + 1] = "";

// blynk receive mister schedule
long start_time_in_seconds, stop_time_in_seconds;
unsigned long t_blynk_connection; // reconnection blynk timer
unsigned long t_sensor_update; // sensor update timer for real-time monitoring

bool flag_past_esp32_cam_pest_detected_g;
byte past_water_level_g;
byte water_level_percent_g = 0; // New variable for water level percentage

// New variables for ESP32-CAM disease detection
bool flag_esp32_cam_fungal_detected_g = false;
bool flag_esp32_cam_bacterial_detected_g = false;
bool flag_esp32_cam_healthy_detected_g = false;
String current_crop_status_g = "NO CROP";

float temp_float_var;  //temporary float variable used for computations
unsigned long temp_long_var; // temporary long variable

//variable for date and time
byte nyr, nmo, ndy, nhh, nmm, nss;

//water TDS variables
const float TDS_WATER_TEMPERATURE = 25.0; // default room temperature when no water temperature is used
unsigned int water_tds_x10_g; // water TDS

//water pH variables
unsigned int water_ph_x10_g;  // water ph level
const byte PH_RAW_MAX = 20;
unsigned int ph_raw_data[PH_RAW_MAX];
byte ph_raw_data_i;

//pH l298n driver ramping variables
byte l298n_driver_current_pwm_ph_up, l298n_driver_current_pwm_ph_down;
byte l298n_driver_target_pwm_ph_up, l298n_driver_target_pwm_ph_down;

//high power drivers ramping variables
byte high_power_driver_target_pwm_mister, high_power_driver_target_pwm_pest; // used for ramping, target pwm to reach
byte high_power_driver_current_pwm_mister, high_power_driver_current_pwm_pest; // current pwm, increas/decrease to rech target pwm

//water pH calculated constants
float ph_below7_slope;
float ph_below7_intercept;
float ph_above7_slope;
float ph_above7_intercept;
byte user_ph_normal_min_x10, user_ph_normal_max_x10; // convert constant float to byte x10

//ultrasonic sensor variables
byte water_level_g; // water level, 0-high(17-24)Red, 1-Medium(8-16)Yellow, 2-Low(<=7)Green
unsigned long ultrasonic_distance; //measured distance of ultrasonic sensor
unsigned long duration;
unsigned int prev_us_distance, temp_us_distance; // used for debouncing
byte past_water_level;
unsigned long timer_debounce_water_level;
const byte WATER_LEVEL_HIGH = 2;
const byte WATER_LEVEL_MEDIUM = 1;
const byte WATER_LEVEL_LOW = 0;

//DHT variables
unsigned int air_humidity_g; // 0 - 99 range
unsigned int air_temperaturex10_g; // air temerpature
unsigned int threshold_air_temeparture_x10;
unsigned long timer_air_temepature_fan_debounce;

//ESP32-CAM variables
//Serial2 connected to ESP32-CAM
bool flag_esp32_cam_pest_detected_g; // true = ESP32-CAM sent "DETECT"
const byte ESP32_CAM_BUF_LEN = 12; // Increased for longer messages
char esp32_cam_buf[ESP32_CAM_BUF_LEN+1] = "";
unsigned int esp32_cam_buf_i;
unsigned long timer_serial_read;
unsigned long timer_espcam_pest_validity;

//lcd display variables
const byte LCD_LEN = 20;
char lcdbuf_0[LCD_LEN+1] = "";
char lcdbuf_1[LCD_LEN+1] = "";
char lcdbuf_2[LCD_LEN+1] = "";
char lcdbuf_3[LCD_LEN+1] = "";
byte state_disp;
unsigned long timer_disp;

//send status variables
bool flag_motor_state_pH_up; // true = pH up is ON
bool flag_motor_state_pH_down; // true = pH down is ON
bool flag_motor_state_pest; // true = ON
bool flag_motor_state_mist; // true = ON

//manual switch
bool flag_sw_manual_pH;
bool flag_sw_manual_pest;
bool flag_sw_manual_mist;
bool flag_sw_manual_fan;
bool flag_sw_manual_light;
bool flag_sw_manual_mode = false; // Global manual mode flag

//on/off switch
bool flag_sw_on_ph_up;
bool flag_sw_on_ph_down;
bool flag_sw_on_pest;
bool flag_sw_on_mist;
bool flag_sw_on_fan;
bool flag_sw_on_light;

//mist schedule from eeprom
unsigned long time_seconds_schedule_mist_on;
unsigned long time_seconds_schedule_mist_off;
unsigned long time_seconds_now_g;

//lcd display pest mist schedule countdown
byte lcd_schedule_mist_on_hr, lcd_schedule_mist_on_mm, lcd_schedule_mist_on_ss;
byte lcd_schedule_mist_off_hr, lcd_schedule_mist_off_mm, lcd_schedule_mist_off_ss;
byte lcd_countdown_mist_on_mm, lcd_countdown_mist_on_ss;
byte lcd_countdown_mist_off_mm, lcd_countdown_mist_off_ss;
bool flag_lcd_misting_ongoing; // used for lcd display to display timing intervals

//light schedule
unsigned long time_seconds_schedule_light_on;
unsigned long time_seconds_schedule_light_off;

//eeprom
const unsigned int EEADD_SCHEDULE_MIST_ON = 0; //4byte
const unsigned int EEADD_SCHEDULE_MIST_OFF = 4; //4byte
const unsigned int EEADD_DATA_HISTORY_START = 8; // Start address for data history

//ph control
byte state_control_pH;
unsigned long timer_control_ph;

//mist control
unsigned long timer_mist_duration_on_off;

//leds
bool flag_led_blynk;

//buzzer
bool flag_buzzer_btn;
unsigned long timer_buzzer;

//Serial1 connected to ESP32
//Serial2 connected to ESPcam

// #define BT Serial1
#define ESP32 Serial1
#define ESPCAM Serial2

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
DS3231 clock;
RTCDateTime dt;
DHT dht(PIN_DHT, DHTTYPE);
GravityTDS gravityTds;
ESP8266 wifi(&ESP32);

//eeprom save unsigned long number
void eewrite_num_ulong(unsigned int eeadd, unsigned long num)
{
  byte a;
  a = (byte)num;
  EEPROM.write(eeadd, a);

  a = (byte)(num >> 8);
  EEPROM.write(eeadd + 1, a);

  a = (byte)(num >> 16);
  EEPROM.write(eeadd + 2, a);

  a = (byte)(num >> 24);
  EEPROM.write(eeadd + 3, a);
}

//eeprom load unsigned long number
unsigned long eeload_num_ulong(unsigned int eeadd)
{
  unsigned long retVal;

  retVal = (unsigned long)(EEPROM.read(eeadd));
  retVal |= (unsigned long)EEPROM.read(eeadd + 1) << 8;
  retVal |= (unsigned long)EEPROM.read(eeadd + 2) << 16;
  retVal |= (unsigned long)EEPROM.read(eeadd + 3) << 24;

  return retVal;
}

// Calculate water level percentage
byte calculateWaterLevelPercent()
{
  if (ultrasonic_distance <= ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_LOW)
  {
    // 0-33% (Low)
    return map(ultrasonic_distance, 0, ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_LOW, 0, 33);
  }
  else if (ultrasonic_distance <= ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_MEDIUM)
  {
    // 34-66% (Medium)
    return map(ultrasonic_distance, ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_LOW + 1, 
               ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_MEDIUM, 34, 66);
  }
  else if (ultrasonic_distance <= ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_HIGH)
  {
    // 67-100% (High)
    return map(ultrasonic_distance, ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_MEDIUM + 1, 
               ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_HIGH, 67, 100);
  }
  else
  {
    return 100; // Full
  }
}

//ultrasonic check distance
void ultrasonic_check()
{
  static unsigned long t;
  if (millis() - t < 50) return;
  t = millis();

  // Clears the trigPin
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(PIN_US_ECHO, HIGH);
  // Calculating the distance
  temp_us_distance = (float)duration * 0.034 / 2;

  //limit distance to valid range
  if (temp_us_distance < 2 || temp_us_distance > ULTRASONIC_SENSOR_MAX_DISTANCE)
  {
    temp_us_distance = ULTRASONIC_SENSOR_MAX_DISTANCE;
  }

  //check if distance data is same as previous data
  if (prev_us_distance == temp_us_distance)
  {
    //get distance starting from bottom of container
    ultrasonic_distance = ULTRASONIC_SENSOR_MAX_DISTANCE - temp_us_distance; // debounced distance
  }
  prev_us_distance = temp_us_distance;

  //water level based on ultrasonic distance
  if (ultrasonic_distance <= ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_LOW)
  {
    if (past_water_level == WATER_LEVEL_LOW) 
    {
      if (millis() - timer_debounce_water_level >= TIME_DEBOUNCE_WATER_LEVEL)
      {
        water_level_g = WATER_LEVEL_LOW;
        water_level_percent_g = calculateWaterLevelPercent();
      }
    }
    else
    {
      timer_debounce_water_level = millis();
    }
    past_water_level = WATER_LEVEL_LOW;
  }
  else if (ultrasonic_distance <= ULTRASONIC_SENSOR_DISTANCE_WATER_LEVEL_MEDIUM)
  {
    if (past_water_level == WATER_LEVEL_MEDIUM)
    {
      if (millis() - timer_debounce_water_level >= TIME_DEBOUNCE_WATER_LEVEL)
      {
        water_level_g = WATER_LEVEL_MEDIUM;
        water_level_percent_g = calculateWaterLevelPercent();
      }
    }
    else
    {
      timer_debounce_water_level = millis();
    }
    past_water_level = WATER_LEVEL_MEDIUM;
  }
  else
  {
    if (past_water_level == WATER_LEVEL_HIGH)
    {
      if (millis() - timer_debounce_water_level >= TIME_DEBOUNCE_WATER_LEVEL)
      {
        water_level_g = WATER_LEVEL_HIGH;
        water_level_percent_g = calculateWaterLevelPercent();
      }
    }
    else
    {
      timer_debounce_water_level = millis();
    }
    past_water_level = WATER_LEVEL_HIGH;
  }
}

//read analog pin x10
unsigned int analogReadx10(byte pin) {
  temp_long_var = 0;
  for (byte i = 0; i < 10; i++) {
    temp_long_var += analogRead(pin);
  }
  temp_long_var /= 10;
  return (unsigned int)temp_long_var;
}

// get the water ph from sensor
void water_ph()
{
  static unsigned long t, time_debounce_ph;
  static byte past_ph;
  if (millis() - t < 4) return;
  t = millis();

  ph_raw_data[ph_raw_data_i] = analogReadx10(PIN_PH);

  //get average of all data in array
  temp_float_var = 0;
  for (byte i = 0; i < PH_RAW_MAX; i++) {
    temp_float_var += ph_raw_data[i];
  }
  temp_float_var /= PH_RAW_MAX;

  if (temp_float_var > PH_ADC_READING_7)  // pH is expected to be 7 below
  {
    temp_float_var = (temp_float_var - ph_below7_intercept) / ph_below7_slope;
  }
  else  //ph is expected to be 7 and up
  {
    temp_float_var = (temp_float_var - ph_above7_intercept) / ph_above7_slope;
  }

  //apply multiplier and offset
  temp_float_var = ((temp_float_var * PH_CALIBRATION_MULTIPLIER) + PH_CALIBRATION_OFFSET) * 10;

  //limit range
  if (temp_float_var > 140) temp_float_var = 140;
  if (temp_float_var <= 0) temp_float_var = 0;

  //check if past pH has only 1 deviation from current reading
  if (past_ph >= (byte)temp_float_var && past_ph <= ((byte)temp_float_var + 1) )
  {
    if (millis() - time_debounce_ph >= 200)
    {
      water_ph_x10_g = past_ph; // debounced water pH
      past_ph = temp_float_var;
    }
  }
  else
  {
    time_debounce_ph = millis();
    past_ph = temp_float_var;
  }

  ph_raw_data_i++;
  if (ph_raw_data_i >= PH_RAW_MAX)
  {
    ph_raw_data_i = 0;
  }
}

//get water tds
void water_tds()
{
  static unsigned long t;
  if (millis() - t < 1000) return;
  t = millis();

  gravityTds.setTemperature(TDS_WATER_TEMPERATURE);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate 
  temp_float_var = gravityTds.getTdsValue();  // then get the value

  water_tds_x10_g = temp_float_var * 10; //water TDS
}

//get air temperature and air humidity using DHT11
void air_humidity_temperature()
{
  static unsigned long t;
  if (millis() - t < 1000) return;
  t = millis();

  air_humidity_g = dht.readHumidity();
  air_temperaturex10_g = dht.readTemperature() * 10;
}

// Enhanced ESP32-CAM check with multiple disease detection
// Simplified ESP32-CAM check - receives raw data
void esp32_cam_check()
{
  //if no data is received from espcam for a period of time, reset status
  if (current_crop_status_g != "NO CROP")
  {
    if (millis() - timer_espcam_pest_validity >= TIME_ESPCAM_PEST_SIGNAL_VALIDITY)
    {
      current_crop_status_g = "NO CROP";
      flag_esp32_cam_pest_detected_g = false;
    }
  }

  if (ESPCAM.available() <= 0) return;

  // Clear buffer and read new data
  memset(esp32_cam_buf, 0, ESP32_CAM_BUF_LEN);
  esp32_cam_buf_i = 0;
  timer_serial_read = millis();
  
  // Read all available data
  while(millis() - timer_serial_read <= 10)
  {
    if (ESPCAM.available())
    {
      esp32_cam_buf[esp32_cam_buf_i] = ESPCAM.read();
      esp32_cam_buf_i++;
    }
    if (esp32_cam_buf_i >= ESP32_CAM_BUF_LEN) break;
  }
  while(ESPCAM.available()) ESPCAM.read();

  // If we received any data, update status
  if (esp32_cam_buf_i > 0)
  {
    // Convert buffer to string and update crop status
    esp32_cam_buf[esp32_cam_buf_i] = '\0'; // Null terminate
    current_crop_status_g = String(esp32_cam_buf);
    
    // Set pest detection flag if it's not "NO CROP"
    if (current_crop_status_g != "NO CROP")
    {
      flag_esp32_cam_pest_detected_g = true;
    }
    else
    {
      flag_esp32_cam_pest_detected_g = false;
    }
    
    timer_espcam_pest_validity = millis();
    
    // Debug output
    Serial.print("ESP32-CAM sent: ");
    Serial.println(current_crop_status_g);
  }
}

void set_pwm_l298n_driver_ph_up(byte target_pwm) // pwm range 0 - 255
{
  // Prevent simultaneous pH up and pH down
  if (target_pwm > 0 && l298n_driver_target_pwm_ph_down > 0)
  {
    // Don't allow pH up if pH down is active
    return;
  }
  
  l298n_driver_target_pwm_ph_up = target_pwm;
  if (target_pwm > 0)
  {
    flag_motor_state_pH_up = true;
  }
  else
  {
    flag_motor_state_pH_up = false;
  }
}

void set_pwm_l298n_driver_ph_down(byte target_pwm) // pwm range 0 - 255
{
  // Prevent simultaneous pH up and pH down
  if (target_pwm > 0 && l298n_driver_target_pwm_ph_up > 0)
  {
    // Don't allow pH down if pH up is active
    return;
  }
  
  l298n_driver_target_pwm_ph_down = target_pwm;
  if (target_pwm > 0)
  {
    flag_motor_state_pH_down = true;
  }
  else
  {
    flag_motor_state_pH_down = false;
  }
}

void l298n_driver_control()
{
  static unsigned long t;
  if (millis() - t < L298N_DRIVER_RAMPING_TIMING) return;
  t = millis();

  //pH up ramping routine
  if (l298n_driver_current_pwm_ph_up == 0 && l298n_driver_target_pwm_ph_up == 0)
  {
    //turn off motor, target and current is 0
    analogWrite(PIN_L298N_ENA,0);
    digitalWrite(PIN_L298N_IN1, LOW);
    digitalWrite(PIN_L298N_IN2, LOW);
  }
  else if (l298n_driver_current_pwm_ph_up < l298n_driver_target_pwm_ph_up)
  {
    //routine if current is lower than target pwm
    digitalWrite(PIN_L298N_IN1, HIGH);
    digitalWrite(PIN_L298N_IN2, LOW);

    if (l298n_driver_target_pwm_ph_up <= L298N_DRIVER_RAMPING_VALUE)
    {
      //target is below ramping value
      l298n_driver_current_pwm_ph_up = l298n_driver_target_pwm_ph_up;
    }
    else if (l298n_driver_current_pwm_ph_up <= l298n_driver_target_pwm_ph_up - L298N_DRIVER_RAMPING_VALUE)
    {
      //ramping current pwm does not exceed target pwm, just ramp up
      l298n_driver_current_pwm_ph_up += L298N_DRIVER_RAMPING_VALUE;
    }
    else
    {
      //ramping current pwm exceed target, current pwm = target pwm
      l298n_driver_current_pwm_ph_up = l298n_driver_target_pwm_ph_up;
    }

    analogWrite(PIN_L298N_ENA,l298n_driver_current_pwm_ph_up);
  }
  else if (l298n_driver_current_pwm_ph_up > l298n_driver_target_pwm_ph_up)
  {
    //routine if current is equal or greater than target
    digitalWrite(PIN_L298N_IN1, HIGH);
    digitalWrite(PIN_L298N_IN2, LOW);

    if (l298n_driver_current_pwm_ph_up <= L298N_DRIVER_RAMPING_VALUE)
    {
      //subtracting ramp value to current results in negative, make current pwm = target pwm
      l298n_driver_current_pwm_ph_up = l298n_driver_target_pwm_ph_up;
    }
    else
    {
      //current pwm is greater than ramping value and target value, just ramp down
      l298n_driver_current_pwm_ph_up -= L298N_DRIVER_RAMPING_VALUE;
    }

    analogWrite(PIN_L298N_ENA,l298n_driver_current_pwm_ph_up);
  }

  //pH down ramping routine
  if (l298n_driver_current_pwm_ph_down == 0 && l298n_driver_target_pwm_ph_down == 0)
  {
    //turn off motor, target and current is 0
    analogWrite(PIN_L298N_ENB,0);
    digitalWrite(PIN_L298N_IN3, LOW);
    digitalWrite(PIN_L298N_IN4, LOW);
  }
  else if (l298n_driver_current_pwm_ph_down < l298n_driver_target_pwm_ph_down)
  {
    //routine if current is lower than target pwm
    digitalWrite(PIN_L298N_IN3, HIGH);
    digitalWrite(PIN_L298N_IN4, LOW);

    if (l298n_driver_target_pwm_ph_down <= L298N_DRIVER_RAMPING_VALUE)
    {
      //target is below ramping value
      l298n_driver_current_pwm_ph_down = l298n_driver_target_pwm_ph_down;
    }
    else if (l298n_driver_current_pwm_ph_down <= l298n_driver_target_pwm_ph_down - L298N_DRIVER_RAMPING_VALUE)
    {
      //ramping current pwm does not exceed target pwm, just ramp up
      l298n_driver_current_pwm_ph_down += L298N_DRIVER_RAMPING_VALUE;
    }
    else
    {
      //ramping current pwm exceed target, current pwm = target pwm
      l298n_driver_current_pwm_ph_down = l298n_driver_target_pwm_ph_down;
    }

    analogWrite(PIN_L298N_ENB,l298n_driver_current_pwm_ph_down);
  }
  else if (l298n_driver_current_pwm_ph_down > l298n_driver_target_pwm_ph_down)
  {
    //routine if current is equal or greater than target
    digitalWrite(PIN_L298N_IN3, HIGH);
    digitalWrite(PIN_L298N_IN4, LOW);

    if (l298n_driver_current_pwm_ph_down <= L298N_DRIVER_RAMPING_VALUE)
    {
      //subtracting ramp value to current results in negative, make current pwm = target pwm
      l298n_driver_current_pwm_ph_down = l298n_driver_target_pwm_ph_down;
    }
    else
    {
      //current pwm is greater than ramping value and target value, just ramp down
      l298n_driver_current_pwm_ph_down -= L298N_DRIVER_RAMPING_VALUE;
    }

    analogWrite(PIN_L298N_ENB,l298n_driver_current_pwm_ph_down);
  }
}

//2CH relay controls
void relay_fan_on()
{
  digitalWrite(PIN_2CHRELAY_IN1_FAN, RELAY_ON);
}
void relay_fan_off()
{
  digitalWrite(PIN_2CHRELAY_IN1_FAN, RELAY_OFF);
}
void relay_light_on()
{
  digitalWrite(PIN_2CHRELAY_IN2_LIGHT, RELAY_ON);
}
void relay_light_off()
{
  digitalWrite(PIN_2CHRELAY_IN2_LIGHT, RELAY_OFF);
}

void set_pwm_high_power_driver_mister(byte target_pwm) // pwm range 0 - 255
{
  high_power_driver_target_pwm_mister = target_pwm;
  if (target_pwm > 0)
  {
    flag_motor_state_mist = true;
  }
  else
  {
    flag_motor_state_mist = false;
  }
}
void set_pwm_high_power_driver_pest(byte target_pwm) // pwm range 0 - 255
{
  high_power_driver_target_pwm_pest = target_pwm;
  if (target_pwm > 0)
  {
    flag_motor_state_pest = true;
  }
  else
  {
    flag_motor_state_pest = false;
  }
}

void high_power_driver_control()
{
  static unsigned long t;
  if (millis() - t < HIGH_POWER_DRIVER_RAMPING_TIMING) return;
  t = millis();

  //mister ramping
  if (high_power_driver_current_pwm_mister == 0 && high_power_driver_target_pwm_mister == 0)
  {
    //turn off motor, target and current is 0
    digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_R_EN, LOW);
    digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_L_EN, LOW);
    analogWrite(PIN_HIGHPOWERDRIVER_MISTER_R_PWM, 0);
    analogWrite(PIN_HIGHPOWERDRIVER_MISTER_L_PWM, 0);
  }
  else if (high_power_driver_current_pwm_mister < high_power_driver_target_pwm_mister)
  {
    //routine if current is lower than target pwm
    digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_R_EN, HIGH);
    digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_L_EN, HIGH);

    if (high_power_driver_target_pwm_mister <= HIGH_POWER_DRIVER_RAMPING_VALUE)
    {
      //target is below ramping value
      high_power_driver_current_pwm_mister = high_power_driver_target_pwm_mister;
    }
    else if (high_power_driver_current_pwm_mister <= high_power_driver_target_pwm_mister - HIGH_POWER_DRIVER_RAMPING_VALUE)
    {
      //ramping current pwm does not exceed target pwm, just ramp up
      high_power_driver_current_pwm_mister += HIGH_POWER_DRIVER_RAMPING_VALUE;
    }
    else
    {
      //ramping currnet pwm exceed target, current pwm = target pwm
      high_power_driver_current_pwm_mister = high_power_driver_target_pwm_mister;
    }

    analogWrite(PIN_HIGHPOWERDRIVER_MISTER_R_PWM, 0);
    analogWrite(PIN_HIGHPOWERDRIVER_MISTER_L_PWM, high_power_driver_current_pwm_mister);
  }
  else if (high_power_driver_current_pwm_mister > high_power_driver_target_pwm_mister)
  {
    //routine if current is equal or greater than target
    digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_R_EN, HIGH);
    digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_L_EN, HIGH);

    if (high_power_driver_current_pwm_mister <= HIGH_POWER_DRIVER_RAMPING_VALUE)
    {
      //subtracting ramp value to current results in negative, make current pwm = target pwm
      high_power_driver_current_pwm_mister = high_power_driver_target_pwm_mister;
    }
    else
    {
      //current pwm is greater than ramping value and target value, just ramp down
      high_power_driver_current_pwm_mister -= HIGH_POWER_DRIVER_RAMPING_VALUE;
    }

    analogWrite(PIN_HIGHPOWERDRIVER_MISTER_R_PWM, 0);
    analogWrite(PIN_HIGHPOWERDRIVER_MISTER_L_PWM, high_power_driver_current_pwm_mister);
  }

  //pest ramping routine
  if (high_power_driver_current_pwm_pest == 0 && high_power_driver_target_pwm_pest == 0)
  {
    //turn off motor, target and current is 0
    digitalWrite(PIN_HIGHPOWERDRIVER_PEST_R_EN, LOW);
    digitalWrite(PIN_HIGHPOWERDRIVER_PEST_L_EN, LOW);
    analogWrite(PIN_HIGHPOWERDRIVER_PEST_R_PWM, 0);
    analogWrite(PIN_HIGHPOWERDRIVER_PEST_L_PWM, 0);
  }
  else if (high_power_driver_current_pwm_pest < high_power_driver_target_pwm_pest)
  {
    //routine if current is lower than target pwm
    digitalWrite(PIN_HIGHPOWERDRIVER_PEST_R_EN, HIGH);
    digitalWrite(PIN_HIGHPOWERDRIVER_PEST_L_EN, HIGH);

    if (high_power_driver_target_pwm_pest <= HIGH_POWER_DRIVER_RAMPING_VALUE)
    {
      //target is below ramping value
      high_power_driver_current_pwm_pest = high_power_driver_target_pwm_pest;
    }
    else if (high_power_driver_current_pwm_pest <= high_power_driver_target_pwm_pest - HIGH_POWER_DRIVER_RAMPING_VALUE)
    {
      //ramping current pwm does not exceed target pwm, just ramp up
      high_power_driver_current_pwm_pest += HIGH_POWER_DRIVER_RAMPING_VALUE;
    }
    else
    {
      //ramping currnet pwm exceed target, current pwm = target pwm
      high_power_driver_current_pwm_pest = high_power_driver_target_pwm_pest;
    }

    analogWrite(PIN_HIGHPOWERDRIVER_PEST_R_PWM, 0);
    analogWrite(PIN_HIGHPOWERDRIVER_PEST_L_PWM, high_power_driver_current_pwm_pest);
  }
  else if (high_power_driver_current_pwm_pest > high_power_driver_target_pwm_pest)
  {
    //routine if current is equal or greater than target
    digitalWrite(PIN_HIGHPOWERDRIVER_PEST_R_EN, HIGH);
    digitalWrite(PIN_HIGHPOWERDRIVER_PEST_L_EN, HIGH);

    if (high_power_driver_current_pwm_pest <= HIGH_POWER_DRIVER_RAMPING_VALUE)
    {
      //subtracting ramp value to current results in negative, make current pwm = target pwm
      high_power_driver_current_pwm_pest = high_power_driver_target_pwm_pest;
    }
    else
    {
      //current pwm is greater than ramping value and target value, just ramp down
      high_power_driver_current_pwm_pest -= HIGH_POWER_DRIVER_RAMPING_VALUE;
    }

    analogWrite(PIN_HIGHPOWERDRIVER_PEST_R_PWM, 0);
    analogWrite(PIN_HIGHPOWERDRIVER_PEST_L_PWM, high_power_driver_current_pwm_pest);
  }
}

//get time every second
void check_date_time()
{
  static unsigned long t;
  if (millis() - t < 1000) return; // update date and time every second
  t = millis();

  dt = clock.getDateTime();

  nyr = dt.year - 2000; // get only the last 2 digit of the year
  nmo = dt.month;
  ndy = dt.day;
  nhh = dt.hour;
  nmm = dt.minute;
  nss = dt.second;

  time_seconds_now_g = ((unsigned long)nhh * 3600) + (nmm * 60) + nss;
}

// Save data to history
void saveDataToHistory()
{
  if (millis() - lastDataHistorySave >= TIME_DATA_HISTORY_INTERVAL)
  {
    lastDataHistorySave = millis();
    
    dataHistory[dataHistoryIndex].ph = water_ph_x10_g / 10.0;
    dataHistory[dataHistoryIndex].tds = water_tds_x10_g / 10;
    dataHistory[dataHistoryIndex].temperature = air_temperaturex10_g / 10.0;
    dataHistory[dataHistoryIndex].humidity = air_humidity_g;
    dataHistory[dataHistoryIndex].water_level_percent = water_level_percent_g;
    dataHistory[dataHistoryIndex].timestamp = millis();
    
    dataHistoryIndex++;
    if (dataHistoryIndex >= DATA_HISTORY_SIZE)
    {
      dataHistoryIndex = 0;
    }
  }
}

// Send history data to Blynk charts
void sendHistoryToBlynk() {
  static unsigned long lastHistorySend = 0;
  
  if (millis() - lastHistorySend >= 300000) { // Send every 5 minutes
    lastHistorySend = millis();
    
    // Send current data point to charts for historical tracking
    Blynk.virtualWrite(VPORT_SENSOR_PH, water_ph_x10_g / 10.0);
    Blynk.virtualWrite(VPORT_SENSOR_TDS, water_tds_x10_g / 10);
    Blynk.virtualWrite(VPORT_SENSOR_TEMP, air_temperaturex10_g / 10.0);
    Blynk.virtualWrite(VPORT_SENSOR_HUMIDITY, air_humidity_g);
    Blynk.virtualWrite(VPORT_WATER_LEVEL_PERCENT, water_level_percent_g);
  }
}

//control LEDs
void led_contol()
{
  static unsigned long t;
  if (millis() - t <= 10) return;
  t = millis();

  static unsigned long t_led_hb;
  //toggle LED state every 500 milliseconds
  if (millis() - t_led_hb >= 500)
  {
    t_led_hb = millis();
    digitalWrite(PIN_LED_HB, !digitalRead(PIN_LED_HB));
  }

  //water level LEDs
  if (water_level_g == WATER_LEVEL_HIGH)
  {
    digitalWrite(PIN_LED_WATER_LEVEL_HIGH, HIGH);
    digitalWrite(PIN_LED_WATER_LEVEL_MEDIUM, LOW);
    digitalWrite(PIN_LED_WATER_LEVEL_LOW, LOW);
  }
  else if (water_level_g == WATER_LEVEL_MEDIUM)
  {
    digitalWrite(PIN_LED_WATER_LEVEL_HIGH, LOW);
    digitalWrite(PIN_LED_WATER_LEVEL_MEDIUM, HIGH);
    digitalWrite(PIN_LED_WATER_LEVEL_LOW, LOW);
  }
  else if (water_level_g == WATER_LEVEL_LOW)
  {
    digitalWrite(PIN_LED_WATER_LEVEL_HIGH, LOW);
    digitalWrite(PIN_LED_WATER_LEVEL_MEDIUM, LOW);
    digitalWrite(PIN_LED_WATER_LEVEL_LOW, HIGH);
  }

  static unsigned long t_blynk_recv_signal;
  if (digitalRead(PIN_LED_BLYNK_RECV) == HIGH)
  {
    if (millis() - t_blynk_recv_signal >= 100)
    {
      digitalWrite(PIN_LED_BLYNK_RECV, LOW);
    }
  }
  else
  {
    if (flag_led_blynk)
    {
      flag_led_blynk = false;
      digitalWrite(PIN_LED_BLYNK_RECV, HIGH);
      t_blynk_recv_signal = millis();
    }
  }
}

void change_state_disp(byte ns)
{
  state_disp = ns;
  timer_disp = millis();
}

void disp_idle()
{
  switch(state_disp)
  {
  case 0:
    if (millis() - timer_disp >= 200)
    {
      sprintf(lcdbuf_0, "%02d/%02d/%02d %02d:%02d:%02d   ", nyr,nmo,ndy,nhh,nmm,nss);

      temp_long_var = water_tds_x10_g/10;
      if (temp_long_var > 999) temp_long_var = 999;
      sprintf(lcdbuf_1, "pH:%02d.%d TDS:%03uppm  ", water_ph_x10_g/10, water_ph_x10_g%10,(unsigned int)temp_long_var);

      if (flag_esp32_cam_pest_detected_g)
      {
        sprintf(lcdbuf_2,"H:%02d%% T:%02d.%dC %s", air_humidity_g, air_temperaturex10_g/10, 
                air_temperaturex10_g%10, current_crop_status_g.substring(0,8).c_str());
      }
      else
      {
        sprintf(lcdbuf_2,"H:%02d%% T:%02d.%dC %s", air_humidity_g, air_temperaturex10_g/10, 
                air_temperaturex10_g%10, current_crop_status_g.substring(0,8).c_str());
      }

      if (flag_lcd_misting_ongoing && flag_sw_manual_mist == false) //misting now & auto mode, show countdown
      {
        if(flag_motor_state_mist) // mister motor is ON
        {
          sprintf(lcdbuf_3,"MIST ON:%02d:%02d WL:%02d%%",lcd_countdown_mist_on_mm,lcd_countdown_mist_on_ss, water_level_percent_g);
        }
        else
        {
          sprintf(lcdbuf_3,"MIST OFF:%02d:%02d WL:%02d%%",lcd_countdown_mist_off_mm,lcd_countdown_mist_off_ss, water_level_percent_g);
        }
      }
      else // show schedule of misting
      {
        sprintf(lcdbuf_3,"M:%02d:%02d-%02d:%02d WL:%02d%%",
          lcd_schedule_mist_on_hr, lcd_schedule_mist_on_mm,
          lcd_schedule_mist_off_hr,lcd_schedule_mist_off_mm, water_level_percent_g);
      }

      lcd.setCursor(0,0); lcd.print(lcdbuf_0);
      change_state_disp(1);
    }
    break;

  case 1:
      lcd.setCursor(0,1); lcd.print(lcdbuf_1);
      change_state_disp(2);
    break;

  case 2:
      lcd.setCursor(0,2); lcd.print(lcdbuf_2);
      change_state_disp(3);
    break;

  case 3:
      lcd.setCursor(0,3); lcd.print(lcdbuf_3);
      change_state_disp(0);
    break;
  }
}

void change_state_control_pH(byte ns)
{
  state_control_pH = ns;
  timer_control_ph = millis();
}

//control motor for pH
void control_pH()
{
  static unsigned long t;
  if (millis() - t <= 3) return;
  t = millis();

  switch(state_control_pH)
  {
  case 0: // auto
    if (flag_sw_manual_pH == true)
    {
      change_state_control_pH(100); // goto manual routine
    }
    else
    {
      set_pwm_l298n_driver_ph_up(0);
      set_pwm_l298n_driver_ph_down(0);

      if (water_ph_x10_g < user_ph_normal_min_x10)
      {
        change_state_control_pH(10);
      }
      else if (water_ph_x10_g > user_ph_normal_max_x10)
      {
        change_state_control_pH(20);
      }
    }
    break;

  case 10: // debounce ph up
    if (flag_sw_manual_pH == true)
    {
      change_state_control_pH(100); // goto manual routine
    }
    else if (water_ph_x10_g < user_ph_normal_min_x10)
    {
      if (millis() - timer_control_ph >= TIME_DEBOUNCE_ABNORMAL_PH)
      {
        set_pwm_l298n_driver_ph_up(255);
        change_state_control_pH(11);
      }
    }
    else
    {
      change_state_control_pH(0);
    }
    break;

  case 11: // dispense pH up solution
    if (flag_sw_manual_pH == true)
    {
      change_state_control_pH(100); // goto manual routine
    }
    else if (millis() - timer_control_ph >= TIME_ABNORMAL_PH_DISPENSE_SOLUTION)
    {
      set_pwm_l298n_driver_ph_up(0);
      change_state_control_pH(12);
    }
    break;

  case 12: // waiting for water to stable after pH up solution
    if (flag_sw_manual_pH == true)
    {
      change_state_control_pH(100); // goto manual routine
    }
    else if (millis() - timer_control_ph >= TIME_ABNORMAL_PH_WAITING_FOR_STABLE)
    {
      change_state_control_pH(0); // back to checking water pH
    }
    break;

  case 20: // debounce pH down
    if (flag_sw_manual_pH == true)
    {
      change_state_control_pH(100); // goto manual routine
    }
    else if (water_ph_x10_g > user_ph_normal_max_x10)
    {
      if (millis() - timer_control_ph >= TIME_DEBOUNCE_ABNORMAL_PH)
      {
        set_pwm_l298n_driver_ph_down(255);
        change_state_control_pH(21);
      }
    }
    else
    {
      change_state_control_pH(0);
    }
    break;

  case 21: // dispense pH down solution
    if (flag_sw_manual_pH == true)
    {
      change_state_control_pH(100); // goto manual routine
    }
    else if (millis() - timer_control_ph >= TIME_ABNORMAL_PH_DISPENSE_SOLUTION)
    {
      set_pwm_l298n_driver_ph_down(0);
      change_state_control_pH(12);
    }
    break;

  case 100: // manual
    if (flag_sw_manual_pH == false)
    {
      //turn off motors
      set_pwm_l298n_driver_ph_up(0);
      set_pwm_l298n_driver_ph_down(0);

      change_state_control_pH(0); //goto auto routine
    }
    else
    {
      if (flag_sw_on_ph_up)
      {
        set_pwm_l298n_driver_ph_up(255);
      }
      else
      {
        set_pwm_l298n_driver_ph_up(0);
      }
      
      if (flag_sw_on_ph_down)
      {
        set_pwm_l298n_driver_ph_down(255);
      }
      else
      {
        set_pwm_l298n_driver_ph_down(0);
      }
    }
    break;
  }
}

//control motor for pest
void control_pest()
{
  static unsigned long t;
  if (millis() - t <= 3) return;
  t = millis();

  if (flag_sw_manual_pest == false) // auto routine
  {
    //state is based on ESPCAM detection
    if (flag_esp32_cam_pest_detected_g == true)
    {
      set_pwm_high_power_driver_pest(255);
    }
    else
    {
      set_pwm_high_power_driver_pest(0);
    }
  }
  else // manual routine
  {
    if (flag_sw_on_pest == true)
    {
      set_pwm_high_power_driver_pest(255);
    }
    else
    {
      set_pwm_high_power_driver_pest(0);
    }
  }
}

//control motor for misting
void control_mist() {
  static unsigned long t;
  if (millis() - t <= 3) return;
  t = millis();

  if (flag_sw_manual_mist == false) // auto routine
  {
    if (time_seconds_schedule_mist_off > time_seconds_schedule_mist_on)
    {
      if (time_seconds_now_g >= time_seconds_schedule_mist_on && time_seconds_now_g <= time_seconds_schedule_mist_off)
      {
        flag_lcd_misting_ongoing = true;
        if (flag_motor_state_mist == true)
        {
          if (millis() - timer_mist_duration_on_off >= TIME_INTERVAL_MIST_ON_DURATION)
          {
            timer_mist_duration_on_off = millis();
            set_pwm_high_power_driver_mister(0); // off
          }
          else
          {
            temp_long_var = (unsigned long)TIME_INTERVAL_MIST_ON_DURATION - (millis() - timer_mist_duration_on_off);
            temp_long_var /= 1000;
            lcd_countdown_mist_on_mm = temp_long_var /  60;
            lcd_countdown_mist_on_ss = temp_long_var %  60;
          }
        }
        else
        {
          if (millis() - timer_mist_duration_on_off >= TIME_INTERVAL_MIST_OFF_DURATION)
          {
            timer_mist_duration_on_off = millis();
            set_pwm_high_power_driver_mister(255); // on
          }
          else
          {
            temp_long_var = (unsigned long)TIME_INTERVAL_MIST_OFF_DURATION - (millis() - timer_mist_duration_on_off);
            temp_long_var /= 1000;
            lcd_countdown_mist_off_mm = temp_long_var /  60;
            lcd_countdown_mist_off_ss = temp_long_var %  60;
          }
        }
      }
      else
      {
        flag_lcd_misting_ongoing = false;
        set_pwm_high_power_driver_mister(0); // off
      }
    }
    else
    {
      if (time_seconds_now_g < time_seconds_schedule_mist_on && time_seconds_now_g > time_seconds_schedule_mist_off)
      {
        flag_lcd_misting_ongoing = false;
        set_pwm_high_power_driver_mister(0); // off
      }
      else
      {
        flag_lcd_misting_ongoing = true;
        if (flag_motor_state_mist == true)
        {
          if (millis() - timer_mist_duration_on_off >= TIME_INTERVAL_MIST_ON_DURATION)
          {
            timer_mist_duration_on_off = millis();
            set_pwm_high_power_driver_mister(0); // off
          }
          else
          {
            temp_long_var = (unsigned long)TIME_INTERVAL_MIST_ON_DURATION - (millis() - timer_mist_duration_on_off);
            temp_long_var /= 1000;
            lcd_countdown_mist_on_mm = temp_long_var /  60;
            lcd_countdown_mist_on_ss = temp_long_var %  60;
          }
        }
        else
        {
          if (millis() - timer_mist_duration_on_off >= TIME_INTERVAL_MIST_OFF_DURATION)
          {
            timer_mist_duration_on_off = millis();
            set_pwm_high_power_driver_mister(255); // on
          }
          else
          {
            temp_long_var = (unsigned long)TIME_INTERVAL_MIST_OFF_DURATION - (millis() - timer_mist_duration_on_off);
            temp_long_var /= 1000;
            lcd_countdown_mist_off_mm = temp_long_var /  60;
            lcd_countdown_mist_off_ss = temp_long_var %  60;
          }
        }
      }
    }
  }
  else //manual routine
  {
    if (flag_sw_on_mist == true)
    {
      set_pwm_high_power_driver_mister(255); // on
    }
    else
    {
      set_pwm_high_power_driver_mister(0); // off
    }
  }
}

//control relay for fan and light
void control_fan_light()
{
  static unsigned long t;
  if (millis() - t <= 10) return;
  t = millis();

  if (flag_sw_manual_fan == false) // auto routine
  {
    //fan
    if (air_temperaturex10_g >= threshold_air_temeparture_x10)
    {
      if (digitalRead(PIN_2CHRELAY_IN1_FAN) == RELAY_OFF)
      {
        if (millis() - timer_air_temepature_fan_debounce >= TIME_DEBOUCNE_AIR_TEMPERATURE_FAN)
        {
          timer_air_temepature_fan_debounce = millis();
          relay_fan_on();
        }
      }
      else
      {
        timer_air_temepature_fan_debounce = millis();
      }
    }
    else
    {
      if (digitalRead(PIN_2CHRELAY_IN1_FAN) == RELAY_ON)
      {
        if (millis() - timer_air_temepature_fan_debounce >= TIME_DEBOUCNE_AIR_TEMPERATURE_FAN)
        {
          timer_air_temepature_fan_debounce = millis();
          relay_fan_off();
        }
      }
      else
      {
        timer_air_temepature_fan_debounce = millis();
      }
    }
  }
  else // manual routine
  {
    //fan
    if (flag_sw_on_fan == true)
    {
      relay_fan_on();
    }
    else
    {
      relay_fan_off();
      timer_air_temepature_fan_debounce = millis();
    }
  }

  if (flag_sw_manual_light == false)
  {
    //light, based on schedule
    if (time_seconds_schedule_light_off > time_seconds_schedule_light_on)
    {
      if (time_seconds_now_g >= time_seconds_schedule_light_on && time_seconds_now_g <= time_seconds_schedule_light_off)
      {
        relay_light_on(); // on
      }
      else
      {
        relay_light_off(); // off
      }
    }
    else
    {
      if (time_seconds_now_g < time_seconds_schedule_light_on && time_seconds_now_g > time_seconds_schedule_light_off)
      {
        relay_light_off(); // off
      }
      else
      {
        relay_light_on(); // on
      }
    }
  }
  else
  {
    //light
    if (flag_sw_on_light == true)
    {
      relay_light_on();
    }
    else
    {
      relay_light_off();
    }
  }
}

void buzzer()
{
  if (digitalRead(PIN_BUZZER) == HIGH)
  {
    if (millis() - timer_buzzer >= 10)
    {
      digitalWrite(PIN_BUZZER, LOW);
    }
  }
  else
  {
    if (flag_buzzer_btn)
    {
      flag_buzzer_btn = false;
      timer_buzzer = millis();
      digitalWrite(PIN_BUZZER, HIGH);
    }
  }
}

void serial_rx()
{
  char serbuf[20];
  byte inx;
  unsigned long t;
  char c;

  //format TYY/MM/DD hh:mm:ss
  if(Serial.available())
  {
    t=millis();
    memset(serbuf,0,20);
    inx=0;
    while(millis() - t < 50)
    {
       if(Serial.available()) 
       {
        c = Serial.read();
        if(c == 'T' || c == 't')
        {
          memset(serbuf,0,20);
          inx=0;
          serbuf[0] = c;  
          inx = 1;
        }
        else if(c == '\r' || c == '\n' )
        {
            break;
        }
        else if(inx>=18)
        {
            break;
        }
        else
        {
            serbuf[inx++] = c;  
        }
       }
    }

    while(Serial.available()) Serial.read();
    
    Serial.print(F("[RX]"));
    Serial.println(serbuf);

    //format TYY/MM/DD hh:mm:ss
    if((serbuf[1] >= '0' && serbuf[1] <= '9') && 
       (serbuf[2] >= '0' && serbuf[2] <= '9') && 
       (serbuf[4] >= '0' && serbuf[4] <= '9') && 
       (serbuf[5] >= '0' && serbuf[5] <= '9') && 
       (serbuf[7] >= '0' && serbuf[7] <= '9') && 
       (serbuf[8] >= '0' && serbuf[8] <= '9') &&
       (serbuf[10] >= '0' && serbuf[10] <= '9') && 
       (serbuf[11] >= '0' && serbuf[11] <= '9') && 
       (serbuf[13] >= '0' && serbuf[13] <= '9') && 
       (serbuf[14] >= '0' && serbuf[14] <= '9') && 
       (serbuf[16] >= '0' && serbuf[16] <= '9') && 
       (serbuf[17] >= '0' && serbuf[17] <= '9') &&         
       (serbuf[3] == '/' && serbuf[6] == '/'  && serbuf[9] == ' '  && serbuf[12] == ':'  && serbuf[15] == ':' ))
     {
       byte yy,mo,dy,hh,mm,ss;
       yy = ((serbuf[1]-'0')*10) + (serbuf[2]-'0');
       mo = ((serbuf[4]-'0')*10) + (serbuf[5]-'0');
       dy = ((serbuf[7]-'0')*10) + (serbuf[8]-'0');
       hh = ((serbuf[10]-'0')*10) + (serbuf[11]-'0');
       mm = ((serbuf[13]-'0')*10) + (serbuf[14]-'0');
       ss = ((serbuf[16]-'0')*10) + (serbuf[17]-'0');

       clock.setDateTime(yy+2000, mo, dy, hh, mm, ss);
       Serial.println(F("Success.Date and Time Set"));
     }  
     else
     {
      Serial.println(F("Error. Format must be: TYY/MM/DD hh:mm:ss. Ex: T24/12/31 13:45:30"));
     }
  }
}

// Real-time sensor updates to Blynk
void updateSensorsToBlynk()
{
  if (millis() - t_sensor_update >= TIME_UPDATE_SENSORS)
  {
    t_sensor_update = millis();
    
    // Update all sensor values
    Blynk.virtualWrite(VPORT_SENSOR_PH, ((float)water_ph_x10_g/10));
    Blynk.virtualWrite(VPORT_SENSOR_TDS, (int)(water_tds_x10_g/10));
    Blynk.virtualWrite(VPORT_SENSOR_TEMP, ((float)air_temperaturex10_g/10));
    Blynk.virtualWrite(VPORT_SENSOR_HUMIDITY, (int)(air_humidity_g));
    Blynk.virtualWrite(VPORT_WATER_LEVEL_PERCENT, (int)(water_level_percent_g));
  }
}

//Blynk refresh button (kept for compatibility but not needed with real-time updates)
BLYNK_WRITE(VPORT_BUTTON_SENSOR_REFRESH) {
  unsigned int dat = param.asInt();

  flag_buzzer_btn = true;

  if (dat) {
    // Force immediate update
    Blynk.virtualWrite(VPORT_SENSOR_PH, ((float)water_ph_x10_g/10));
    Blynk.virtualWrite(VPORT_SENSOR_TDS, (int)(water_tds_x10_g/10));
    Blynk.virtualWrite(VPORT_SENSOR_TEMP, ((float)air_temperaturex10_g/10));
    Blynk.virtualWrite(VPORT_SENSOR_HUMIDITY, (int)(air_humidity_g));
    Blynk.virtualWrite(VPORT_WATER_LEVEL_PERCENT, (int)(water_level_percent_g));
  }
}

//blynk manual switch
BLYNK_WRITE(VPORT_BUTTON_RELAY_MANUAL) {
  unsigned int dat = param.asInt();

  if (dat) {
    //manual
    flag_sw_manual_mode = true;
    flag_sw_manual_pH = true;
    flag_sw_manual_pest = true;
    flag_sw_manual_mist = true;
    flag_sw_manual_fan = true;
    flag_sw_manual_light = true;
  }
  else
  {
    //auto
    flag_sw_manual_mode = false;
    flag_sw_manual_pH = false;
    flag_sw_manual_pest = false;
    flag_sw_manual_mist = false;
    flag_sw_manual_fan = false;
    flag_sw_manual_light = false;
  }

  flag_buzzer_btn = true;

  //turn off switch
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_UP, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_FAN, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_PESTICIDE, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_DOWN, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_LIGHT, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_MISTER, 0);

  //turn off relays
  flag_sw_on_ph_up = false;
  flag_sw_on_ph_down = false;
  flag_sw_on_pest = false;
  flag_sw_on_mist = false;
  flag_sw_on_fan = false;
  flag_sw_on_light = false;
}

//Blynk switches for relays - Only work in manual mode
BLYNK_WRITE(VPORT_BUTTON_RELAY_PH_UP) {
  unsigned int dat = param.asInt();

  if (!flag_sw_manual_mode) {
    // In auto mode, don't allow manual control
    Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_UP, 0);
    return;
  }

  flag_buzzer_btn = true;

  if (dat) {
    // Turn off pH down if pH up is being turned on
    if (flag_sw_on_ph_down) {
      flag_sw_on_ph_down = false;
      Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_DOWN, 0);
    }
    flag_sw_on_ph_up = true;
  }
  else
  {
    flag_sw_on_ph_up = false;
  }
}

BLYNK_WRITE(VPORT_BUTTON_RELAY_FAN) {
  unsigned int dat = param.asInt();
  
  if (!flag_sw_manual_mode) {
    // In auto mode, don't allow manual control
    Blynk.virtualWrite(VPORT_BUTTON_RELAY_FAN, 0);
    return;
  }
  
  flag_buzzer_btn = true;

  if (dat) {
    flag_sw_on_fan = true;
  }
  else
  {
    flag_sw_on_fan = false;
  }
}

BLYNK_WRITE(VPORT_BUTTON_RELAY_PESTICIDE) {
  unsigned int dat = param.asInt();
  
  if (!flag_sw_manual_mode) {
    // In auto mode, don't allow manual control
    Blynk.virtualWrite(VPORT_BUTTON_RELAY_PESTICIDE, 0);
    return;
  }
  
  flag_buzzer_btn = true;

  if (dat) {
    flag_sw_on_pest = true;
  }
  else
  {
    flag_sw_on_pest = false;
  }
}

BLYNK_WRITE(VPORT_BUTTON_RELAY_PH_DOWN) {
  unsigned int dat = param.asInt();
  
  if (!flag_sw_manual_mode) {
    // In auto mode, don't allow manual control
    Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_DOWN, 0);
    return;
  }
  
  flag_buzzer_btn = true;

  if (dat) {
    // Turn off pH up if pH down is being turned on
    if (flag_sw_on_ph_up) {
      flag_sw_on_ph_up = false;
      Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_UP, 0);
    }
    flag_sw_on_ph_down = true;
  }
  else
  {
    flag_sw_on_ph_down = false;
  }
}

BLYNK_WRITE(VPORT_BUTTON_RELAY_LIGHT) {
  unsigned int dat = param.asInt();
  
  if (!flag_sw_manual_mode) {
    // In auto mode, don't allow manual control
    Blynk.virtualWrite(VPORT_BUTTON_RELAY_LIGHT, 0);
    return;
  }
  
  flag_buzzer_btn = true;

  if (dat) {
    flag_sw_on_light = true;
  }
  else
  {
    flag_sw_on_light = false;
  }
}

BLYNK_WRITE(VPORT_BUTTON_RELAY_MISTER) {
  unsigned int dat = param.asInt();
  
  if (!flag_sw_manual_mode) {
    // In auto mode, don't allow manual control
    Blynk.virtualWrite(VPORT_BUTTON_RELAY_MISTER, 0);
    return;
  }
  
  flag_buzzer_btn = true;

  if (dat) {
    flag_sw_on_mist = true;
  }
  else
  {
    flag_sw_on_mist = false;
  }
}

//blynk mister schedule update
BLYNK_WRITE(VPORT_TIME_MISTER_SCHEDULE) {

  start_time_in_seconds = param[0].asLong();
  stop_time_in_seconds = param[1].asLong();
  
  flag_buzzer_btn = true;

  //save valid time to eeprom
  eewrite_num_ulong(EEADD_SCHEDULE_MIST_ON,start_time_in_seconds);
  eewrite_num_ulong(EEADD_SCHEDULE_MIST_OFF,stop_time_in_seconds);

  //load time
  time_seconds_schedule_mist_on = eeload_num_ulong(EEADD_SCHEDULE_MIST_ON);
  time_seconds_schedule_mist_off = eeload_num_ulong(EEADD_SCHEDULE_MIST_OFF);

  lcd_schedule_mist_on_hr = time_seconds_schedule_mist_on / 3600;
  lcd_schedule_mist_on_mm = (time_seconds_schedule_mist_on % 3600) / 60;
  lcd_schedule_mist_on_ss = time_seconds_schedule_mist_on % 60;

  lcd_schedule_mist_off_hr = time_seconds_schedule_mist_off / 3600;
  lcd_schedule_mist_off_mm = (time_seconds_schedule_mist_off % 3600) / 60;
  lcd_schedule_mist_off_ss = time_seconds_schedule_mist_off % 60;
}

//update blynk when fungal infection or water level changes
void blynk_fungal_infection_water_level()
{
  static String past_crop_status = "";
  static byte past_water_level_percent = 255;
  
  // Update crop status
  if (past_crop_status != current_crop_status_g)
  {
    Blynk.virtualWrite(VPORT_ALARM_FUNGAL_INFECTION, current_crop_status_g);
    past_crop_status = current_crop_status_g;
  }

  // Update water level percentage
  if (past_water_level_percent != water_level_percent_g)
  {
    Blynk.virtualWrite(VPORT_WATER_LEVEL_PERCENT, water_level_percent_g);
    
    // Update color based on percentage
    if (water_level_percent_g <= 33)
    {
      Blynk.setProperty(VPORT_ALARM_WATER_LEVEL, "color", BLYNK_RED);
    }
    else if (water_level_percent_g <= 66)
    {
      Blynk.setProperty(VPORT_ALARM_WATER_LEVEL, "color", BLYNK_YELLOW);
    }
    else
    {
      Blynk.setProperty(VPORT_ALARM_WATER_LEVEL, "color", BLYNK_GREEN);
    }
    
    // Update the alarm value with percentage
    Blynk.virtualWrite(VPORT_ALARM_WATER_LEVEL, water_level_percent_g);
    past_water_level_percent = water_level_percent_g;
  }
}

void CheckConnection() {  // check every 11s if connected to Blynk server
  if (millis() - t_blynk_connection < TIME_CHECK_BLYNK_CONNECTION) return;

  if (!Blynk.connected()) {
    digitalWrite(PIN_BUZZER, LOW);

    lcd.setCursor(0, 0);
    lcd.print(F("                    "));
    lcd.setCursor(0, 1);
    lcd.print(F("      BUSY...       "));
    lcd.setCursor(0, 2);
    lcd.print(F(" RECONNECTING BLYNK "));
    lcd.setCursor(0, 3);
    lcd.print(F("                    "));

    relay_fan_off();
    relay_light_off();

    //ramp down high power drivers before reconnecting blynk
    while(high_power_driver_current_pwm_mister || high_power_driver_current_pwm_pest ||
      l298n_driver_current_pwm_ph_up || l298n_driver_current_pwm_ph_down)
    {
      if (high_power_driver_current_pwm_mister > 0)
      {
        high_power_driver_current_pwm_mister--;
      }
      if (high_power_driver_current_pwm_pest > 0)
      {
        high_power_driver_current_pwm_pest--;
      }
      if (l298n_driver_current_pwm_ph_up > 0)
      {
        l298n_driver_current_pwm_ph_up--;
      }
      if (l298n_driver_current_pwm_ph_down > 0)
      {
        l298n_driver_current_pwm_ph_down--;
      }

      analogWrite(PIN_HIGHPOWERDRIVER_MISTER_R_PWM, 0);
      analogWrite(PIN_HIGHPOWERDRIVER_MISTER_L_PWM, high_power_driver_current_pwm_mister);
      analogWrite(PIN_HIGHPOWERDRIVER_PEST_R_PWM, 0);
      analogWrite(PIN_HIGHPOWERDRIVER_PEST_L_PWM, high_power_driver_current_pwm_pest);

      analogWrite(PIN_L298N_ENA,l298n_driver_current_pwm_ph_up);
      analogWrite(PIN_L298N_ENB,l298n_driver_current_pwm_ph_down);

      delay(1);
    }
    digitalWrite(PIN_L298N_IN1, LOW);
    digitalWrite(PIN_L298N_IN2, LOW);
    digitalWrite(PIN_L298N_IN3, LOW);
    digitalWrite(PIN_L298N_IN4, LOW);

    Blynk.connect();  // try to connect to server with default timeout

    if (Blynk.connected()) {
      lcd.setCursor(0, 3);
      lcd.print(F("       SUCCESS      "));
      flag_led_blynk = false;
    } else {
      lcd.setCursor(0, 3);
      lcd.print(F("       FAILED       "));
      flag_led_blynk = true;
    }
  }

  t_blynk_connection = millis();
}

void setup() {

  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println(F("=== GAWIS System Starting ==="));
  Serial.print(F("WiFi SSID: "));
  Serial.println(ssid);
  Serial.println(F("Checking ESP8266..."));
  // put your setup code here, to run once:

  //2CH-relay for fan and lights
  pinMode(PIN_2CHRELAY_IN1_FAN, OUTPUT);
  relay_fan_off();
  pinMode(PIN_2CHRELAY_IN2_LIGHT, OUTPUT);
  relay_light_off();
  
  pinMode(PIN_HIGHPOWERDRIVER_MISTER_R_EN, OUTPUT);
  pinMode(PIN_HIGHPOWERDRIVER_MISTER_L_EN, OUTPUT);
  pinMode(PIN_HIGHPOWERDRIVER_MISTER_R_PWM, OUTPUT);
  pinMode(PIN_HIGHPOWERDRIVER_MISTER_L_PWM, OUTPUT);

  pinMode(PIN_HIGHPOWERDRIVER_PEST_R_EN, OUTPUT);
  pinMode(PIN_HIGHPOWERDRIVER_PEST_L_EN, OUTPUT);
  pinMode(PIN_HIGHPOWERDRIVER_PEST_R_PWM, OUTPUT);
  pinMode(PIN_HIGHPOWERDRIVER_PEST_L_PWM, OUTPUT);

  //turn off high power drivers
  digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_R_EN, LOW);
  digitalWrite(PIN_HIGHPOWERDRIVER_MISTER_L_EN, LOW);
  analogWrite(PIN_HIGHPOWERDRIVER_MISTER_R_PWM, 0);
  analogWrite(PIN_HIGHPOWERDRIVER_MISTER_L_PWM, 0);
  digitalWrite(PIN_HIGHPOWERDRIVER_PEST_R_EN, LOW);
  digitalWrite(PIN_HIGHPOWERDRIVER_PEST_L_EN, LOW);
  analogWrite(PIN_HIGHPOWERDRIVER_PEST_R_PWM, 0);
  analogWrite(PIN_HIGHPOWERDRIVER_PEST_L_PWM, 0);

  // L298N module for pH up/down
  pinMode(PIN_L298N_ENA, OUTPUT);
  pinMode(PIN_L298N_IN1, OUTPUT);
  pinMode(PIN_L298N_IN2, OUTPUT);
  pinMode(PIN_L298N_ENB, OUTPUT);
  pinMode(PIN_L298N_IN3, OUTPUT);
  pinMode(PIN_L298N_IN4, OUTPUT);

  //turn off pumps
  analogWrite(PIN_L298N_ENA,0);
  digitalWrite(PIN_L298N_IN1, LOW);
  digitalWrite(PIN_L298N_IN2, LOW);
  analogWrite(PIN_L298N_ENB,0);
  digitalWrite(PIN_L298N_IN3, LOW);
  digitalWrite(PIN_L298N_IN4, LOW);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_WATER_LEVEL_HIGH, OUTPUT);
  pinMode(PIN_LED_WATER_LEVEL_MEDIUM, OUTPUT);
  pinMode(PIN_LED_WATER_LEVEL_LOW, OUTPUT);
  pinMode(PIN_LED_HB, OUTPUT);
  pinMode(PIN_LED_BLYNK_RECV, OUTPUT);
  
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);

  Serial.begin(115200);
  ESP32.begin(115200); //blynk
  ESPCAM.begin(115200); // ESP32_CAM

  clock.begin(); //initialize RTC

  // Manual (YYYY, MM, DD, HH, II, SS
  // clock.setDateTime(2024, 12, 2, 1, 18, 0); //keep this commented when not updating RTC date and time

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0,0); lcd.print(F("PLEASE WAIT..."));

  lcd.setCursor(0,1); lcd.print(F("CONNECTING TO"));
  lcd.setCursor(0,2); lcd.print(F("BLYNK..."));

  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);

  lcd.print(F("OK"));
  
  //TDS initialization
  gravityTds.setPin(PIN_TDS);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  //initialize DHT
  dht.begin();

  //pH sensor calculate slope and intercept
  ph_below7_slope = ((float)PH_ADC_READING_4 - (float)PH_ADC_READING_7) / (4.01 - 7.0);
  ph_below7_intercept = (float)PH_ADC_READING_4 - ((float)ph_below7_slope * 4.01);

  ph_above7_slope = ((float)PH_ADC_READING_9 - (float)PH_ADC_READING_7) / (9.18 - 7.0);
  ph_above7_intercept = (float)PH_ADC_READING_7 - (ph_above7_slope * 7.0);

  //read initial pH
  temp_float_var = 0;
  for (byte i = 0; i < PH_RAW_MAX; i++) {
    ph_raw_data[i] = analogReadx10(PIN_PH);
    temp_float_var += ph_raw_data[i];
    delay(10);
  }
  temp_float_var /= PH_RAW_MAX;

  if (temp_float_var > PH_ADC_READING_7)  // pH is expected to be 7 below
  {
    temp_float_var = (temp_float_var - ph_below7_intercept) / ph_below7_slope;
  } else  //ph is expected to be 7 and up
  {
    temp_float_var = (temp_float_var - ph_above7_intercept) / ph_above7_slope;
  }
  temp_float_var = ((temp_float_var * PH_CALIBRATION_MULTIPLIER) + PH_CALIBRATION_OFFSET) * 10;
  if (temp_float_var > 140) temp_float_var = 140;
  if (temp_float_var <= 0) temp_float_var = 0;
  water_ph_x10_g = temp_float_var;

  threshold_air_temeparture_x10 = AIR_TEMPERATURE_THRESHOLD * 10; 

  user_ph_normal_min_x10 = NORMAL_RANGE_PH_MIN * 10;
  user_ph_normal_max_x10 = NORMAL_RANGE_PH_MAX * 10;

  time_seconds_schedule_light_on =  ((unsigned long)TIME_SCHEDULE_LIGHT_ON_HR * 3600) + (TIME_SCHEDULE_LIGHT_ON_MM * 60) + TIME_SCHEDULE_LIGHT_ON_SS;
  time_seconds_schedule_light_off =  ((unsigned long)TIME_SCHEDULE_LIGHT_OFF_HR * 3600) + (TIME_SCHEDULE_LIGHT_OFF_MM * 60) + TIME_SCHEDULE_LIGHT_OFF_SS;

  time_seconds_schedule_mist_on = eeload_num_ulong(EEADD_SCHEDULE_MIST_ON);
  time_seconds_schedule_mist_off = eeload_num_ulong(EEADD_SCHEDULE_MIST_OFF);

  lcd_schedule_mist_on_hr = time_seconds_schedule_mist_on / 3600;
  lcd_schedule_mist_on_mm = (time_seconds_schedule_mist_on % 3600) / 60;
  lcd_schedule_mist_on_ss = time_seconds_schedule_mist_on % 60;

  lcd_schedule_mist_off_hr = time_seconds_schedule_mist_off / 3600;
  lcd_schedule_mist_off_mm = (time_seconds_schedule_mist_off % 3600) / 60;
  lcd_schedule_mist_off_ss = time_seconds_schedule_mist_off % 60;

  //get initial TDS
  gravityTds.setTemperature(TDS_WATER_TEMPERATURE);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate 
  temp_float_var = gravityTds.getTdsValue();  // then get the value

  water_tds_x10_g = temp_float_var * 10; //water TDS

  air_humidity_g = dht.readHumidity();
  air_temperaturex10_g = dht.readTemperature() * 10;

  water_level_g = WATER_LEVEL_HIGH;
  water_level_percent_g = 100;

  // Initialize data history
  for (int i = 0; i < DATA_HISTORY_SIZE; i++) {
    dataHistory[i].ph = 0;
    dataHistory[i].tds = 0;
    dataHistory[i].temperature = 0;
    dataHistory[i].humidity = 0;
    dataHistory[i].water_level_percent = 0;
    dataHistory[i].timestamp = 0;
  }

  //send init data to blynk
  //sensors
  Blynk.virtualWrite(VPORT_SENSOR_PH, ((float)water_ph_x10_g/10));
  Blynk.virtualWrite(VPORT_SENSOR_TDS, (int)(water_tds_x10_g/10));
  Blynk.virtualWrite(VPORT_SENSOR_TEMP, ((float)air_temperaturex10_g/10));
  Blynk.virtualWrite(VPORT_SENSOR_HUMIDITY, (int)(air_humidity_g));
  Blynk.virtualWrite(VPORT_WATER_LEVEL_PERCENT, (int)(water_level_percent_g));
  //alarm
  flag_past_esp32_cam_pest_detected_g = false;
  past_water_level_g = WATER_LEVEL_HIGH;
  current_crop_status_g = "NO CROP";
  Blynk.virtualWrite(VPORT_ALARM_FUNGAL_INFECTION, current_crop_status_g);
  Blynk.virtualWrite(VPORT_ALARM_WATER_LEVEL, water_level_percent_g);
  Blynk.setProperty(VPORT_ALARM_WATER_LEVEL, "color", BLYNK_GREEN);
  //switch for relay
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_MANUAL, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_UP, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_FAN, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_PESTICIDE, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_PH_DOWN, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_LIGHT, 0);
  Blynk.virtualWrite(VPORT_BUTTON_RELAY_MISTER, 0);

  //short buzzer to indicate setup is complete
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.println(F("start"));
  Serial.println(F("ESP32-CAM Disease Detection System Ready"));

  t_blynk_connection = millis();
  t_sensor_update = millis();
}

void loop() {

  if (Blynk.connected()) {
    flag_led_blynk = false;
    Blynk.run();
    updateSensorsToBlynk(); // Real-time sensor updates
  } else {
    flag_led_blynk = true;
    CheckConnection();
  }

  // put your main code here, to run repeatedly:
  ultrasonic_check(); // checks water level
  water_ph(); // get the water ph
  water_tds(); // get water TDS
  air_humidity_temperature(); // get air humidity and temeprature
  esp32_cam_check(); // check data from ESPCAM for disease detection
  check_date_time(); // updates date time variables

  control_pH(); // control motor for pH up/down
  control_pest(); // control motor for pest
  control_mist(); // control motor for misting
  control_fan_light(); // relay for fan and light control

  disp_idle(); // display idle screen, displays time and sensor values
  led_contol(); // control leds

  blynk_fungal_infection_water_level(); // update blynk when fungal infection or water level changes

  buzzer(); // control buzzer behaviour
  high_power_driver_control(); // ramping motors routine
  l298n_driver_control(); // ramping motors routine

  saveDataToHistory(); // Save data to history for analytics
  sendHistoryToBlynk(); // ADD THIS LINE HERE - Send history to Blynk charts

  serial_rx(); //change date and time via serial
}