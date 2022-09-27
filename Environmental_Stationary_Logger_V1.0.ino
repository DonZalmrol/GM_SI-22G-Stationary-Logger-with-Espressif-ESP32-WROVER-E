// Include libraries
#include <Wire.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <ArduinoHttpClient.h>
#include <esp_task_wdt.h>
#include <ESPPerfectTime.h>
#include <movingAvg.h>
#include <bsec.h>
#include <driver/pcnt.h>
#include <esp_adc_cal.h>
#include <Tomoto_HM330X.h>
#include <Digital_Light_TSL2561.h>

// disabled on 2022-09-25
//#include <Arduino.h>
//#include <driver/adc.h>

// Include optional files
#include "arduino_secrets.h"

// Configure the BSEC library with information about the sensor : 3.3V, 3S OPmode, 4D age of the sensor
const uint8_t bsec_config_iaq[] =
{ 
  #include "bsec_iaq.h"
};

// Defines
// Set 75 seconds watchdog timer
#define WDT_TIMEOUT 75

// Tube dead time in seconds
// SI-22G = 0.000xxx seconds <- still searching/ researching actual data
// SBM-20 = 0.000190 seconds
#define tubeDeadTime 0.000190

// Tube conversion value
// SI-22G = 0.001714
// SBM-20 = 0.006315
#define tubeConversionFactor 0.001714

// 360 minutes (6 hours) = saving 4 times a day
//#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000) 
unsigned long STATE_SAVE_PERIOD = (1 * 60 * 1000);

// Helper functions declarations
static void WiFiSetup(void);
float outputSieverts(float);
float displayTubeVoltage(float);
static void uploadTaskFunction(void);
static void connecToRadMonLogger(void);
static void connecToURadMonLogger(void);
static void scanI2C(void);
static void loadState(void);
static void updateStateFunction(void);

// Variables
int increaseSecCount = 1, var_iaq = 0.0, var_iaqAccuracy = 0, var_pm01 = 0, var_pm25 = 0, var_pm10 = 0;
float var_temperature = 0.0, var_humidity = 0.0, var_pressure = 0.0, var_voc = 0.0, var_co2 = 0.0, var_hcho = 0.0, tubeVoltage = 0.0;

//unsigned long cps_1 = 0, cps_2 = 0;
int16_t cps_1 = 0, cps_2 = 0;
unsigned long actual_cps_1 = 0, actual_cps_2 = 0;
unsigned long cpm = 0, cpm_temp = 0;
unsigned long sensorMovingAvg = 0;
unsigned long currentMillis = 0, previousMillis_1 = 0, previousMillis_2 = 0, previousMillis_3 = 0;
unsigned long luminosity = 0;

bool updateEEPROM = false;

// Create and set sensor to 120 data points for moving average (2x 60 second data entry points)
movingAvg cps_sensor(120);

boolean eventTriggerd = false;

// Create time_t object
time_t epoch = 0;

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

// Grove Laser PM2.5 Sensor (HM3301)
Tomoto_HM330X pm_sensor;
// Tomoto_HM330X sensor(Wire1); // to use the alternative wire

// ESP32 Pulse Counter
#define PCNT_UNIT_01 PCNT_UNIT_0   // Select the Pulse Counter Unit 0
#define PCNT_UNIT_02 PCNT_UNIT_1   // Select the Pulse Counter Unit 1

#define PCNT_H_LIM_VAL 125          // Set the high limit count to trigger the interrupt
#define PCNT_L_LIM_VAL 0            // Set the low limit count to trigger the interrupt
#define PCNT_FILTER_VAL 128         // Set the filter value in nanoseconds, 1 second = 1 000 000 000 nanoseconds

#define PCNT_INPUT_SIG_IO_01 13     // Pulse Input selected as GPIO14
#define PCNT_INPUT_SIG_IO_02 14     // Pulse Input selected as GPIO12

pcnt_isr_handle_t user_isr_handle = NULL; // User ISR handler for Interrupt

// WiFi credentials
const char *my_ssid = SECRET_SSID;
const char *my_password = SECRET_PASS;
const char *my_hostname = SECRET_HOST;
const long timeoutTime = 5000;

// RADMON credentials
const char *UserName = SECRET_USER_NAME;
const char *DataSendingPassWord = SECRET_USER_PASS_01;

// URadMonitoring credentials
const char *USER_ID = SECRET_USER_ID;
const char *USER_KEY = SECRET_USER_KEY;
const char *DEVICE_ID = SECRET_DEVICE_ID;

// Set NTP server for Europe
const char *ntpServer = "europe.pool.ntp.org";

// Create a task handle
TaskHandle_t uploadTask;
TaskHandle_t updateState;

//--------------------//
//--- START SETUP ---//
//------------------//
void setup()
{
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
  
  // Initialize UART on 115200 baud rate for ethernet shield
  Serial.begin(115200);

  Serial.println(F("Initializing Serial."));
  while (!Serial)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println(F("Serial initialized."));
  
  // Initialize watchdog
  Serial.println(F("Initializing WatchDogTimer (WDT)"));
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  // Check if I2C bus is enabled
  if(!Wire.begin())
  {
    Serial.println(F("I2C bus not initialized!"));
  }
  
  else
  {
    Serial.println(F("I2C bus is initialized."));
  
    // Connect to the WiFi
    WiFiSetup();

    // Test scan I2C address bus for devices
    scanI2C();
  
    // Initialize variables
    cps_1 = 0, cps_2 = 0;
    actual_cps_1 = 0, actual_cps_2 = 0;
    sensorMovingAvg = 0;
    increaseSecCount = 1;
  
    // Initialize sensor
    cps_sensor.begin();
  
    // Initialize Arduino pins
    pinMode(33, INPUT_PULLUP);  // Set pin34 (GPIO33) input for ADC1_CHANNEL_5
    pinMode(34, INPUT_PULLUP);  // Set pin34 (GPIO03) input for HCHO sensor A

    pinMode(13, INPUT_PULLUP);  // Set pin13 (GPIO14) input for capturing GM Tube 01 events (pulses)
    pinMode(14, INPUT_PULLUP);  // Set pin14 (GPIO12) input for  capturing GM Tube 02 events (pulses)

    //pinMode(26, OUTPUT);      // Set pin04 (GPIO26) as output for LED
    //pinMode(25, OUTPUT);      // Set pin25 (GPIO25) as output for LED upload = OK

    // Set the iaqSensor(BME680) to initialize on the I2C bus
    iaqSensor.begin(0x76, Wire);
    
    if (iaqSensor.bme680Status != BME680_OK)
    {
      Serial.println(F("No valid BME680 found!"));
      Serial.println("BME680 error code : " + String(iaqSensor.bme680Status));
    }
    else
    {
      Serial.println(F("Valid BME680 found."));
      Serial.println("BSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));

      iaqSensor.setConfig(bsec_config_iaq);
      loadState();

      bsec_virtual_sensor_t sensorList[10] = 
      {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
      };

      iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    }

    if(!pm_sensor.begin())
    {
      Serial.println(F("No valid HM330X found!"));
      while (1);
    }

    else
    {
      Serial.println(F("Valid HM330X found."));
    }

    TSL2561.init();
    Serial.println(F("Valid TSL2561 found."));

    // Testing of Pulse Counter (PCNT)
    Init_PulseCounter_01();
    Init_PulseCounter_02();
    
    // Configure SNTP Client in EU/Brussels time
    pftime::configTzTime(PSTR("CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"), ntpServer);
  
    Serial.println(F("--- Start program ---"));
    Serial.println(F("\n"));
  }
}
//------------------//
//--- END SETUP ---//
//----------------//

//-------------------//
//--- START LOOP ---//
//-----------------//
void loop()
{
  // If reached 01 seconds with a total of 60 seconds
  if (((millis() - previousMillis_1) >= 1000UL) && (increaseSecCount <= 60))
  {
    if(eventTriggerd)
    {
      Serial.println(F("--- ISR ---"));
      Serial.println("| Counter of tube exceeded the value of " + String(PCNT_H_LIM_VAL));
      Serial.println(F("--- ISR ---"));

      eventTriggerd = false;
    }

    pcnt_get_counter_value(PCNT_UNIT_01, &cps_1);
    pcnt_get_counter_value(PCNT_UNIT_02, &cps_2);

    // Recalute with tube dead time accounted for actual CPS of both tubes
    actual_cps_1 = (cps_1 / (1 - (cps_1 * tubeDeadTime)));
    actual_cps_2 = (cps_2 / (1 - (cps_2 * tubeDeadTime)));

    // Add a new datapoint entry for both tubes to the moving average array
    sensorMovingAvg = cps_sensor.reading(actual_cps_1);
    sensorMovingAvg = cps_sensor.reading(actual_cps_2);
    
    //   // Uncomment below for testing 1 sec progress
    //   displayTubeVoltage();
    //   calculateHCHO();

    Serial.println(F("--- 01 sec ---"));
    Serial.println("| Tube 1 " + String(increaseSecCount) + " sec current count (cps_1) = " + String(actual_cps_1));
    Serial.println("| Tube 2 " + String(increaseSecCount) + " sec current count (cps_2) = " + String(actual_cps_2));
    Serial.println("| Tubes current total mavg " + String(increaseSecCount) + " sec current count = " + String(sensorMovingAvg));
    Serial.println("| Voltage tubes = " + String(displayTubeVoltage(), 3));
    Serial.println(F("--- 01 sec ---"));

    // Display 1 second tick for a total of 60
    Serial.println(String(increaseSecCount) + " of 60");

    if (iaqSensor.run())
    {
      // variable = value, 3 decimal places
      var_temperature = iaqSensor.temperature, 3;
      var_pressure = iaqSensor.pressure, 3;
      var_humidity = iaqSensor.humidity, 3;
      var_voc = iaqSensor.gasResistance, 3;
      var_co2 = iaqSensor.co2Equivalent, 3;
      
      var_iaq = iaqSensor.iaq;
      var_iaqAccuracy = iaqSensor.iaqAccuracy;

      // Uncomment below for IAQ testing
      //Serial.println("IAQ level = " + String(var_iaq));
      // 0 = Stabilizing, 1 = Uncertain, 2 = Calibrating, 3 = Calibrated
      //Serial.println("IAQ Accuracy = " + String(var_iaqAccuracy));

      // Initialize Task
      xTaskCreatePinnedToCore
      (
        updateStateFunction, /* Function to implement the task */
        "updateState", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &updateState,  /* Task handle. */
        1 /* Core where the task should run */
      );
    }

    previousMillis_1 = millis();
    increaseSecCount++;
  }

  // If 31 seconds have been reached calculate HM330X values
  // BSEC needs 30 seconds to warm up
	if (((millis() - previousMillis_2) >= 31000UL) && (increaseSecCount == 31))
	{
    if (!pm_sensor.readSensor())
		{
			Serial.println(F("Failed to read values from HM330X"));
		}

    else
		{
      var_pm01 = pm_sensor.std.getPM1();
      var_pm25 = pm_sensor.std.getPM2_5();
      var_pm10 = pm_sensor.std.getPM10();

      // // Uncomment for testing
      // Serial.println("Sensor number = " + String(pm_sensor.getSensorNumber()));
      // Serial.println(F("Concentration based on CF=1 standard particlate matter (ug/m^3) --"));
      // Serial.println("PM1.0 = " + String(var_pm01));
      // Serial.println("PM2.5 = " + String(var_pm25));
      // Serial.println("PM10 = " + String(var_pm10));
    }
  }
  
  // If 61 seconds have been reached do uploads and reset vars
  if (((millis() - previousMillis_3) >= 61000UL) && (increaseSecCount >= 61))
  {
    // Extra WIFI connection check
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(F("Reconnecting to WiFi..."));
        WiFi.disconnect();
        WiFi.reconnect();
    }

    else
    {
      // Fetch values
      cpm = cps_sensor.getAvg();
      cpm_temp = cpm;
      luminosity = TSL2561.readVisibleLux();
      var_hcho = calculateHCHO();
      //battery = (analogRead(37)/1000), 3; // Read pin 37 CapVP
      tubeVoltage = displayTubeVoltage();

      // Get current time as UNIX time
      epoch = pftime::time(nullptr);

      // Print new line
      Serial.println(F("\n"));

      Serial.println(F("--- 61 sec ---"));
      Serial.println("| Tube 1 cpm = " + String(actual_cps_1));
      Serial.println("| Tube 2 cpm = " + String(actual_cps_2));
      Serial.println("| cpm_temp = " + String(cpm_temp));
      Serial.println("| var_temperature = " + String(var_temperature) + " C");
      Serial.println("| var_pressure = " + String(var_pressure) + " hPa");
      Serial.println("| var_humidity = " + String(var_humidity) + " %");
      Serial.println("| var_voc = " + String(var_voc) + " Ohm");
      Serial.println("| var_co2 = " + String(var_co2) + " ppm");
      Serial.println("| IAQ level = " + String(var_iaq));
      Serial.println("| IAQ Accuracy = " + String(var_iaqAccuracy));   // 0 = Stabilizing, 1 = Uncertain, 2 = Calibrating, 3 = Calibrated
      Serial.println("| luminosity = " + String(luminosity));
      Serial.println("| var_hcho = " + String(var_hcho));
      Serial.println("| var_pm01 = " + String(var_pm01));
      Serial.println("| var_pm25 = " + String(var_pm25));
      Serial.println("| var_pm10 = " + String(var_pm10));
      Serial.println("| tubeVoltage = " + String(tubeVoltage));
      Serial.println(F("--- 61 sec ---"));
      Serial.println(F("| Commence uploading"));
      Serial.println(F("| Resetting vars"));
      Serial.println(F("--- 61 sec ---"));

      // Print new line
      Serial.println(F("\n"));

      // Initialize Task
      xTaskCreatePinnedToCore
      (
      uploadTaskFunction, /* Function to implement the task */
      "uploadTask", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &uploadTask,  /* Task handle. */
      1 /* Core where the task should run */
      );

      // Reset variables
      cps_1 = 0, cps_2 = 0, actual_cps_1 = 0, actual_cps_2 = 0, cpm = 0, sensorMovingAvg = 0;
      increaseSecCount = 1;
      
      Clean_Counters();
      cps_sensor.reset();

      // Reset WatchDogTimer
      esp_task_wdt_reset();

      previousMillis_1 = millis();
      previousMillis_2 = millis();
      previousMillis_3 = millis();

      eventTriggerd = false;
    }
  }
}
//-----------------//
//--- END LOOP ---//
//---------------//

//------------------------//
//--- START functions ---//
//----------------------//

// Setup WiFi connection
static void WiFiSetup(void)
{
  Serial.println(F("\n"));
  Serial.println("Connecting to = " + String(my_ssid));
  
  WiFi.begin(my_ssid, my_password);
  WiFi.setHostname(my_hostname);
  
  // Set up Wifi connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  // Wifi is connected
  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F("\n"));
    Serial.println(F("WiFi connected"));
    Serial.println("IP address = " +  String(WiFi.localIP().toString()));
    Serial.println("Gateway = " + String(WiFi.gatewayIP().toString()));
    Serial.println("SubnetMask = " + String(WiFi.subnetMask().toString()));
    Serial.println("DNS 1 = " + String(WiFi.dnsIP(0).toString()));
    Serial.println("DNS 2 = " + String(WiFi.dnsIP(1).toString()));
    Serial.println("MAC = " + String(WiFi.macAddress()));
    Serial.println("RSSI = " + String(WiFi.RSSI()) + " dB");
    Serial.println(F("\n"));
  }

  // Wifi is not connected
  else
  {
      Serial.println(F("WiFi NOT connected"));
      Serial.println(F("\n"));
      
      // Restart ESP32
      ESP.restart();
  }
}

// Convert counts to uSV
float outputSieverts(float counts) 
{
  float uSV = counts * tubeConversionFactor;
  return uSV;
}

// Measure Tube voltage through A0
float displayTubeVoltage(void)
{
  float adcInput = 0.0, lowVoltage = 0.0, voltage_offset = 0.0, highVoltage = 0.0;

  adc1_config_width(ADC_WIDTH_12Bit);
  //adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_11db);
  adcInput = adc1_get_raw(ADC1_CHANNEL_5);

  // ESP32 pin 33 measures from 0-3.3V and has a width of 0-4096
  // Use 3.4 for as correcting value
  lowVoltage = ((adcInput * 3.4 ) / 4096.0);
  
  /*
    If you want to monitor high voltage on tubes with microcontroller ADC you can use A0 module output.
    ~190-195 conversion factor according to Alex - RH Electronics 2021-03-15
    ~184.097 calcultated on 2022-09-20 for ESP32 Wrover-E after research, see below

    Calculate your value with the "ADC Linearity calculator", yellow is your measured ADC value.
    Measure the voltage between your tubes and multiple x2, measure voltage of pin A0
    
    Measured:
     - ADC = 2863
     - Voltage of tube = 2x 218.8V = 437.6V
     - Voltage of A0 = 2.37V

    Calculated:
     - Voltage should be 2377mV
     - Conversion factor calculation = 437.6V / 2.377 = 184.097
     - Confirm with calculated high voltage table     
  */
  voltage_offset = 184.097;
  highVoltage = (lowVoltage * voltage_offset);

  // Uncomment for testing
  //Serial.println("| adcInput = " + String(adcInput, 3));
  //Serial.println("| A0 LV = " + String(lowVoltage, 3));
  //Serial.println("| Tubes HV = " + String(highVoltage, 3));

  return highVoltage;
}

float calculateHCHO(void)
{
  // Source https://wiki.seeedstudio.com/Grove-HCHO_Sensor/
  float Rs = 0.0, ppm = 0.0;

  // Read the input on the analog pin13 (GPIO13)
  int sensorValue = analogRead(34);
  Rs = (1023.0 / sensorValue ) - 1;

  // Calculated R0 = 10.37R on 2022-09-20
  float R0 = 10.37;

  ppm=pow(10.0,((log10(Rs/R0)-0.0827)/(-0.4807)));

  // Uncomment for testing
  //Serial.println("sensorValue = " + String(sensorValue));
  //Serial.println("| R0 = " + String(R0));
  //Serial.println("| RS = " + String(Rs));
  //Serial.println("| ppm = " + String(ppm));

  return ppm;
}

// Contains the logic to start, manage and stop the uploads
static void uploadTaskFunction(void * parameter)
{
  esp_task_wdt_add(NULL);

  // Connect to RadMon.org
  connecToRadMonLogger();
  
  // Connect to Uradmonitor
  connecToURadMonLogger();

  // Reset variables
  epoch = 0;
  cpm_temp = 0;
  var_temperature = 0.0, var_humidity = 0.0, var_pressure = 0.0, luminosity = 0, var_voc = 0.0, var_co2 = 0.0, var_hcho = 0.0, tubeVoltage = 0.0;

  esp_task_wdt_delete(NULL);

  vTaskDelete(uploadTask);
}

// Upload data to the RadMon.org server
static void connecToRadMonLogger(void)
{
  esp_task_wdt_add(NULL);
  
  WiFiClient wifi;
  HttpClient client = HttpClient(wifi, "radmon.org", 80);

  Serial.println(F("Connection to radmon monitoring platform succeeded!"));

  // Concat data for POST
  // API URL
  String ptr = "/radmon.php?function=submit";
  ptr += "&user=";
  ptr += UserName;
  ptr += "&password=";
  ptr += DataSendingPassWord;
  ptr += "&value=";
  ptr += cpm_temp;
  ptr += "&unit=CPM";

  // Test output
  Serial.println("Uploaded CPM = " + String(cpm_temp));
  Serial.println("created PTR = " + ptr);

  client.beginRequest();
  client.post(ptr);
  client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client.sendHeader("Content-Length", ptr.length());
  client.beginBody();
  client.print("");
  client.endRequest();

  // Keep statuscode and response in comments during production, used for testing!
  // read the status code and body of the response
  int statusCode = client.responseStatusCode();
  Serial.println("Status code = " + String(statusCode));

  // Give the client some time to stop
  yield();

  esp_task_wdt_delete(NULL);
  
  Serial.println(F("Connection to radmon monitoring platform Disconnected."));
  Serial.println(F("\n"));
}

// Upload data to the uradmonitor.com server
static void connecToURadMonLogger(void)
{
  esp_task_wdt_add(NULL);
  
  WiFiClient wifi;
  HttpClient client = HttpClient(wifi, "data.uradmonitor.com", 80);

  Serial.println(F("Connection to uradmonitoring platform succeeded!"));
  
  /** Concat data for POST
  * API URL based on the expProtocol.h
  * https://github.com/radhoo/uradmonitor_kit1/blob/master/code/misc/expProtocol.h
  */
  String ptr = "/api/v1/upload/exp";
  ptr += "/01/";                // compulsory: local time in seconds
  ptr += epoch;                 // time epoch value
  ptr += "/02/";                // 02 = optional: temperature in degrees celsius
  ptr += var_temperature;       // temperature value
  ptr += "/03/";                // 03 = optional: barometric pressure in pascals
  ptr += var_pressure;          // pressure value 
  ptr += "/04/";                // 04 = optional: humidity as relative humidity in percentage %
  ptr += var_humidity;          // humidity value
  ptr += "/05/";                // 05 = optional: luminosity as relative luminosity in percentage ‰
  ptr += luminosity;            // luminosity value
  ptr += "/06/";                // 06 = optional: VOC (volatile organic compounds) in ohms
  ptr += var_voc;               // VOC value
  ptr += "/07/";                // 07 = optional: CO2 (carbon dioxide) in ppm
  ptr += var_co2;               // CO2 value
  ptr += "/08/";                // 08 = optional: formaldehyde in ppm
  ptr += var_hcho;              // HCHO value
  ptr += "/09/";                // 09 = optional: particulate matter in micro grams per cubic meter
  ptr += var_pm25;              // PM2.5 value
  ptr += "/10/";                // 10 = optional: 
  ptr += var_pm01;              // PM1.0 value
  ptr += "/11/";                // 11 = optional: 
  ptr += var_pm10;              // PM10 value
  //ptr += "/0A/";                // 0A = optional: device battery voltage in volts
  //ptr += battery;               // battery value
  ptr += "/0B/";                // 0B = optional: radiation measured on geiger tube in cpm
  ptr += cpm_temp;              // a-cpm value
  ptr += "/0C/";                // 0C = optional: high voltage geiger tube inverter voltage in volts
  ptr += tubeVoltage;           // tube voltage value
  //ptr += "/0D/";                // 0D = optional: high voltage geiger tube inverter duty in ‰
  //ptr += var_pwm;               // PWM value
  ptr += "/0E/106/0F/123";      // 0F : 123 = ver_sw : value | 0E : 106 = ver_hw : value
  ptr += "/10/6";               // 10 = Tube ID | 6 (0x6) = GEIGER_TUBE_SI22G
  
  // Test output
  Serial.println("Uploaded CPM = " + String(cpm_temp));
  Serial.println("created EXP code = " + ptr);

  client.beginRequest();
  client.post(ptr);
  client.sendHeader("X-User-id", USER_ID);
  client.sendHeader("X-User-hash", USER_KEY);
  client.sendHeader("X-Device-id", DEVICE_ID);
  client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client.sendHeader("Content-Length", ptr.length());
  client.beginBody();
  client.print("");
  client.endRequest();

  // Keep statuscode and response in comments during production, used for testing!
  // read the status code and body of the response
  int statusCode = client.responseStatusCode();
  Serial.println("Status code = " + String(statusCode));

  // Give the client some time to stop
  yield();

  esp_task_wdt_delete(NULL);
  
  Serial.println(F("Connection to uradmonitoring platform Disconnected."));
  Serial.println(F("\n"));
}

static void scanI2C(void)
{
  byte error, address;
  int nDevices;
  
  Serial.println("Scanning for I2C devices...");
  
  nDevices = 0;
  
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      
      if (address<16)
      {
        Serial.print("0");
      }
      
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      
      if (address<16)
      {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("Finished scanning for I2C devices\n");
  }
  delay(5000);          
}

// Load latest data from EEPROM
static void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
  {
    // Existing state in EEPROM
    Serial.println(F("Reading state from EEPROM"));

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      bsecState[i] = EEPROM.read(i + 1);
      // Uncomment below to see to-be EEPROM output
      //Serial.println(bsecState[i], HEX);
    }
    iaqSensor.setState(bsecState);
    iaqSensor.getState(bsecState);
  }
  
  else
  {
    // Erase the EEPROM with zeroes
    Serial.println(F("Erasing EEPROM"));

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
    {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
  }
}

// Save data to EEPROM
static void updateStateFunction(void * parameter)
{
  esp_task_wdt_add(NULL);
  
  currentMillis = millis();
  
  /* 
    Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 
    0 = Stabilizing
    1 = Uncertain
    2 = Calibrating
    3 = Calibrated
  */
  if (stateUpdateCounter == 0)
  {
    if (iaqSensor.iaqAccuracy >= 3)
    {
      updateEEPROM = true;
      stateUpdateCounter++;
    }
  }
  
  else
  {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    //if ((stateUpdateCounter * STATE_SAVE_PERIOD) <= currentMillis) 
    if (((stateUpdateCounter * STATE_SAVE_PERIOD) <= currentMillis) & (iaqSensor.iaq >= 25))
    {
      updateEEPROM = true;
      stateUpdateCounter++;
    }

    else
    {
      updateEEPROM = false;
    }
  }

  if (updateEEPROM)
  {
    iaqSensor.getState(bsecState);
    Serial.println(F("Saving state to EEPROM"));

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++)
    {
      EEPROM.write(i + 1, bsecState[i]);
      // Uncomment below to see to-be EEPROM input
      //Serial.println(bsecState[i], HEX);
    }
    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }

  //else
  //{
    //Serial.println(F("Not saved state to EEPROM"));
  //}
  
  esp_task_wdt_delete(NULL);

  vTaskDelete(updateState);
}

// ISR Function for counter 1
static void IRAM_ATTR Counter_ISR(void *arg)
{
  eventTriggerd = true;

  if(user_isr_handle)
  {
    //Free the ISR service handle.
    esp_intr_free(user_isr_handle);
    //user_isr_handle = NULL;
  }
}

static void Init_PulseCounter_01(void)
{
  pcnt_config_t pcnt_config_01 = { };
  pcnt_config_01.pulse_gpio_num = PCNT_INPUT_SIG_IO_01;         // Set pulse input GPIO member
  //pcnt_config_01.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  // What to do on the positive / negative edge of pulse input?
  pcnt_config_01.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
  //pcnt_config_01.neg_mode = PCNT_COUNT_DIS;   // Count down disable

  // What to do when control input is low or high?
  pcnt_config_01.lctrl_mode = PCNT_MODE_KEEP; // Keep the primary counter mode if low
  pcnt_config_01.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode 

  // Set the maximum and minimum limit values to watch
  pcnt_config_01.counter_h_lim  = PCNT_H_LIM_VAL;
  pcnt_config_01.counter_l_lim  = PCNT_L_LIM_VAL;

  pcnt_config_01.unit = PCNT_UNIT_01;                           // Select pulse unit
  pcnt_config_01.channel = PCNT_CHANNEL_0;                      // Select PCNT channel 0
  pcnt_unit_config(&pcnt_config_01);                            // Configure PCNT

  pcnt_set_filter_value(PCNT_UNIT_01, PCNT_FILTER_VAL);         // Maximum filter_val should be limited to 1023.
  pcnt_filter_enable(PCNT_UNIT_01);                             // Enable filter

  pcnt_event_enable(PCNT_UNIT_01, PCNT_EVT_H_LIM);              // Enable event for when PCNT watch point event: Maximum counter value
  pcnt_event_enable(PCNT_UNIT_01, PCNT_EVT_L_LIM);

  pcnt_isr_register(Counter_ISR, NULL, 0, &user_isr_handle);    // Set call back function for the Event
  pcnt_intr_enable(PCNT_UNIT_01);                               // Enable Pulse Counter (PCNT)

  pcnt_counter_pause(PCNT_UNIT_01);                             // Pause PCNT counter
  pcnt_counter_clear(PCNT_UNIT_01);                             // Clear PCNT counter

  pcnt_counter_resume(PCNT_UNIT_01);
  
  Serial.println(F("PCNT_01 Init Completed"));
}

static void Init_PulseCounter_02(void)
{
  pcnt_config_t pcnt_config_02 = { };

  pcnt_config_02.pulse_gpio_num = PCNT_INPUT_SIG_IO_02;         // Set pulse input GPIO member
  //pcnt_config_02.ctrl_gpio_num = PCNT_PIN_NOT_USED;

  // What to do on the positive / negative edge of pulse input?
  pcnt_config_02.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
  //pcnt_config_02.neg_mode = PCNT_COUNT_DIS;   // Count down disable

  // What to do when control input is low or high?
  pcnt_config_02.lctrl_mode = PCNT_MODE_KEEP; // Keep the primary counter mode if low
  pcnt_config_02.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode

  // Set the maximum and minimum limit values to watch
  pcnt_config_02.counter_h_lim  = PCNT_H_LIM_VAL;
  pcnt_config_02.counter_l_lim  = PCNT_L_LIM_VAL;

  pcnt_config_02.unit = PCNT_UNIT_02;                           // Select pulse unit
  pcnt_config_02.channel = PCNT_CHANNEL_1;                      // Select PCNT channel 1
  pcnt_unit_config(&pcnt_config_02);                            // Configure PCNT

  pcnt_set_filter_value(PCNT_UNIT_02, PCNT_FILTER_VAL);         // Set filter value
  pcnt_filter_enable(PCNT_UNIT_02);                             // Enable filter

  pcnt_event_enable(PCNT_UNIT_01, PCNT_EVT_H_LIM);              // Enable event for when PCNT watch point event: Maximum counter value
  pcnt_event_enable(PCNT_UNIT_01, PCNT_EVT_L_LIM);

  pcnt_isr_register(Counter_ISR, NULL, 0, &user_isr_handle);    // Set call back function for the Event
  pcnt_intr_enable(PCNT_UNIT_02);                               // Enable Pulse Counter (PCNT)

  pcnt_counter_pause(PCNT_UNIT_02);                             // Pause PCNT counter
  pcnt_counter_clear(PCNT_UNIT_02);                             // Clear PCNT counter

  pcnt_counter_resume(PCNT_UNIT_02);
  
  Serial.println(F("PCNT_02 Init Completed"));
}

// Function to clean the Counter and its variables
static void Clean_Counters()                                       
{
  pcnt_counter_pause(PCNT_UNIT_01);    // Pause Pulse counters such that we can set event
  pcnt_counter_pause(PCNT_UNIT_02);

  pcnt_counter_clear(PCNT_UNIT_01);    // Clean Pulse Counters
  pcnt_counter_clear(PCNT_UNIT_02);

  pcnt_counter_resume(PCNT_UNIT_01);   // Resume Pulse Counters
  pcnt_counter_resume(PCNT_UNIT_02);
}