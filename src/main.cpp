#include <Arduino.h>
#include <oled_display.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <i2c_diagnostics.h>
#include <ADS1X15.h>
// #include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <Adafruit_BMP280.h>
#include <INA3221.h>
#include "n2k.h"
#include <RTClib.h>
// #include "pcf8563.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// #define PCF8523_ADDRESS 0x51  

#define I2C_SDA 21
#define I2C_SCL 22

#define ALARM_PIN 25

#define SPLASHDATA "TEST"         // The part number of the sensor
#define SPLASHLABEL "MCC400"  // A label for the splash screen.

// #define ESP32_CAN_TX_PIN GPIO_NUM_4 
// #define ESP32_CAN_RX_PIN GPIO_NUM_2 

// #include <Preferences.h>
// #include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
// #include <N2kMessages.h>
// #include <N2kMaretron.h>

// #define N2K_MANUFACTURER_ID "1" // Manufacturer's Model serial code
// #define N2K_PRODUCT_ID 101 // Manufacturer's product code
// #define N2K_MODEL_ID "MCC400" // Manufacturer's Model ID
// #define N2K_SOFTWARE_VERSION "0.0.1-1 (2023-04-25)" // Manufacturer's Software version code
// #define N2K_HARDWARE_VERSION "0.0.1-1 (2023-04-25)" // Manufacturer's Model version
// /// TODO: multiple function and classes required.
// #define N2k_DEVICE_FUNCTION 130 // Device function = Devices that measure/report temperature
// #define N2k_DEVICE_CLASS 75 // Device class = Sensor Communication Interface
// #define N2k_DEVICE_INDUSTRY_GROUP 2002 // From code list on 
// // Define READ_STREAM to port, where you write data from PC e.g. with NMEA Simulator.
// #define READ_STREAM Serial       
// // Define ForwardStream to port, what you listen on PC side. On Arduino Due you can use e.g. SerialUSB
// #define FORWARD_STREAM Serial    


// Stream *ReadStream=&READ_STREAM;
// Stream *ForwardStream=&FORWARD_STREAM;

void powerUp();

/*
* Pre-define methods for measuring sensors
*/
// uint32_t getDeviceId(void);                         // get a unique identifier for the processor from its MAC address
// void    getNodeAddress(void);                       // Get the node address from the chip id
// // void    displayUpdate(void);                        // Cycle through OLED display screens on cycle set in `DisplayScheduler`
// void    n2KInit(void);                              // initialize the NMEA2000 library
// float   calibrateAdc(float value);                  // calibrate the ADC that measures alternator sensewire output voltage
// void    altSenseAdcMeasure(void);                   // measure alternator sense wire output voltage using ADC
void    ds18b20Measure(void);     
// void    lm75Measure(void);                          // measure temperature using the LM75 sensor on the I2C bus
// void    ina219Measure(void);                        // measure volts, amps, power using the ina219 sensor on the I2C bus
void    i2cDisplay(void);                           // print the addresses of all I2C devices on the two-wire bus
// void    measureLightSensor(void);                   // measure the ambient light levels using the ADC connected to the LDR
void    oneWireBusInit(void);                       // initialize the one-wire bus and devices

void RTC_Init();
void RTC_getTime();
void sendSystemTime();

void BME280_Init();
void BME820_Read();

void ADC_Init();
void ADC_Read();    

void MQ_Init();
void MQ_Read();                  


void INA3221_Init();                 
void INA3221_Read();

void Alarm_Init();                 
void Alarm_Beep(uint16_t duration);

void Flowmeter_Init();
void Flowmeter_Read();

// void    getHouseBatteryStatus(const tN2kMsg &N2kMsg);  // handler for PGN 127508l.
// void    getPosition(const tN2kMsg &N2kMsg);         // handler for PGN 129029, GNSS Position Data

#define ONE_WIRE_BUS 15           // One-wire bus is connected to GPIO15
RTC_PCF8563 rtc;
// PCF8563_Class rtc;

unsigned long delayTime;

// Set I2C address to 0x41 (A0 pin -> VCC)
INA3221 ina3221(INA3221_ADDR40_GND);

Adafruit_BME280  bmp820; // I2C

// Set up a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// initialize ADS1115 on I2C bus 1 with default address 0x48
ADS1115 ADC(0x48);

// initialize ADS1115 on I2C bus 1 with default address 0x48
ADS1115 MQADC(0x49);

// Pass our oneWire reference to Dallas Temperature sensor class instance
DallasTemperature oneWireSensors(&oneWire);

// I2C bus diagnostics instance, used for discovering I2C devices.
TwoWireDiagnostics i2c = TwoWireDiagnostics(Wire);

byte devices[9];          // List of connected I2C device addresses.

volatile uint32_t flowCount = 0;

// variable for storing the pushbutton status 
uint8_t flowState = 0;

void setup() {
  // put your setup code here, to run once:
  // delay (2500);
  // delay (2500);
  Alarm_Init();  
  delay (2500);
  powerUp();  
  Serial.begin(115200);
  Serial.println("Up and running!");
 
  // while (!Serial) {
  //     // will pause Zero, Leonardo, etc until serial console opens
  //     delay(1);
  // }

  Wire.begin(I2C_SDA, I2C_SCL);
  
  oneWireBusInit();
  
  ADC_Init();
  MQ_Init();

  INA3221_Init();

  // Flowmeter_Init();

  BME280_Init();

  n2KInit();

  delay(1000);

    // poll the I2C bus to identify all the devices
  i2c.devices(devices, false);

    // print the list of I2C devices
  i2cDisplay();


  RTC_Init();


  oledInit();
  
  oledSplash(SPLASHLABEL, SPLASHDATA);

delay (500);  
  BME820_Read();
delay(2500);

  // Serial.println("Connected to device");
}

void loop() {
  
  NMEA2000.ParseMessages();
  // put your main code here, to run repeatedly:
  // ds18b20Measure();
  // ADC_Read();
  // Flowmeter_Read();
  INA3221_Read();
  // String flow = String(flowCount);
  // Serial.println(flow);
  // oledWrite("FLOW", flow); 
  // MQ_Read();
  BME820_Read();
  RTC_getTime();
  sendSystemTime();
  delay(3000);
}


void powerUp(){
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);

}


void RTC_Init(){
  int i = 0;
  delay(500);
  // rtc.begin(&Wire, 0x51);
  bool rtcReady = false;
  // while (i<20 && !rtcReady){
  //   rtcReady = rtc.begin(Wire, 0x51);    
  //   i++;
  // }
  // if(!rtcReady){
  //     Serial.println("Failed to initialize RTC");
  //   } else {
  //     rtc.setDateTime(2023, 5, 2, 15, 04, 59);
  //   }
  while (i<20 && !rtcReady) {
    rtcReady = rtc.begin(&Wire);
    delay(100);    
    i++;
  }
  
  if (!rtcReady ) {
        Serial.println("Couldn't find RTC");
        Serial.flush();
      
  } else {
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    if (rtc.lostPower()) {
        Serial.println("RTC is NOT initialized, let's set the time!");
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
        //
        // Note: allow 2 seconds after inserting battery or applying external power
        // without battery before calling adjust(). This gives the PCF8523's
        // crystal oscillator time to stabilize. If you call adjust() very quickly
        // after the RTC is powered, lostPower() may still return true.
      }

      // When time needs to be re-set on a previously configured device, the
      // following line sets the RTC to the date & time this sketch was compiled
      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      //  rtc.adjust(DateTime(2023, 6, 5, 12, 39, 0));

      // When the RTC was stopped and stays connected to the battery, it has
      // to be restarted by clearing the STOP bit. Let's do this to ensure
      // the RTC is running.
      rtc.start(); 
  }

}

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void sendSystemTime(){
  DateTime now = rtc.now();
  tN2kMsg N2kMsg; 
  unsigned char SID; 
  uint16_t SystemDate = uint16_t(now.unixtime()/60/60/24);
  
  tN2kTimeSource TimeSource=N2ktimes_LocalCrystalClock;
  double SystemTime = double (now.unixtime()-uint32_t(SystemDate) * 60 *60 *24);
  Serial.printf("%u days since 1/1/1970, and %u seconds since midnight\n", 
      SystemDate, uint32_t(SystemTime));
  SetN2kSystemTime(N2kMsg,SID,SystemDate,SystemTime,TimeSource);
  // tN2kTimeSource TimeSource=N2ktimes_GPS
  NMEA2000.SendMsg(N2kMsg);
    Serial.println("Sent PGN" + String(N2kMsg.PGN));

}

void RTC_getTime(){
      // RTC_Date time = rtc.getDateTime();
      // Serial.println(time.hour);
      // Serial.println(rtc.formatDateTime(PCF_TIMEFORMAT_YYYY_MM_DD_H_M_S));

    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    // Serial.print(" since midnight 1/1/1970 = ");
    // Serial.print(now.unixtime());
    // Serial.print("s = ");
    // Serial.print(now.unixtime() / 86400L);
    // Serial.println("d");

    // // calculate a date which is 7 days, 12 hours and 30 seconds into the future
    // DateTime future (now + TimeSpan(7,12,30,6));

    // Serial.print(" now + 7d + 12h + 30m + 6s: ");
    // Serial.print(future.year(), DEC);
    // Serial.print('/');
    // Serial.print(future.month(), DEC);
    // Serial.print('/');
    // Serial.print(future.day(), DEC);
    // Serial.print(' ');
    // Serial.print(future.hour(), DEC);
    // Serial.print(':');
    // Serial.print(future.minute(), DEC);
    // Serial.print(':');
    // Serial.print(future.second(), DEC);
    // Serial.println();

    Serial.println();
    // delay(3000);

}


void MQ_Init(){
  if (MQADC.begin()){
    MQADC.setMode(1);
    MQADC.setGain(2);
    MQADC.requestADC(3);
    Serial.println("Max MQADC Volgate " + String(MQADC.getMaxVoltage()));
  } else {
    Serial.println("Error connecting to MQADC");
  }
}

void MQ_Read(){  
  int16_t val_3 = MQADC.readADC(0);  
  float f = MQADC.toVoltage(2);  // voltage factor
  float tank = val_3 * f ;;//*100 / 1.9;
  tank = tank > 100? 100: tank;
  oledWrite("LPG Level", String(tank,2) + "PPM");
  if (tank < 10){
  Alarm_Beep(2000);}
  delay(2000);
}

void Flowmeter_Init(){
  pinMode(35, INPUT);
  attachInterrupt(35, Flowmeter_Read, RISING);
  // interrupts();
}

void Flowmeter_Read(){
  // flowState = digitalRead(35);
  // Serial.println(flowState);
  // if (flowState== LOW){
    flowCount++;
    // String flow = String(flowCount);
    // Serial.println(flow);
    // oledWrite("FLOW", flow); 
    // delay(25);
  }


void Alarm_Init(){
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, HIGH);
  delay(2500);
  digitalWrite(ALARM_PIN, LOW);


} 


void Alarm_Beep(uint16_t duration){
  uint64_t timestamp = millis();
    while (millis() - timestamp < duration)
    {
      digitalWrite(ALARM_PIN, HIGH);
    }
    Alarm_Init();
}



void INA3221_Init(){

    ina3221.begin();
    ina3221.reset();

    // Set shunt resistors to 10 mOhm for all channels
    ina3221.setShuntRes(470, 470, 470);

    // Set series filter resistors to 10 Ohm for all channels.
    // Series filter resistors introduce error to the current measurement.
    // The error can be estimated and depends on the resitor values and the bus
    // voltage.
    ina3221.setFilterRes(10, 10, 10);
}               

void INA3221_Read(){
  float current;
    float current_compensated;
    float voltage;

    current             = ina3221.getCurrent(INA3221_CH1);
    current_compensated = ina3221.getCurrentCompensated(INA3221_CH1);
    voltage             = ina3221.getVoltage(INA3221_CH1);

    Serial.print("Channel 1: \n Current: ");
    Serial.print(current, 3);
    Serial.print("A\n Compensated current: ");
    Serial.print(current_compensated, 3);
    Serial.print("\n Voltage: ");
    Serial.print(voltage, 3);
    Serial.println("V");

    
    oledWrite("VOLTS" , String(voltage,1)+ " V");    
    delay(500);
    oledWrite("AMPS" , String(current*1000,0)+ " mA");    
    delay(500);
    oledWrite("AMPS (C)" , String(current_compensated*1000,0)+ " mA");    
    delay(500);

}


void BME280_Init(){  
    unsigned status;
    
    // default settings
    status = bmp820.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bmp820.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
       delay(2500);
    } else {
      
    }
}

void BME820_Read() {

    float temp = bmp820.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temp, 2);
    Serial.println(" °C");
    // oledWrite("AMBIENT" , String(temp,1)+ " C");
    delay(1500);

    double press = double(bmp820.readPressure());
    Serial.print("Pressure = ");
    Serial.print(press);
    Serial.println(" hPa");
    // oledWrite("BAROMETER" , String(press,1)+ " hPa");
    // delay(1500);

    float alt = bmp820.readAltitude(SEALEVELPRESSURE_HPA);
    // Serial.print("Approx. Altitude = ");
    // Serial.print(alt);
    // Serial.println(" m");    
    // oledWrite("ALTITUDE" , String(alt,0)+ " m");
    oledWrite(String(temp,1)+ " C" , String(PascalTohPA(press),1)+ " hPa");

    tN2kMsg N2kMsg;
    SetN2kEnvironmentalParameters(N2kMsg, 
                                  1,
                                  N2kts_MainCabinTemperature, 
                                  CToKelvin(temp),
                                  N2khs_InsideHumidity,
                                  70,
                                  press);

    NMEA2000.SendMsg(N2kMsg);
    Serial.println("Sent PGN" + String(N2kMsg.PGN));
    // delay(1500);

    // Serial.print("Humidity = ");
    // // Serial.print(bmp820.readHumidity());
    // Serial.println(" %");

    // Serial.println();
}

void ADC_Init(){
  if (ADC.begin()){
    ADC.setMode(1);
    ADC.setGain(2);
    ADC.requestADC(3);
    Serial.println("Max ADC Voltage" + String(ADC.getMaxVoltage()));
  } else {
    Serial.println("Error connecting to ADC");
  }
}

void ADC_Read(){  
  int16_t val_3 = ADC.readADC(3);  
  float f = ADC.toVoltage(1);  // voltage factor
  float tank = val_3 * f *100 / 1.9;
  tank = tank > 100? 100: tank;
  oledWrite("TANK 4", String(tank,0) + "%");
  if (tank < 10){
  Alarm_Beep(2000);}
  delay(2000);
}

void i2cDisplay(){
  // loop through the devices
  for (byte i = 0; i<9; i++){
    byte device = devices[i];
    if (device!=0x00){
      // display the address for each I2C device
      String address = i2c.getAddressString(device);
      // oledWrite("I2C Device at ", address);
      Serial.println("I2C Device at " + String(address));
      // show the address for 2 seconds
      // delay(500);
    }
  }
}

void oneWireBusInit(){
  uint8_t address;

  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  
  // activate the one-wire sensors
  oneWireSensors.begin();

  // print the number of one-wire sensors attached to the one-wire bus
  Serial.println("1-Wire device count: " + String(oneWireSensors.getDeviceCount()));

  oneWireSensors.getAddress(&address,0); // read the address of the device at index 0;
  Serial.println("DS18B20 found at address: " + String(address, HEX));

  // // Set the status bit for the one-wire bus
  // ECU_STATUS = ECU_STATUS | ECU_DS18B20;
}

void ds18b20Measure(){
  oneWireSensors.requestTemperatures();

  oledWrite("TEMP_0", String(oneWireSensors.getTempCByIndex(0)));
  delay(1000);
  // pushPop(oneWireSensors.getTempCByIndex(0), alternatorTemp_arr, 10);
  // alternatorTemp = average(alternatorTemp_arr, 10);
}