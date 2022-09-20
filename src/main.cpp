/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <SPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
#include "GP2YDustSensor.h"
#include <ArduinoJson.h>
#include <JC_Button.h>
#include <EEPROM.h>
#include <Adafruit_SleepyDog.h>
/*================================================================================================*/
#define SHARP_LED_PIN 6
#define SHARP_VO_PIN A0 
#define CCS_811_INTERRUPT_PIN 3 //TODO change back to 7 // 0,1 are UART, 2,3 are i2c so 7 is the only remaining pin 
#define CCS_811_nWAKE 4
#define DHTPIN 5
#define DHTTYPE DHT22
#define EEPROM_CLEAR_BUTTON_PIN 20 //TODO maybe change the pin
/*________________________________________________________________________________________________*/
// If you debug using serial and not a hardware debugger you can enable/disable it here
#define DEBUG // TODO comment this line in final build

#ifdef DEBUG
    #define DEBUGPRINT(x) Serial.print(x)
    #define DEBUGPRINTLN(x) Serial.println(x)
    #define DEBUGSERIALBEGIN(x) Serial.begin(x)
#else
    #define DEBUGPRINT(x)
    #define DEBUGPRINTLN(x)
    #define DEBUGSERIALBEGIN(x)
#endif
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    READ_GP2Y_SENSOR,
    TRANSMIT_SERIAL,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
uint16_t g_uuid = 12345, g_loopCount = 0; //TODO remove g_uuid assignment
volatile bool b_isrFlag = false; //a flag which is flipped inside an isr
/*________________________________________________________________________________________________*/
DHT tempHmdSensor(DHTPIN, DHTTYPE); //Create the DHT object
Adafruit_CCS811 co2Sensor;
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN);
/*================================================================================================*/
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic engine of CCS811
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    Serial.begin(9600);
   // setupEEPROM(); //TODO Reanable EEPROM
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    dustSensor.begin();
    tempHmdSensor.begin();
    if(!co2Sensor.begin()){
        while(1){
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500); //When falure blink LED rapidly
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_10SEC);
    co2Sensor.enableInterrupt();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine
    attachInterrupt(digitalPinToInterrupt(CCS_811_INTERRUPT_PIN), setReadFlagISRCallback, FALLING); //NOTE ALTERNATIVE IS LOW
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    //After ~80 seconds of non responsivness, Watchdog will reset the MCU.*/
    //Watchdog.enable(80000); //TODO reenable Watchdog
}
/*________________________________________________________________________________________________*/
/**
 * @brief The main loop.
 * Because the microcontroller has native USB-Serial and we are constantly connected to a power
 * source, we will not sleep, because this would mess with the server software which reads the
 * Serial connection.
 */
void loop() {
    //make static to retain variables even if out of scope.
    static uint16_t eco2Value, tvocValue, dustDensityValue;
    static float temperatureValue, humidityValue, dustSensorBaseline;
    static bool b_ENVDataCorrection;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    if(b_isrFlag){
        if(g_loopCount < 60) {
            //because we trigger this at the start of loop g_loopCount will be 0 after 60 iterations
            g_loopCount++; 
        } else {
            g_loopCount = 0;
        }
        e_state = READ_DHT_SENSOR; //This is safer than setting the vlaue inside the ISR itself
        b_isrFlag = false;
        Serial.println("Interrupt triggered");
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch (e_state)
    {
    case READ_DHT_SENSOR:
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue);
        /* Unnecessary because we don't break but improves readability
        * optimizer will likly remove it anyway
        */
        e_state = READ_CCS_SENSOR;
        Serial.println("State: DHT done");
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        if(b_ENVDataCorrection){
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }else{
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = READ_GP2Y_SENSOR;
        Serial.println("State: CCS done");
        Serial.print("CO2: ");
        Serial.println(eco2Value);
        Serial.print("tvoc: ");
        Serial.println(tvocValue);
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_GP2Y_SENSOR:
        dustDensityValue = readGP2YSensor();
        if(g_loopCount == 0){ //this will occur only after 60 readings ~1h
            dustSensorBaseline = readGPY2SensorBaselineCanidate();
        }
        e_state = TRANSMIT_SERIAL;
        Serial.println("State: GP2Y done");
        Serial.print("Dust Density: ");
        Serial.println(dustDensityValue);
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case TRANSMIT_SERIAL:
        if(b_ENVDataCorrection){
            if(g_loopCount == 0){
                transmitSerial(eco2Value, tvocValue, dustDensityValue, 
                    temperatureValue, humidityValue, dustSensorBaseline);
            }else{
                transmitSerial(eco2Value, tvocValue, dustDensityValue, 
                    temperatureValue, humidityValue);
            }

        }else{
            if(g_loopCount == 0){
                transmitSerial(eco2Value, tvocValue, dustDensityValue, dustSensorBaseline);
            }else{
                transmitSerial(eco2Value, tvocValue, dustDensityValue);
            }
        }
        Serial.println("State: JSON Done");
        e_state = IDLE; //set the loop to idle until new interrupt triggers a change.
        //Watchdog.reset();
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case IDLE:
    default:
        delay(1000); //Slow down a little bit
        Serial.print("Interrupt Pin Value: ");
        Serial.println(digitalRead(CCS_811_INTERRUPT_PIN));
        Serial.print("Analog Value Dust:");
        Serial.println(analogRead(SHARP_VO_PIN));
        break;
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief Set the Read Flag ISR Callback
 * 
 */
void setReadFlagISRCallback(){
    b_isrFlag = true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief writes a random UUID to EEPROM (on SAMD chips to Flash) if never set, else it reads the
 * existing UUID from the EEPROM (or on SAMD chips to Flash). If in the first 5 seconds of the setup
 * a button is pressed, it will clear the flash and generate a new UUID
 */
void setupEEPROM(){
    
    Button eepromClearButton(EEPROM_CLEAR_BUTTON_PIN, 50);
    eepromClearButton.begin();
    unsigned long loopEnd = millis() + 5000;
    
    while(millis() < loopEnd){ //check for 5 seconds if the button is pressed
        eepromClearButton.read();
        if(eepromClearButton.isPressed()){
            // This loop will take about 3.3*256 ms to complete which is about 0.85 seconds.
            for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
                EEPROM.write(i, 0);
            }
            break; //when we reset the eeprom terminate the while loop
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    /* For compability reasons and portability to other microcontroller we will not use the more
     * convinient EEPROM.get(addr, g_uuid); and EEPROM.put(addr, g_uuid); methods. 
     * Instead we read individual bytes in the little endian order as it is standard on the AVR 
     * and common on the ARM Architecture.
     */
    uint16_t addr = 0;
    uint16_t uuidUpperByte = (uint16_t) EEPROM.read(addr + 1) << 8;
    uint16_t uuidLowerByte = (uint16_t) EEPROM.read(addr);
    g_uuid = uuidUpperByte + uuidLowerByte; //leftshift by 8 bit
    /* Under the curcumstance that we had reset the eeprom once all bytes are 0.
     * When we never wrote anything to the EEPROM of the microcontroller all bytes will be FF.
     * Because we have a two byte variable, we have to check of the value not 0 or FFFF
     * 
     * We only generate a random variable between 0x100 and 0xFFFE, to we always use both bytes,
     * but never use the "never written" value of FFFF.
     */
    if((g_uuid == 0x0) || (g_uuid == 0xFFFF)){
        g_uuid = (uint16_t)random(0x100,0xFFFE); //generates random uuid in range of 0x100 to 0xFFFE
        uuidUpperByte = g_uuid >> 8;
        uuidLowerByte = (g_uuid << 8) >> 8; //remove the upper 8 bit
        //EEPROM.put(addr, g_uuid);
        //Write the generated UUID to EEPROM
        EEPROM.write(addr, (uint8_t) uuidLowerByte);
        EEPROM.write(addr + 1, (uint8_t) uuidUpperByte);
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 * 
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the tvoc Value should be saved at
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue){
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic Engine of co2Sensor
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    if(!co2Sensor.readData()){
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine od co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 * 
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the eco2 Value should be saved at
 * @param temperatureValue The current temperature to create a more accurate reading
 * @param humidityValue The current humidity to create a more accurate reading
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue, 
    float temperatureValue, float humidityValue){
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic Engine of co2Sensor
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    co2Sensor.setEnvironmentalData(humidityValue, temperatureValue);
    if(!co2Sensor.readData()){
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine od co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief Get average dust density between numSamples in ug/m3 
 * 
 * @param numberOfSamples The number of samples which should be taken. The default is 20
 * @return uint16_t dust density between 0 and 600 ug/m3 
 */
uint16_t readGP2YSensor(uint16_t numberOfSamples = (uint16_t)20U){
    return dustSensor.getDustDensity(numberOfSamples);
}
/*________________________________________________________________________________________________*/
/**
 * @brief Reads and set the Baseline for the sensor to adjust for sensor-drift over time
 * 
 * @return float the baseline
 */
float readGPY2SensorBaselineCanidate(){
    float baseline = dustSensor.getBaselineCandidate();
    dustSensor.setBaseline(baseline); //Adjusts for sensor-drift
    return baseline;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Gets the Current Temperature and Humidity
 * 
 * @param temperatureValue A Reference where the temperature Value should be saved at
 * @param humidityValue A Reference where the humidity Value should be saved at
 * @return true if the Reading was sucessful
 * @return false if the reading was unsucessful and the read value is 'Not A Number'
 */
bool readDHTSensor(float &temperatureValue, float &humidityValue){
    temperatureValue = tempHmdSensor.readTemperature();
    humidityValue = tempHmdSensor.readHumidity();
    if (isnan(humidityValue) || isnan(temperatureValue)) {
        return false;
    }
    return true;
} 
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 * 
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param dustDensityValue The read Dust Density Value
 * @param temperatureValue The read Temperature Value
 * @param humidityValue The Read Humidity Value 
 * @param dustSensorBaseline The dust Sensor baseline when available
 */
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue,
    float temperatureValue, float humidityValue, float dustSensorBaseline){
    
    StaticJsonDocument<100> json; //create a json object //NOTE size of document check
    json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["temperature"].set(temperatureValue);
    json["fields"]["humidity"].set(humidityValue);
    json["fields"]["dust density"].set(dustDensityValue);
    
    if(g_loopCount == 0) //The value only changes if an interrupt is triggered
        json["fields"]["dust baseline"].set(dustSensorBaseline);
    
    while(!Serial); //Because we have USB Serial, we do not have to begin Serial
    serializeJson(json,Serial);
    Serial.println();
    Serial.flush(); //clear output buffer
       
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 * 
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param dustDensityValue The read Dust Density Value
 * @param dustSensorBaseline The dust Sensor baseline when available
 */
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue,
    float dustSensorBaseline){
    
    StaticJsonDocument<100> json; //create a json object //NOTE size of document check
    json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["dust density"].set(dustDensityValue);
    
    if(g_loopCount == 0) //The value only changes if an interrupt is triggered
        json["fields"]["dust baseline"].set(dustSensorBaseline);
    
    while(!Serial); //Because we have USB Serial, we do not have to begin Serial
    serializeJson(json,Serial);
    Serial.println();
    Serial.flush(); //clear output buffer   
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 * 
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param dustDensityValue The read Dust Density Value
 * @param temperatureValue The read Temperature Value
 * @param humidityValue The Read Humidity Value
 */
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue,
    float temperatureValue, float humidityValue){
    
    StaticJsonDocument<100> json; //create a json object //NOTE size of document check
    json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["temperature"].set(temperatureValue);
    json["fields"]["humidity"].set(humidityValue);
    json["fields"]["dust density"].set(dustDensityValue);
    while(!Serial); //Because we have USB Serial, we do not have to begin Serial
    serializeJson(json,Serial);
    Serial.println();
    Serial.flush(); //clear output buffer
       
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 * 
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param dustDensityValue The read Dust Density Value
 */
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue){
    
    StaticJsonDocument<100> json; //create a json object //NOTE size of document check
    json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["dust density"].set(dustDensityValue);
    while(!Serial); //Because we have USB Serial, we do not have to begin Serial
    serializeJson(json,Serial);
    Serial.println();
    Serial.flush(); //clear output buffer   
}

/*end of file*/