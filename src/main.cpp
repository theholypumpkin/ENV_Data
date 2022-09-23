/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <SPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
#include <ArduinoJson.h>
#include <JC_Button.h>
#include <EEPROM.h>
#include <Adafruit_SleepyDog.h>
/*================================================================================================*/
#define RANDOM_SEED_ADC_PIN A0 // NOTE NEVER CONNECT A SENSOR TO THIS PIN
#define CCS_811_INTERRUPT_PIN 7 // 0,1 are UART, 2,3 are i2c so 7 is the only remaining pin 
#define CCS_811_nWAKE 4
#define DHTPIN 5
#define DHTTYPE DHT22
#define EEPROM_CLEAR_BUTTON_PIN 20 //TODO maybe change the pin
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    TRANSMIT_SERIAL,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
uint16_t g_uuid;
volatile bool b_isrFlag = false; //a flag which is flipped inside an isr
/*________________________________________________________________________________________________*/
DHT tempHmdSensor(DHTPIN, DHTTYPE); //Create the DHT object
Adafruit_CCS811 co2Sensor;
/*================================================================================================*/
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    setupEEPROM();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    * Attach Hardware interrupt BEFORE Setting up the CCS 811 sensor, because of a race condition.
    * It cloud happen, that the pin was already driven sow by the sensor, before we attach the
    * interrupt. When the pin is already low, we can no longer detect a falling edge, hence the ISR
    * would never trigger.
    */
    attachInterrupt(digitalPinToInterrupt(CCS_811_INTERRUPT_PIN), setReadFlagISRCallback, FALLING);
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic engine of CCS811
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    tempHmdSensor.begin();
    if(!co2Sensor.begin()){
        while(1){
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500); //When falure blink LED rapidly
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    co2Sensor.enableInterrupt();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    //After ~80 seconds of non responsivness, Watchdog will reset the MCU.*/
    Watchdog.enable(80000);
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
        e_state = READ_DHT_SENSOR; //This is safer than setting the vlaue inside the ISR itself
        b_isrFlag = false;
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
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        if(b_ENVDataCorrection){
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }else{
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = TRANSMIT_SERIAL;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case TRANSMIT_SERIAL:
        if(b_ENVDataCorrection){
            transmitSerial(eco2Value, tvocValue, temperatureValue, humidityValue);
        }else{
            transmitSerial(eco2Value, tvocValue);
        }
        e_state = IDLE; //set the loop to idle until new interrupt triggers a change.
        Watchdog.reset();
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case IDLE:
    default:
        delay(10); //Slow down a little bit
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
    
    Button eepromClearButton(EEPROM_CLEAR_BUTTON_PIN);
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
        //When we have to generate a new UUID generate new seed for Random function
        randomSeed(analogRead(RANDOM_SEED_ADC_PIN));
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
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue,
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
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue){
    
    StaticJsonDocument<100> json; //create a json object //NOTE size of document check
    json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    
    while(!Serial); //Because we have USB Serial, we do not have to begin Serial
    serializeJson(json,Serial);
    Serial.println();
    Serial.flush(); //clear output buffer   
}

/*end of file*/