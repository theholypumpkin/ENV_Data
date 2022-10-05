/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
#include <ArduinoJson.h>
#include <JC_Button.h>
#include <PubSubClient.h>
#include <math.h>
#include <RTCZero.h> //Also inplements sleep
/* SAMD21. SAMD51 chips do not have EEPROM. This library provides an EEPROM-like API and hence
 * allows the same code to be used.
 */
#include <FlashAsEEPROM.h>
/*================================================================================================*/
#define RANDOM_SEED_ADC_PIN A0 // NOTE: NEVER CONNECT A SENSOR TO THIS PIN
#define BATTERY_VOLTAGE_ADC_PIN A1
#define CCS_811_INTERRUPT_PIN 13
#define CCS_811_nWAKE 16
#define DHTPIN 21
#define DHTTYPE DHT22
#define EEPROM_CLEAR_BUTTON_PIN 20
#define DOCUMENT_SIZE 255
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    READ_BATTERY,
    PUBLISH_MQTT,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
uint8_t g_lastRtcUpdateDay;
uint16_t g_uuid;
const float MAX_BATTERY_VOLTAGE = 4.2,
            ADC_VOLTAGE_FACTOR = MAX_BATTERY_VOLTAGE / powf(2.0, ADC_RESOLUTION);

volatile bool b_isrFlag = false; // a flag which is flipped inside an isr
/*________________________________________________________________________________________________*/
RTCZero rtc;
DHT tempHmdSensor(DHTPIN, DHTTYPE); // Create the DHT object
Adafruit_CCS811 co2Sensor;
WiFiClient wifiClient;
PubSubClient mqttClient(g_mqttServerUrl, g_mqttServerPort, wifiClient);
/*================================================================================================*/
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    analogReadResolution(ADC_RESOLUTION);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    setupEEPROM();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    Serial1.begin(115200);            // Use Hardware Serial (not USB Serial) to debug
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic engine of CCS811
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    tempHmdSensor.begin();
    if (!co2Sensor.begin())
    {
        while (1)
        {
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500); // When falure blink LED rapidly
            Serial1.println("CCS Error");
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    rtc.begin(); //begin the rtc at "random time" probably  Jan 1 2000 at 00:00:00 o'clock
    g_lastRtcUpdateDay = rtc.getDay();
    if(updateNetworkTime()){ //set real time acording to network
        Serial1.println("Network Time successful");
    }else{
        Serial1.println("ERROR: No Network Time");
    }
    rtc.setAlarmSeconds(0);
    rtc.enableAlarm(rtc.MATCH_SS); //Set Alarm every full minute (when seconds are 0)
    rtc.attachInterrupt(alarmISRCallback); //When alarm trigger this callback.
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delay(10000); // give us some time to upload a new program
}
/*________________________________________________________________________________________________*/
/**
 * @brief The main loop.
 * Because the microcontroller has native USB-Serial and we are constantly connected to a power
 * source, we will not sleep, because this would mess with the server software which reads the
 * Serial connection.
 */
void loop()
{
    // make static to retain variables even if out of scope.
    static uint16_t eco2Value, tvocValue;
    static float temperatureValue, humidityValue, batteryPercentage, batteryVoltage;
    static bool b_ENVDataCorrection;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    if (b_isrFlag)
    {
        e_state = READ_DHT_SENSOR; // This is safer than setting the vlaue inside the ISR itself
        b_isrFlag = false;
    }
    if(g_lastRtcUpdateDay != rtc.getDay()){ //update network time daily
        if(updateNetworkTime()){ //set real time acording to network
            Serial1.println("Network Time successful");
        }else{
            Serial1.println("ERROR: No Network Time");
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch(e_state){
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case IDLE:
        //Go to sleep until Alarm wakes you
        rtc.standbyMode();
        break;
    case READ_DHT_SENSOR:
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue);
        /* Unnecessary because we don't break but improves readability
         * optimizer will likly remove it anyway
         */
        e_state = READ_CCS_SENSOR;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        if (b_ENVDataCorrection)
        {
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }
        else
        {
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = READ_BATTERY;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_BATTERY:
        batteryVoltage = analogRead(BATTERY_VOLTAGE_ADC_PIN) * ADC_VOLTAGE_FACTOR;
        batteryPercentage = calcBatteryPercentage(batteryVoltage);
        e_state = PUBLISH_MQTT;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case PUBLISH_MQTT:
        /*//Enable the NINA WiFi Modul by setting the internal pin high!
        digitalWrite(27, HIGH);
        delay(10);*/
        //Configure static networking and set hostname for this paticular request  
        WiFi.config(g_staticIPAddress, g_DNSAddress, g_gateway, g_subnet);
        WiFi.setHostname(g_name);

        //esablishing wifi connection
        while(WiFi.begin(g_wifiSsid, g_wifiPass) != WL_CONNECTED){ //retry connect indef
            delay(1000);
            //TODO Wire gpio to reset pin and triger it after some time
        }
        
        //reconnect To mqtt (returns imidiatly if already connected)
        mqttReconnect();
        long wifiSignalStrength = WiFi.RSSI();
        if (b_ENVDataCorrection)
        {
            publishMQTT(eco2Value, tvocValue, wifiSignalStrength, temperatureValue, 
            humidityValue, batteryVoltage, batteryPercentage);
        }
        else
        {
            publishMQTT(eco2Value, tvocValue, wifiSignalStrength, batteryVoltage, 
            batteryPercentage);
        }
        mqttClient.loop();
        e_state = IDLE;
        WiFi.lowPowerMode(); //maybe unnecessary but won't hurt
        WiFi.end(); //We cant stay connected to WiFi it drains 30mA in Low Power Mode and 100mA w/o LP
        /*delay(10);
        digitalWrite(27, LOW); //Pull Reset of Nina WiFi Low to disable it and save even more Battery*/
        break;
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief Set the Read Flag ISR Callback
 */
void alarmISRCallback()
{
    b_isrFlag = true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Connects to WiFi using a static IP-Address, DNS, Gateway and Subnet than
 * it fetchs the latest network time and set the Real Time Clock to this time.
 * Afterwards it disconnects from wifi to conserve battery.
 */
bool updateNetworkTime(){
    /*//Enable the NINA WiFi Modul by setting the internal pin high!
    digitalWrite(27, HIGH);
    delay(10);*/
    //Configure static networking and set hostname for this paticular request
    WiFi.config(g_staticIPAddress, g_DNSAddress, g_gateway, g_subnet);
    WiFi.setHostname(g_name);

    uint8_t connection_attempts = 0;
    while(WiFi.begin(g_wifiSsid, g_wifiPass) != WL_CONNECTED){ //retry connect
        connection_attempts++;
        if(connection_attempts == 4){
            return false; //return early
        }
        delay(1000);
    }
    WiFiUDP ntpUdpObject;
    NTPClient ntpClient(ntpUdpObject, g_ntpTimeServerURL);
    ntpClient.begin();
    connection_attempts = 0;
    while(!ntpClient.update()){ //attempt to connect to ntp server up to 5 times.
        connection_attempts++;
        if(connection_attempts == 4){
            WiFi.end();
            return false; //return early
        }
        delay(1000);
    }
    unsigned long epochTime = ntpClient.getEpochTime();
    rtc.setEpoch(epochTime);
    WiFi.lowPowerMode(); //maybe unnecessary but won't hurt
    WiFi.end(); //We cant stay connected to WiFi it drains 30mA in Low Power Mode and 100mA w/o LP
    g_lastRtcUpdateDay = rtc.getDay(); //set to actual day
    /*delay(10);
    digitalWrite(27, LOW); //Pull Reset of Nina WiFi Low to disable it and save even more Battery*/
    return true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Attempt to reconnect to mqtt Server
 * 
 */
void mqttReconnect() {
  // Loop until we're reconnected
    while (!mqttClient.connected()) {
        if (!mqttClient.connect(g_name, g_mqttUsername, g_mqttPassword)){
            Serial1.print("failed, rc=");
            Serial1.print(mqttClient.state());
            Serial1.println(" try again in 5 seconds");
            // Wait 1 seconds before retrying
            delay(1000);
        }
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief writes a random UUID to EEPROM (on SAMD chips to Flash) if never set, else it reads the
 * existing UUID from the EEPROM (or on SAMD chips to Flash). If in the first 5 seconds of the setup
 * a button is pressed, it will clear the flash and generate a new UUID
 */
void setupEEPROM()
{

    Button eepromClearButton(EEPROM_CLEAR_BUTTON_PIN);
    eepromClearButton.begin();
    unsigned long loopEnd = millis() + 5000;
    while (millis() < loopEnd)
    { // check for 5 seconds if the button is pressed
        eepromClearButton.read();
        if (eepromClearButton.isPressed())
        {
            // This loop will take about 3.3*256 ms to complete which is about 0.85 seconds.
            for (uint16_t i = 0; i < EEPROM.length(); i++)
            {
                EEPROM.write(i, 0);
            }
            break; // when we reset the eeprom terminate the while loop
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    /* For compability reasons and portability to other microcontroller we will not use the more
     * convinient EEPROM.get(addr, g_uuid); and EEPROM.put(addr, g_uuid); methods.
     * Instead we read individual bytes in the little endian order as it is standard on the AVR
     * and common on the ARM Architecture.
     */
    uint16_t addr = 0;
    uint16_t uuidUpperByte = (uint16_t)EEPROM.read(addr + 1) << 8;
    uint16_t uuidLowerByte = (uint16_t)EEPROM.read(addr);
    g_uuid = uuidUpperByte + uuidLowerByte; // leftshift by 8 bit
    /* Under the curcumstance that we had reset the eeprom once all bytes are 0.
     * When we never wrote anything to the EEPROM of the microcontroller all bytes will be FF.
     * Because we have a two byte variable, we have to check of the value not 0 or FFFF
     *
     * We only generate a random variable between 0x100 and 0xFFFE, to we always use both bytes,
     * but never use the "never written" value of FFFF.
     */
    if ((g_uuid == 0x0) || (g_uuid == 0xFFFF))
    {
        // When we have to generate a new UUID generate new seed for Random function
        randomSeed(analogRead(RANDOM_SEED_ADC_PIN));
        g_uuid = (uint16_t)random(0x100, 0xFFFE); // generates random uuid in range of 0x100 to 0xFFFE
        uuidUpperByte = g_uuid >> 8;
        uuidLowerByte = (g_uuid << 8) >> 8; // remove the upper 8 bit
        // EEPROM.put(addr, g_uuid);
        // Write the generated UUID to EEPROM
        EEPROM.write(addr, (uint8_t)uuidLowerByte);
        EEPROM.write(addr + 1, (uint8_t)uuidUpperByte);
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 *
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the tvoc Value should be saved at
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue)
{
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic Engine of co2Sensor
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    if (!co2Sensor.readData())
    {
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine od co2_sensor to save power
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
                   float temperatureValue, float humidityValue)
{
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic Engine of co2Sensor
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    co2Sensor.setEnvironmentalData(humidityValue, temperatureValue);
    uint8_t before = rtc.getSeconds();
    while(!co2Sensor.available()){delay(10);} //due to clock drifing we loop until data is available
    uint8_t after = rtc.getSeconds();
    if (!co2Sensor.readData())
    {
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine od co2_sensor to save power
    /* If we idle very long because the CO2 Sensor internal clock and the alarm clock are out of
     * sync, we set the alarm to the closest second, so we can maximize sleep
     */
    if (before != after){ //When we idle very long
        //rtc.detachInterrupt();
        //rtc.disableAlarm();
        rtc.setAlarmSeconds(after);
        //rtc.enableAlarm(rtc.MATCH_SS);
        //rtc.attachInterrupt(alarmISRCallback);
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief Gets the Current Temperature and Humidity
 *
 * @param temperatureValue A Reference where the temperature Value should be saved at
 * @param humidityValue A Reference where the humidity Value should be saved at
 * @return true if the Reading was sucessful
 * @return false if the reading was unsucessful and the read value is "Not A Number"
 */
bool readDHTSensor(float &temperatureValue, float &humidityValue)
{
    temperatureValue = tempHmdSensor.readTemperature();
    humidityValue = tempHmdSensor.readHumidity();
    if (isnan(humidityValue) || isnan(temperatureValue))
    {
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
 * @param rssi The WiFi Signal Strength
 * @param dustDensityValue The read Dust Density Value
 * @param dustSensorBaseline The dust Sensor baseline when available
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssi, 
float voltage, float percentage)
{

    StaticJsonDocument<200> json; // create a json object //NOTE size of document check
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["Battery Voltage"].set(voltage);
    json["fields"]["Battery Percentage"].set(percentage);
    json["fields"]["WiFi RSSI"].set(rssi);
    // using a buffer speeds up the mqtt publishing process by over 100x
    char mqttJsonBuffer[100];
    size_t n = serializeJson(json, mqttJsonBuffer); // saves a bit of time when publishing
    mqttClient.publish(g_indoorAirQualityTopic, mqttJsonBuffer, n);
    Serial1.println(mqttJsonBuffer);
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 *
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param rssi The WiFi Signal Strength
 * @param dustDensityValue The read Dust Density Value
 * @param temperatureValue The read Temperature Value
 * @param humidityValue The Read Humidity Value
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssi,
                 float temperatureValue, float humidityValue, float voltage, float percentage)
{
    StaticJsonDocument<DOCUMENT_SIZE> json; // create a json object //NOTE size of document check
    // json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["temperature"].set(temperatureValue);
    json["fields"]["humidity"].set(humidityValue);
    json["fields"]["Battery Voltage"].set(voltage);
    json["fields"]["Battery Percentage"].set(percentage);
    json["fields"]["WiFi RSSI"].set(rssi);
    // using a buffer speeds up the mqtt publishing process by over 100x
    char mqttJsonBuffer[DOCUMENT_SIZE];
    size_t n = serializeJson(json, mqttJsonBuffer);
    mqttClient.publish(g_indoorAirQualityTopic, mqttJsonBuffer, n);
    Serial1.println(mqttJsonBuffer);
}
/*________________________________________________________________________________________________*/
/**
 * @brief Calculate the battery percentage acording to two differnet formulas depending on the
 * voltage read by the ADC.
 *
 * @param adcValue the integer Reading of the ADC across the voltage divider. The factor to convert
 * the value is calculated only once on setup.
 * @return float the battery percantage based on the formula y = 143*sqrt(x-3.71) for change above
 * 16% and abs(30*pow(abs(x-3.724),1/6)-25.8) for charge below 16% or 3.723 Volts
 */
float calcBatteryPercentage(float voltage)
{
    if (voltage < 3.723)
    {
        float absRadiant = abs(voltage - 3.724f);
        float sixthRoot = powf(absRadiant, 0.1666f);
        float percentage = 30.0f * sixthRoot - 25.8f;
        float absPercentage = abs(percentage);
        return roundf(absPercentage);
    }
    else
    {
        return 143 * sqrtf(voltage - 3.71f);
    }
}
/*end of file*/